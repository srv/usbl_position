#!/usr/bin/env python3

import roslib
import rospy
from std_srvs.srv import Empty, EmptyResponse
import tf
import datetime
import numpy as np
import PyKDL
import sys
from cola2_msgs.msg import GoalDescriptor, NavSts
from cola2_msgs.srv import Goto, GotoRequest

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


from evologics_ros_sync.msg import EvologicsUsbllong
from data import Data


class USBLAutoCalibration:
    """ Auto Calibration node for the USBL """

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.calibration_file_path  = self.get_param('usbl_calibration/calibration_file_path')
        self.world_frame  = 'world'
        self.usbl_frame  = 'xiroi/usbl'
        self.modem_frame  = 'turbot/modem'
        self.min_samples = 50

        # Sevice server
        self.start_srv = rospy.Service('navigation/start_usbl_calibration',
                                       Goto,
                                       self.start_usbl_calibration)

        # Object
        self.listener = tf.TransformListener()
        self.xyz_data = Data(name, self.calibration_file_path, 'xyz', np.array(['x','y','z','xp','yp','zp']))

        # Subscribers
        usbl_sub = rospy.Subscriber('usbllong', EvologicsUsbllong, self.usbl_callback, queue_size=1)
        nav_sub = rospy.Subscriber('nav_sts', NavSts, self.nav_callback, queue_size=1)

        # Show message
        rospy.loginfo('[%s]: initialized', self.name)


    def nav_callback(self, nav):
        self.last_nav = nav


    def usbl_callback(self, usbl_msg):
        if self.record_usbl:
            # Get TF from usbl to modem
            T_usbl_modem = self.get_transform_mat(self.usbl_frame, self.modem_frame)
            print(T_usbl_modem)
            xyz_p = np.array([T_usbl_modem[0][3],
                            T_usbl_modem[1][3],
                            T_usbl_modem[2][3]])

            # Get USBL measurement
            xyz = np.array([usbl_msg.N,
                            usbl_msg.E,
                            usbl_msg.D, 
                            T_usbl_modem[0][3],
                            T_usbl_modem[1][3],
                            T_usbl_modem[2][3]])
            self.xyz_data.append(xyz)

        if not self.usbl_received: self.usbl_received = True

    def start_usbl_calibration(self, req):
        """ Main method """
        rospy.loginfo('[%s]: calling start usbl calibration service', self.name)

        #Enable start calibration maneuver.
        self.call_empty_srv('/control/enable_thrusters')
        pose = self.ned.geodetic2ned([req.north_lat,
                                      req.east_lon,
                                      0.0])

        # Perform star maneuver
        text = '{0}: perform start maneuver at ({1}, {2}), depth {3} in a {4}m radius start of {5} sides.'.format(self.name, pose[0], pose[1], pose[2], self.radius, self.sides)
        rospy.loginfo(text)
        self.perform_star_maneuver(pose, self.radius, self.sides)

        # Call service to compute calibration
        rospy.loginfo('[%s]: calling compute usbl calibration service', self.name)
        self.compute_magnetometer_calibration()

        rospy.loginfo('[%s]: finished', self.name)
        return True


    def perform_star_maneuver(self, pose, radius, sides):
        """Perform star maneuver."""
        x, y, z = pose
        self.transit_to([x, y, 0.0])
        angle_error_sum = 0.0
        angle_delta = 2.0 * math.pi * math.floor(sides / 2) / sides
        angle = 0.0
        x1 = x + radius
        y1 = y
        self.record_usbl = True
        for i in range(sides):
            angle_new = angle + angle_delta
            x2 = x + math.cos(angle_new) * radius
            y2 = y + math.sin(angle_new) * radius
            self.star_segment(x1, y1, x2, y2, self.depth)
            desired = math.atan2(y2 - y1, x2 - x1)
            achieved = math.atan2(self.last_nav.position.east - y1,
                                  self.last_nav.position.north - x1)
            error = math.degrees(utils.wrapAngle(achieved - desired))
            angle_error_sum = angle_error_sum + error
            rospy.loginfo('%s: transect error = %s degrees',
                          self.name, str(error))
            angle = angle_new
            x1 = x2
            y1 = y2
        rospy.loginfo('%s: final mean error = %s degrees',
                      self.name, str(angle_error_sum / float(sides)))
        self.transit_to([x,y,0.0])
        self.record_usbl = False
        rospy.loginfo('%s: done', self.name)


    def transit_to(self, pose):
        """Goto to position x, y, z, at velocity vel."""
        goto_request = GotoRequest()
        goto_request.north_lat = pose[0]
        goto_request.east_lon = pose[1]
        goto_request.z = pose[2]
        goto_request.altitude_mode = False
        goto_request.tolerance = self.tolerance
        self.goto_srv(goto_request)
        rospy.sleep(1.0)

    def star_segment(self, x1, y1, x2, y2, depth):
        """ Perform an start segment submerged taking GPS before and after."""
        self.transit_to([x1, y1, 0.0])
        rospy.sleep(1.0)
        self.transit_to([x1, y1, depth])
        rospy.sleep(1.0)
        self.transit_to([x2, y2, depth])
        rospy.sleep(1.0)
        self.transit_to([x2, y2, 0.0])
        rospy.sleep(1.0)

    def call_empty_srv(self, topic):
        # Enable thrusters if not enabled
        try:
            rospy.wait_for_service(topic, 10)  # Wait 1s
            self.service = rospy.ServiceProxy(topic, Empty)
            self.service()
            rospy.loginfo('[%s]: successful call to service %s',
                          self.name, topic)
        except:
            rospy.logerr('[%s]: unable to call service %s',
                         self.name, topic)

    def compute_magnetometer_calibration(self):
        """ Compute magnetometer calibration service callback """
        rospy.loginfo('[%s]: computing calibration', self.name)
        # Verify data
        self.verify_magnetometer_calibration_data()
        # Get calibration data
        self.data = self.xyz_data.get()
        # Build equations
        print(data.shape())
        self.affine_calibration_2d()
        

    def affine_calibration_2d(self, data):
        """ Compute affine calibration in 2d """
        d = 2
        try:
            self.get_affine_matrices(d)
            np.linalg.lstsq(self.M, self.q)
        except:
            rospy.logerr('[%s]: error in self.affine_calibration_2d', self.name)

    # def affine_calibration_3d(self, q, Q):
    #     """ Compute affine calibration in 3d """
    #     try:

    #     except:
    #         rospy.logerr('[%s]: error in self.affine_calibration_3d', self.name)

    def get_affine_matrices(self, d=2):
        M = np.zeros([d,(d+1)*3])
        q = np.zeros([d,1])
        for sample in data:
            q = sample[0:d]
            for i in range(d):
                M[i][d*i:d*(i+1)] = sample[d:]
                M[i][d*(i+1)+1] = 1
        if q.shape[0] < d: # Not initializated
            self.q = q
            self.M = M
        else:
            self.q.append(q)     
            self.M.append(M)     
    
    # def projective_calibration_2d(self, ):
    #     """ Compute projective calibration in 2d """
    #     try:

    #     except:
    #         rospy.logerr('[%s]: error in self.projective_calibration_2d', self.name)

    # def projective_calibration_3d(self, ):
    #     """ Compute projective calibration in 3d """
    #     try:

    #     except:
    #         rospy.logerr('[%s]: error in self.projective_calibration_3d', self.name)

    # def verify_magnetometer_calibration_data(self):
    #     """ Verify data """
    #     # Verify length
    #     if self.xyz.get_size() < self.min_samples:
    #         rospy.logwarn('[%s]: not enough data to use in calibration', self.name)
    #         return


    def print_magnetometer_results_to_file(self):
        try:
            """ Print results to file """
            _file = open(self.calibration_file_path + '/imu_calibration.txt', 'w')
            _now = datetime.datetime.now()
            _header = '\n------------------------------------------------------------------------------------------\nMAGNETOMETER CALIBRATION, ' + str(_now) + '\n\n'
            _mat_results = 'Soft iron:\n' + str(self.imu_frame_S) + '\n\nHard iron:\n' + str(self.imu_frame_H) + '\n\n'
            _num_results = 'navigator/soft_iron: [' + str(self.imu_frame_S[0, 0]) + ', ' + str(self.imu_frame_S[0, 1]) + ', ' + str(self.imu_frame_S[0, 2]) + ', ' + str(self.imu_frame_S[1, 0]) + ', ' + str(self.imu_frame_S[1, 1]) + ', ' + str(self.imu_frame_S[1, 2]) + ', ' + str(
                self.imu_frame_S[2, 0]) + ', ' + str(self.imu_frame_S[2, 1]) + ', ' + str(self.imu_frame_S[2, 2]) + ']\nnavigator/hard_iron: [' + str(self.imu_frame_H[0, 0]) + ', ' + str(self.imu_frame_H[1, 0]) + ', ' + str(self.imu_frame_H[2, 0]) + ']\n\n\n'
            _data = 'Data:\nMX:\n' + str(self.mx) + '\n\nMY:\n' + str(self.my) + '\n\nMZ:\n' + str(self.mz) + '\n\nAX:\n' + str(self.ax) + '\n\nAY:\n' + str(self.ay) + '\n\nAZ:\n' + str(self.az) + '\n\nFILTERED_MX:\n' + str(self.filtered_tilt_corrected_mx) + '\n\nFILTERED_MY:\n' + str(self.filtered_tilt_corrected_my) + '\n\nFILTERED_MZ:\n' + str(self.filtered_tilt_corrected_mz) + '\n\n'
            _string = _header + _mat_results + _num_results + _data
            _file.write(_string)
            _file.close()

            fmt = '%Y%m%d%H%M%S'
            now_str = datetime.datetime.now().strftime(fmt)
            _file = open(self.calibration_file_path + '/ned_origin_'+str(now_str)+'.yaml', 'w')
            _string = 'navigator/ned_origin_lat: ' + str(self.ned_origin_lat) + '\nnavigator/ned_origin_lon: ' + str(self.ned_origin_lon) + '\n' + _num_results
            _file.write(_string)
            _file.close()
        except:
            rospy.logerr('[%s]: error printing results to file', self.name)


    def print_magnetometer_results_to_image(self):
        try:
            """ Print results to image (plot) """
            # TODO: optimize this method and improve variable names
            _now = datetime.datetime.now()
            fig = plt.figure()
            fig.canvas.manager.set_window_title('USBL calibration: ' + str(_now))
            sub_plot = fig.add_subplot(111, aspect='equal')

            # Without correction
            sub_plot.plot(self.data[:,0:2], marker='o', color='r', ls='', label='raw')  # Red -> raw data

            # With correction
            sub_plot.plot(self.data[:,0:2], marker='o', color='g', ls='', label='corrected')  # Green -> correction


            sub_plot.set_title('USBL calibration')

            sub_plot.set_xlabl('X axis [gauss]')
            sub_plot.set_ylabel('Y axis [gauss]')
            sub_plot.legend()
            plt.savefig(self.calibration_file_path + '/plot.png')
        except:
            rospy.logerr('[%s]: error printing image to file', self.name)

    def get_param(self, param_name, default = None):
        if rospy.has_param(param_name):
            param_value = rospy.get_param(param_name)
            return param_value
        elif default is not None:
            return default
        else:
            rospy.logfatal('[%s]: invalid parameters for %s in param server!', self.name, param_name)
            rospy.logfatal('[%s]: shutdown due to invalid config parameters!', self.name)
            exit(0)

    def get_transform_mat(self, A, B):
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(A, B, now, rospy.Duration(10.0))
            trans, q = self.listener.lookupTransform(A, B, now)
            T = np.dot(trans,rot)
            return T
        except:
            rospy.logwarn("[%s]: unable to get_transform from %s to %s!", self.name, A, B)
            return False



if __name__ == '__main__':
    try:
        rospy.init_node('usbl_calibration')
        usbl_auto_calibration = USBLAutoCalibration(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
