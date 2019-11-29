#!/usr/bin/env python

import roslib
import rospy
from sensors.msg import AdisImu
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse
import tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import datetime
from numpy import *
import PyKDL
import sys
from std_srvs.srv import Empty
from auv_msgs.msg import GoalDescriptor
from auv_msgs.msg import NavSts
from control.srv import Goto, GotoRequest

from utils import utils, NED

from std_srvs.srv import Empty, EmptyRequest


class MagnetometerAutoCalibration:
    """ Auto Calibration node for the IMU """

    def __init__(self, name):
        """ Init the class """
        self.name = name
        self.robot_frame_id = self.get_param('frames/base_link','sparus2')
        self.ned_origin_lat = self.get_param('navigator/ned_origin_lat')
        self.ned_origin_lon = self.get_param('navigator/ned_origin_lon')
        self.adis_topic_name = self.get_param('magnetometer_star_calibration/adis_topic_name')
        self.frame_id = self.get_param('magnetometer_star_calibration/frame_id')
        self.calibration_file_path  = self.get_param('magnetometer_star_calibration/calibration_file_path')
        self.depth = self.get_param('magnetometer_star_calibration/depth', 0.0)
        self.radius = self.get_param('magnetometer_star_calibration/radius', 12.0)
        self.sides = self.get_param('magnetometer_star_calibration/sides', 5)
        self.tolerance = self.get_param('magnetometer_star_calibration/tolerance', 2.0)
        self.acceleration_limit = self.get_param('magnetometer_star_calibration/acceleration_limit', 0.2)
        self.hard_iron_param_name = self.get_param('magnetometer_star_calibration/hard_iron_param_name', 2.0)
        self.soft_iron_param_name = self.get_param('magnetometer_star_calibration/soft_iron_param_name', 2.0)

        self.imu_received = False
        self.record_imu = False

        # tf listener
        self.listener = tf.TransformListener()
        try:
            self.listener.waitForTransform(self.robot_frame_id,
                                           self.frame_id,
                                           rospy.Time(),
                                           rospy.Duration(5.0))
            (trans, rot) = self.listener.lookupTransform(
                              self.robot_frame_id, self.frame_id, rospy.Time())
            euler = tf.transformations.euler_from_quaternion(rot)
            imu_rot = PyKDL.Rotation.RPY(euler[0], euler[1], euler[2])
            self.imu_rotation = matrix([[imu_rot[0, 0],imu_rot[0, 1],imu_rot[0, 2]],
                                        [imu_rot[1, 0],imu_rot[1, 1],imu_rot[1, 2]],
                                        [imu_rot[2, 0],imu_rot[2, 1],imu_rot[2, 2]]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal('[%s]: Impossible to transform from %s to %s', self.name, self.robot_frame_id, self.frame_id)
            exit(0)

        self.ned = NED.NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)
        # Services clients
        try:
            rospy.wait_for_service('/control/goto_local_block', 20)
            self.goto_srv = rospy.ServiceProxy(
                        '/control/goto_local_block', Goto)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s: error creating client to goto service',
                         self.name)
            rospy.signal_shutdown('Error creating client to goto service')

        # Sevice server
        self.start_srv = rospy.Service('/control/start_magnetometer_calibration_star',
                                       Goto,
                                       self.start_magnetometer_calibration_star)
        # Show message
        rospy.loginfo('[%s]: initialized', self.name)

    def start_magnetometer_calibration_star(self, req):
        """ Main method """
        rospy.loginfo('[%s]: calling start magnetometer calibration service', self.name)

        # Clear calib vectors
        self.ax_calib_vector = []
        self.ay_calib_vector = []
        self.az_calib_vector = []
        self.mx_calib_vector = []
        self.my_calib_vector = []
        self.mz_calib_vector = []
        self.filtered_tilt_corrected_mx_calib_vector = []
        self.filtered_tilt_corrected_my_calib_vector = []
        self.filtered_tilt_corrected_mz_calib_vector = []

        # Subscribe to IMU and nav_sts
        imu_sub = rospy.Subscriber(self.adis_topic_name, AdisImu, self.imu_callback, queue_size=1)
        nav_sub = rospy.Subscriber('/navigation/nav_sts', NavSts, self.nav_callback, queue_size=1)

        # Wait for IMU to be available
        r = rospy.Rate(1) # 1 Hz
        while not rospy.is_shutdown():
            if (self.imu_received):
                break
            else:
                rospy.loginfo('Waiting for IMU topic to be available')
            r.sleep()

        # remove any calibration
        rospy.set_param(self.soft_iron_param_name, '[1, 0, 0, 0, 1, 0, 0, 0, 1]')
        rospy.set_param(self.hard_iron_param_name, '[0.0, 0.0, 0.0]')

        """Enable start calibration maneuver."""
        self.call_empty_srv('/control/enable_thrusters')
        pose = self.ned.geodetic2ned([req.north_lat,
                                      req.east_lon,
                                      self.depth])
        # Perform star maneuver
        text = '{0}: perform start maneuver at ({1}, {2}), depth {3} in a {4}m radius start of {5} sides.'.format(self.name, pose[0], pose[1], pose[2], self.radius, self.sides)
        rospy.loginfo(text)
        self.perform_star_maneuver(pose, self.radius, self.sides)

        rospy.loginfo('[%s]: calling compute magnetometer calibration service', self.name)
        imu_sub.unregister()
        nav_sub.unregister()
        self.compute_magnetometer_calibration()

        # Call service to compute calibration
        rospy.loginfo('[%s]: finished', self.name)
        return True

    def nav_callback(self, nav):
        self.last_nav = nav

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

    def perform_star_maneuver(self, pose, radius, sides):
        """Perform star maneuver."""
        self.transit_to(pose)
        x, y, z = pose
        angle_error_sum = 0.0
        angle_delta = 2.0 * math.pi * math.floor(sides / 2) / sides
        angle = 0.0
        x1 = x + radius
        y1 = y
        self.record_imu = True
        for i in range(sides):
            angle_new = angle + angle_delta
            x2 = x + math.cos(angle_new) * radius
            y2 = y + math.sin(angle_new) * radius
            self.star_segment(x1, y1, x2, y2, z)
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
        self.record_imu = False
        rospy.loginfo('%s: done', self.name)

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
        # Compute calibration
        self.magnetometer_calibration_2d()
        # Erase magnetometer calibration data
        self.ax_calib_vector = []
        self.ay_calib_vector = []
        self.az_calib_vector = []
        self.mx_calib_vector = []
        self.my_calib_vector = []
        self.mz_calib_vector = []
        self.filtered_tilt_corrected_mx_calib_vector = []
        self.filtered_tilt_corrected_my_calib_vector = []
        self.filtered_tilt_corrected_mz_calib_vector = []

    def imu_callback(self, imu_message):
        if self.record_imu:
            acc_vector = matrix([[imu_message.AX],
                                 [imu_message.AY],
                                 [imu_message.AZ]])
            acc_vector = self.imu_rotation * acc_vector
            mag_input = matrix([[imu_message.MX],
                                [imu_message.MY],
                                [imu_message.MZ]])
            mag_vector = self.imu_rotation * mag_input

            self.ax_calib_vector.append(acc_vector[0, 0])
            self.ay_calib_vector.append(acc_vector[1, 0])
            self.az_calib_vector.append(acc_vector[2, 0])
            self.mx_calib_vector.append(mag_vector[0, 0])
            self.my_calib_vector.append(mag_vector[1, 0])
            self.mz_calib_vector.append(mag_vector[2, 0])

        if not self.imu_received:
            self.imu_received = True

    def magnetometer_calibration_2d(self):
        """ Compute magnetometer calibration in 2d """
        # TODO: optimize this method and improve variable names
        try:
            # Leveling and tilt correction
            for it in range(len(self.ax_calib_vector)):
                if abs((self.ax_calib_vector[it] ** 2.0 + self.ay_calib_vector[it] ** 2.0 + self.az_calib_vector[it] ** 2.0) ** 0.5 - 9.80665) < self.acceleration_limit:  # Reject data with high acceleration
                    tilt_roll = math.atan2(-self.ay_calib_vector[it], -self.az_calib_vector[it])
                    #tilt_pitch = math.atan2(-self.ax_calib_vector[it], self.ay_calib_vector[it]*math.sin(tilt_roll) + self.az_calib_vector[it]*math.cos(tilt_roll))
                    den = -self.ay_calib_vector[it] * math.sin(tilt_roll) - self.az_calib_vector[it] * math.cos(tilt_roll)  # This is to have the pitch from pi/2 to -pi/2 (to force unique output)
                    if den != 0.0:
                        tilt_pitch = math.atan(self.ax_calib_vector[it] / den)
                    else:
                        if self.ax_calib_vector[it] == 0.0:
                            tilt_pitch = 0.0  # Not enough information
                        elif self.ax_calib_vector[it] < 0.0:
                            tilt_pitch = -math.pi * 0.5  # -pi/2
                        else:
                            tilt_pitch = math.pi * 0.5  # pi/2
                    rot1 = matrix([[math.cos(tilt_pitch), 0.0, math.sin(tilt_pitch)], [0.0, 1.0, 0.0], [-math.sin(tilt_pitch), 0.0, math.cos(tilt_pitch)]])
                    rot2 = matrix([[1.0, 0.0, 0.0], [0.0, math.cos(tilt_roll), -math.sin(tilt_roll)], [0.0, math.sin(tilt_roll), math.cos(tilt_roll)]])
                    rot = rot1 * rot2
                    corrected_m = rot * matrix([[self.mx_calib_vector[it]], [self.my_calib_vector[it]], [self.mz_calib_vector[it]]])
                    self.filtered_tilt_corrected_mx_calib_vector.append(corrected_m[0, 0])
                    self.filtered_tilt_corrected_my_calib_vector.append(corrected_m[1, 0])
                    self.filtered_tilt_corrected_mz_calib_vector.append(corrected_m[2, 0])

            # Check size of filtered data for calibration
            if len(self.filtered_tilt_corrected_mx_calib_vector) < 300:
                rospy.logwarn('[%s]: few data after filtering for calibration', self.name)

            # Fit initial ellipse
            try:
                [a, b, c, d, e, f] = self.fit_ellipse(self.filtered_tilt_corrected_mx_calib_vector, self.filtered_tilt_corrected_my_calib_vector)
            except:
                rospy.logerr('[%s]: error fitting initial ellipse', self.name)
                self.print_error_to_file('Error fitting initial ellipse')
                return

            # Center of the ellipse
            x_cent = (2.0 * c * d / b - e) / (b - 4.0 * a * c / b)
            y_cent = (d - 2.0 * a * e / b) / (4.0 * a * c / b - b)

            # Center ellipse
            x_calib_data = [x - x_cent for x in self.filtered_tilt_corrected_mx_calib_vector]
            y_calib_data = [y - y_cent for y in self.filtered_tilt_corrected_my_calib_vector]

            # Fit centered ellipse
            try:
                [a, b, c, d, e, f] = self.fit_ellipse(x_calib_data, y_calib_data)
            except:
                rospy.logerr('[%s]: error fitting centered ellipse', self.name)
                self.print_error_to_file('Error fitting centered ellipse')
                return

            # Angle of the major axis Q
            q = math.pow((math.pow(((c - a) / b), 2.0) + 1.0), 0.5) + (c - a) / b
            angle_1 = math.atan(q)
            angle_2 = math.atan(-1.0 / q)

            distance1 = math.pow((abs(-f / (a + b * math.tan(angle_1) + c * math.pow(math.tan(angle_1), 2.0))) + abs(-f / (a / math.pow(math.tan(angle_1), 2.0) + b / math.tan(angle_1) + c))), 0.5)
            distance2 = math.pow((abs(-f / (a + b * math.tan(angle_2) + c * math.pow(math.tan(angle_2), 2.0))) + abs(-f / (a / math.pow(math.tan(angle_2), 2.0) + b / math.tan(angle_2) + c))), 0.5)

            if distance1 > distance2:
                Q = angle_1
                scale_factor = distance1 / distance2
            else:
                Q = angle_2
                scale_factor = distance2 / distance1

            # Compose matrices
            ROT = matrix(eye(2))
            ROT[0, 0] = math.cos(Q)
            ROT[0, 1] = -math.sin(Q)
            ROT[1, 0] = math.sin(Q)
            ROT[1, 1] = math.cos(Q)

            preS = ROT * matrix([[1, 0], [0, scale_factor]]) * ROT.T
            S = matrix(eye(3))
            S[0, 0] = preS[0, 0]
            S[0, 1] = preS[0, 1]
            S[1, 0] = preS[1, 0]
            S[1, 1] = preS[1, 1]

            preH = matrix(zeros((3, 1)))
            preH[0,0] = -x_cent
            preH[1,0] = -y_cent

            self.ned_frame_S = S
            self.ned_frame_H = S * preH

            self.imu_frame_S =  self.ned_frame_S
            self.imu_frame_H =  self.ned_frame_H

            self.print_magnetometer_results_to_file()
            self.print_magnetometer_results_to_image()

            rospy.set_param(self.soft_iron_param_name, self.imu_frame_S.A1.tolist())
            rospy.set_param(self.hard_iron_param_name, self.imu_frame_H.A1.tolist())
        except:
            rospy.logerr('[%s]: error in self.magnetometer_calibration_2d', self.name)
            self.print_error_to_file('Unexpected error in calibration')

    def verify_magnetometer_calibration_data(self):
        """ Verify data """
        try:
            # Verify length
            if len(self.mx_calib_vector) < 600:
                rospy.logwarn('[%s]: not enough data to use in calibration', self.name)

            # Data from all quads are needed
            quad = [False] * 4
            it = 0
            while not all(quad) and it < len(self.mx_calib_vector):
                if self.mx_calib_vector[it] > 0.0:
                    if self.my_calib_vector[it] > 0.0:
                        quad[0] = True
                    else:
                        quad[3] = True
                else:
                    if self.my_calib_vector[it] > 0.0:
                        quad[1] = True
                    else:
                        quad[2] = True
                it += 1
            if not all(quad):
                rospy.logwarn('[%s]: data not from all quad. in calibration', self.name)
        except:
            rospy.logerr('[%s]: unexpected error in data verification', self.name)


    def fit_ellipse(self, x_data, y_data):
        """ Fits an ellipse to x and y data """
        # TODO: optimize this method and improve variable names
        # Ellipse fit to ax^2+bxy+cy^2+dx+ey+f=0
        # Initialize some vars

        D1mat = matrix(zeros((len(x_data), 3)))
        D2mat = matrix(zeros((len(x_data), 3)))

        # Compute needed matrices from data buffers
        for it in range(len(x_data)):
            D1mat[it, 0] = x_data[it] * x_data[it]
            D1mat[it, 1] = x_data[it] * y_data[it]
            D1mat[it, 2] = y_data[it] * y_data[it]

            D2mat[it, 0] = x_data[it]
            D2mat[it, 1] = y_data[it]
            D2mat[it, 2] = 1.0

        S1mat = matrix(D1mat.T * D1mat)
        S2mat = matrix(D1mat.T * D2mat)
        S3mat = matrix(D2mat.T * D2mat)

        Tmat = matrix(-S3mat.I * S2mat.T)

        Mmat = matrix(S1mat + S2mat * Tmat)

        newMmat = matrix(zeros((3, 3)))
        newMmat[0, :] = Mmat[2, :] * 0.5
        newMmat[1, :] = -Mmat[1, :]
        newMmat[2, :] = Mmat[0, :] * 0.5

        evalues, evectors = linalg.eig(newMmat)

        cond0 = 4.0 * evectors[0, 0] * evectors[2, 0] - evectors[1, 0] * evectors[1, 0]
        if cond0 > 0:
            a1 = matrix(evectors[:, 0])
        cond1 = 4.0 * evectors[0, 1] * evectors[2, 1] - evectors[1, 1] * evectors[1, 1]
        if cond1 > 0:
            a1 = matrix(evectors[:,1])
        cond2 = 4.0 * evectors[0, 2] * evectors[2, 2] - evectors[1, 2] * evectors[1, 2]
        if cond2 > 0:
            a1 = matrix(evectors[:, 2])

        a2 = Tmat * a1

        return [a1[0, 0], a1[1, 0], a1[2, 0], a2[0, 0], a2[1, 0], a2[2, 0]]


    def print_magnetometer_results_to_file(self):
        try:
            """ Print results to file """
            _file = open(self.calibration_file_path + '/imu_calibration.txt', 'w')
            _now = datetime.datetime.now()
            _header = '\n------------------------------------------------------------------------------------------\nMAGNETOMETER CALIBRATION, ' + str(_now) + '\n\n'
            _mat_results = 'Soft iron:\n' + str(self.imu_frame_S) + '\n\nHard iron:\n' + str(self.imu_frame_H) + '\n\n'
            _num_results = 'navigator/soft_iron: [' + str(self.imu_frame_S[0, 0]) + ', ' + str(self.imu_frame_S[0, 1]) + ', ' + str(self.imu_frame_S[0, 2]) + ', ' + str(self.imu_frame_S[1, 0]) + ', ' + str(self.imu_frame_S[1, 1]) + ', ' + str(self.imu_frame_S[1, 2]) + ', ' + str(
                self.imu_frame_S[2, 0]) + ', ' + str(self.imu_frame_S[2, 1]) + ', ' + str(self.imu_frame_S[2, 2]) + ']\nnavigator/hard_iron: [' + str(self.imu_frame_H[0, 0]) + ', ' + str(self.imu_frame_H[1, 0]) + ', ' + str(self.imu_frame_H[2, 0]) + ']\n\n\n'
            _data = 'Data:\nMX:\n' + str(self.mx_calib_vector) + '\n\nMY:\n' + str(self.my_calib_vector) + '\n\nMZ:\n' + str(self.mz_calib_vector) + '\n\nAX:\n' + str(self.ax_calib_vector) + '\n\nAY:\n' + str(self.ay_calib_vector) + '\n\nAZ:\n' + str(self.az_calib_vector) + '\n\nFILTERED_MX:\n' + str(self.filtered_tilt_corrected_mx_calib_vector) + '\n\nFILTERED_MY:\n' + str(self.filtered_tilt_corrected_my_calib_vector) + '\n\nFILTERED_MZ:\n' + str(self.filtered_tilt_corrected_mz_calib_vector) + '\n\n'
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


    def print_error_to_file(self, _error):
        rospy.loginfo('Printing error to /tmp/imu_calibration_error.txt')
        try:
            """ Print error to file """
            _file = open('/tmp/imu_calibration_error.txt', 'w')
            _now = datetime.datetime.now()
            _header = "\n------------------------------------------------------------------------------------------\nMAGNETOMETER CALIBRATION, " + str(_now) + "\n\n"
            _data = "Data:\nMX:\n" + str(self.mx_calib_vector) + "\n\nMY:\n" + str(self.my_calib_vector) + "\n\nMZ:\n" + str(self.mz_calib_vector) + "\n\nAX:\n" + str(self.ax_calib_vector) + "\n\nAY:\n" + str(self.ay_calib_vector) + "\n\nAZ:\n" + str(self.az_calib_vector) + "\n\nFILTERED_MX:\n" + str(self.filtered_tilt_corrected_mx_calib_vector) + "\n\nFILTERED_MY:\n" + str(self.filtered_tilt_corrected_my_calib_vector) + "\n\nFILTERED_MZ:\n" + str(self.filtered_tilt_corrected_mz_calib_vector) + "\n\n"
            _string = _header + "Error:\n" + _error + "\n\n" + _data
            _file.write(_string)
            _file.close()
        except:
            rospy.logerr('[%s]: error printing error to file', self.name)


    def print_magnetometer_results_to_image(self):
        try:
            """ Print results to image (plot) """
            # TODO: optimize this method and improve variable names
            _now = datetime.datetime.now()
            fig = plt.figure()
            fig.canvas.manager.set_window_title('IMU calibration: ' + str(_now))
            sub_plot = fig.add_subplot(111, aspect='equal')

            # Without correction
            aux_x = list(self.mx_calib_vector)
            aux_y = list(self.my_calib_vector)
            _pointsA = []
            while len(aux_x) > 0:
                _pointsA.append([aux_x.pop(0), aux_y.pop(0)])
            sub_plot.plot(*zip(*_pointsA), marker='o', color='r', ls='')  # Red -> raw data

            # With tilt correction
            aux_x = list(self.filtered_tilt_corrected_mx_calib_vector)
            aux_y = list(self.filtered_tilt_corrected_my_calib_vector)
            _pointsB = []
            while len(aux_x) > 0:
                _pointsB.append([aux_x.pop(0), aux_y.pop(0)])
            sub_plot.plot(*zip(*_pointsB), marker='o', color='g', ls='')  # Green -> tilt correction

            # With whole correction
            aux_x = list(self.mx_calib_vector)
            aux_y = list(self.my_calib_vector)
            _pointsC = []
            while len(aux_x) > 0:
                mx = aux_x.pop(0)
                my = aux_y.pop(0)
                _pointsC.append([self.ned_frame_S[0, 0] * mx + self.ned_frame_S[0, 1] * my + self.ned_frame_H[0, 0], self.ned_frame_S[1, 0] * mx + self.ned_frame_S[1, 1] * my + self.ned_frame_H[1, 0]])
            sub_plot.plot(*zip(*_pointsC), marker='x', color='b', ls='')  # Blue -> whole correction
            sub_plot.set_title('Magnetometer calibration\n(Red -> Raw, Green -> Filtered and tilt corrected, Blue -> Whole correction)')
            sub_plot.set_xlabel('X axis [gauss]')
            sub_plot.set_ylabel('Y axis [gauss]')
            plt.savefig(self.calibration_file_path + '/imu_calibration_plot.png')
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

if __name__ == '__main__':
    try:
        rospy.init_node('magnetometer_star_calibration')
        imu_auto_calibration = MagnetometerAutoCalibration(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
