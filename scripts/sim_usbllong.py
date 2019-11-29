#!/usr/bin/env python

# Basic ROS imports
import roslib
roslib.load_manifest('usbl_position')
import rospy
import numpy as np

# import msgs
from nav_msgs.msg import Odometry
from ned_tools import utils, utils_ros, NED

from evologics_ros_sync.msg import EvologicsUsbllong


# More imports
import tf
import math

INIT_POSITION = [0.0, 0.0, 0.0]
INIT_ORIENTATION = [0.0, 0.0, 0.0, 1.0]


class SimNavSensors:
    """ This class is able to simulate the navigation sensors of s2 AUV """
    def __init__(self, name):

        """ Constructor """
        self.name = name
        self.ns = rospy.get_namespace()

        # Load dynamic parameters
        self.get_config()

        # Initial vehicle pose
        vehicle_pose_t = tf.transformations.translation_matrix(np.array(INIT_POSITION))
        vehicle_pose_q = tf.transformations.quaternion_matrix(np.array(INIT_ORIENTATION))
        self.vehicle_pose = tf.transformations.concatenate_matrices(vehicle_pose_t, vehicle_pose_q)

        # Tfs
        self.listener = tf.TransformListener()
        self.usbl_tf_init = False

        # usbl positons
        self.usbl_positions = []

        # Create publishers
        self.pub_usbl = rospy.Publisher('usbllong',
                             EvologicsUsbllong,
                             queue_size = 2)

        # Create subscribers to odometry
        rospy.Subscriber(self.odom_topic_name, Odometry, self.update_odometry, queue_size = 1)

        # Get Static TFs
        self.usbl2world = self.get_static_transform(self.usbl_frame_id, self.world_frame_id)
        self.baselink2modem = self.get_static_transform(self.robot_frame_id, self.modem_frame_id)

        # Init simulated sensor timer
        rospy.Timer(rospy.Duration(self.usbl_period), self.pub_usbl_callback)

        # Show message
        rospy.loginfo("[%s]: initialized", self.name)



    def update_odometry(self, odom):
        """ This method is a callback of the odometry message that comes
            from dynamics node """
        self.odom = odom

        vehicle_pose_t = tf.transformations.translation_matrix(np.array([self.odom.pose.pose.position.x,
                                                                           self.odom.pose.pose.position.y,
                                                                           self.odom.pose.pose.position.z]))
        vehicle_pose_q = tf.transformations.quaternion_matrix(np.array([self.odom.pose.pose.orientation.x,
                                                                          self.odom.pose.pose.orientation.y,
                                                                          self.odom.pose.pose.orientation.z,
                                                                          self.odom.pose.pose.orientation.w]))
        self.vehicle_pose = tf.transformations.concatenate_matrices(vehicle_pose_t, vehicle_pose_q)

        # Quaternion to Euler
        self.orientation = tf.transformations.euler_from_quaternion(
                                    [self.odom.pose.pose.orientation.x,
                                     self.odom.pose.pose.orientation.y,
                                     self.odom.pose.pose.orientation.z,
                                     self.odom.pose.pose.orientation.w])



    def pub_usbl_callback(self, event):
        """ This method is a callback of a timer. This publishes usbl position data """

        # Publish usbl position data only far of the surface
        if self.odom.pose.pose.position.z > self.gps_min_depth:   

            world2modem = tf.transformations.concatenate_matrices(self.vehicle_pose, self.baselink2modem)
            usbl2modem = tf.transformations.concatenate_matrices(self.usbl2world, world2modem)
            usbllong_tf = tf.transformations.translation_from_matrix(usbl2modem)

            #Define message
            usbllong = EvologicsUsbllong()
            usbllong.header.stamp = rospy.Time.now()
            usbllong.header.frame_id = 'usbl'
            usbllong.N = (usbllong_tf[0] + np.random.normal(self.usbl_drift[0], self.usbl_position_covariance_gen[0]))
            usbllong.E = (usbllong_tf[1] + np.random.normal(self.usbl_drift[1], self.usbl_position_covariance_gen[1]))
            usbllong.D = (usbllong_tf[2] + np.random.normal(self.usbl_drift[2], self.usbl_position_covariance_gen[2])) 

            usbllong.accuracy = self.usbl_position_covariance[0]
            usbllong.rssi = -40
            usbllong.integrity = 120


            self.usbl_positions.append(usbllong)
            rospy.sleep(self.usbl_delay)
            self.pub_usbl.publish(self.usbl_positions.pop(0))

    def get_static_transform(self, parent, child):
        try:
            rospy.logwarn("[%s]: waiting transform from %s to %s", self.name, parent, child)
            self.listener.waitForTransform(parent,
                                            child,
                                            rospy.Time(),
                                            rospy.Duration(1.0))

            (trans, rot) = self.listener.lookupTransform(
                                parent, child, rospy.Time())
            rospy.loginfo("[%s]: transform for %s found", self.name, child)
            transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), 
                                                                        tf.transformations.quaternion_matrix(rot))

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
                tf.Exception):
            rospy.logerr('[%s]: define %s transform!',
                            self.name,
                            child)
        return transform

    def get_config(self):
        """ Get config from param server """

        param_dict = {'odom_topic_name': self.ns + "dynamics/topic_name",
                      'gps_min_depth': self.ns + "sim_nav_sensors/gps_min_depth",
                      'world_frame_id': self.ns + "frames/map",
                      'robot_frame_id': self.ns + "frames/base_link",
                      'usbl_frame_id': self.ns + "frames/sensors/usbl",
                      'modem_frame_id': self.ns + "frames/sensors/modem",
                      'usbl_period': self.ns + "sim_nav_sensors/usbl/period",
                      'usbl_drift': self.ns + "sim_nav_sensors/usbl/drift",
                      'usbl_delay': self.ns + "sim_nav_sensors/usbl/delay",
                      'usbl_position_covariance': self.ns + "sim_nav_sensors/usbl/position_covariance",
                      'usbl_position_covariance_gen': self.ns + "sim_nav_sensors/usbl/position_covariance_gen"}

        if not utils_ros.getRosParams(self, param_dict, self.name):
            rospy.logfatal("[%s]: shutdown due to invalid config parameters!", self.name)
            exit(0)  # TODO: find a better way


def __compute_tf__(transform):
    r = PyKDL.Rotation.RPY(math.radians(transform[3]),
                           math.radians(transform[4]),
                           math.radians(transform[5]))
    v = PyKDL.Vector(transform[0], transform[1], transform[2])
    frame = PyKDL.Frame(r, v)
    return frame


if __name__ == '__main__':
    try:
        rospy.init_node('sim_nav_sensors')
        sim_nav_sensors = SimNavSensors(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
