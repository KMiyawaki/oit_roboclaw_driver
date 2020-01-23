# -*- coding: utf-8 -*-
from math import degrees, cos, pi, sin
import angles
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

from oit.roboclaw.constants import Constants
from oit.roboclaw.utils import print_odom


class EncoderOdomBase(object):
    def __init__(self, kinematic_parameters, wheel_size, topic_name_odom):
        self._kinematic_parameters = kinematic_parameters
        self._odom_pub = rospy.Publisher(
            topic_name_odom, Odometry, queue_size=10)
        self._cur_pose = Pose2D()
        self._last_encs = [0] * wheel_size
        self._last_enc_time = rospy.Time.now()
        self._broad_caster = tf.TransformBroadcaster()

    def update(self, encoder_counts):
        pass

    def update_publish(self, encoder_counts):
        pass

    def publish_odom(self, twist):
        quat = tf.transformations.quaternion_from_euler(
            0, 0, self._cur_pose.theta)
        current_time = rospy.Time.now()

        translation = (self._cur_pose.x, self._cur_pose.y, 0)
        self._broad_caster.sendTransform(
            translation, quat, current_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        # set the position
        odom.pose.pose = Pose(
            Point(self._cur_pose.x, self._cur_pose.y, 0.), Quaternion(*quat))

        # set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist = twist

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance = odom.pose.covariance

        rospy.logdebug(print_odom(odom))
        self._odom_pub.publish(odom)
