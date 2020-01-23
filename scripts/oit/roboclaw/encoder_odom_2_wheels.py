# -*- coding: utf_8 -*-
from math import degrees, cos, pi, sin
import angles
import copy
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

from oit.roboclaw.constants import Constants
from oit.roboclaw.utils import print_odom
from oit.roboclaw.encoder_odom_base import EncoderOdomBase


class EncoderOdom2Wheels(EncoderOdomBase):
    def __init__(self, kinematic_parameters, tread, topic_name_odom):
        EncoderOdomBase.__init__(
            self, kinematic_parameters, 2, topic_name_odom)
        self.__tread = tread

    def update(self, encoder_counts):  # left: 0, right: 1
        ticks = [encoder_counts[Constants().WHEEL_LEFT] - self._last_encs[Constants().WHEEL_LEFT],
                 encoder_counts[Constants().WHEEL_RIGHT] - self._last_encs[Constants().WHEEL_RIGHT]]
        self._last_encs = copy.deepcopy(encoder_counts)

        dist = [0.0, 0.0]
        for i in range(0, 2):
            dist[i] = ticks[i] / self._kinematic_parameters.ticks_per_meter
        total_dist = (dist[Constants().WHEEL_LEFT] +
                      dist[Constants().WHEEL_RIGHT]) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self._last_enc_time).to_sec()
        self._last_enc_time = current_time

        # TODO find better what to determine going straight,
        # this means slight deviation is accounted
        if ticks[Constants().WHEEL_LEFT] == ticks[Constants().WHEEL_RIGHT]:
            d_theta = 0.0
            self._cur_pose.x += total_dist * cos(self._cur_pose.theta)
            self._cur_pose.y += total_dist * sin(self._cur_pose.theta)
        else:
            d_theta = (dist[Constants().WHEEL_RIGHT] -
                       dist[Constants().WHEEL_LEFT]) / self.__tread
            new_theta = self._cur_pose.theta + d_theta
            radius = total_dist / d_theta
            self._cur_pose.x += radius * \
                (sin(new_theta) - sin(self._cur_pose.theta))
            self._cur_pose.y -= radius * \
                (cos(new_theta) - cos(self._cur_pose.theta))
            self._cur_pose.theta = angles.normalize_angle(new_theta)

        twist = Twist()
        if abs(d_time) < 0.000001:
            return twist
        else:
            twist.linear.x = total_dist / d_time
            twist.angular.z = d_theta / d_time
        return twist

    def update_publish(self, encoder_counts):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        thresh_value = 20000
        format_string = "Ignoring encoder[%d] jump: cur %d, last %d"

        for i in range(0, 2):
            diff = encoder_counts[i] - self._last_encs[i]
            if diff > thresh_value:
                message = format_string % (
                    i, encoder_counts[i], self._last_encs[i])
                rospy.logerr(message)
                return
        else:
            twist = self.update(encoder_counts)
            self.publish_odom(twist)
