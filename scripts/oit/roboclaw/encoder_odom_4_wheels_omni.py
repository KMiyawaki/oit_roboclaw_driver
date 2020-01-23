# -*- coding: utf_8 -*-
from math import degrees, cos, pi, sin
import angles
import copy
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

from oit.roboclaw.constants import Constants
from oit.roboclaw.utils import print_odom
from oit.roboclaw.encoder_odom_base import EncoderOdomBase


class EncoderOdom4WheelsOmni(EncoderOdomBase):
    def __init__(self, kinematic_parameters, velocity_to_wheel_vels_mat, topic_name_odom):
        EncoderOdomBase.__init__(
            self, kinematic_parameters, 4, topic_name_odom)
        self.__velocity_to_wheel_vels_mat = velocity_to_wheel_vels_mat
        self.__wheel_vels_to_velocity = np.linalg.pinv(
            self.__velocity_to_wheel_vels_mat)
        self.__cur_pose_mat = np.matrix([0, 0, 0]).transpose()

    def update(self, encoder_counts):
        thresh_value = 20000
        cur_ticks = np.matrix(encoder_counts).transpose()
        diff_ticks = cur_ticks - np.matrix(self._last_encs).transpose()
        for i in range(4):
            if diff_ticks[i, 0] > thresh_value:
                format_string = "Ignoring encoder[%d] jump: cur %d, last %d"
                message = format_string % (
                    i, cur_ticks[i, 0], self._last_encs[i, 0])
                rospy.logerr(message)
                return None

        self._last_encs = copy.deepcopy(encoder_counts)

        wheels_delta = diff_ticks / self._kinematic_parameters.ticks_per_meter
        local_pose_delta = self.__wheel_vels_to_velocity * wheels_delta
        theta = self.__cur_pose_mat[2, 0]
        rot = np.matrix([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
        self.__cur_pose_mat = self.__cur_pose_mat + rot * local_pose_delta
        self._cur_pose.x = self.__cur_pose_mat[0, 0]
        self._cur_pose.y = self.__cur_pose_mat[1, 0]
        self._cur_pose.theta = self.__cur_pose_mat[2, 0]

        current_time = rospy.Time.now()
        d_time = (current_time - self._last_enc_time).to_sec()
        self._last_enc_time = current_time
        vels = local_pose_delta / d_time
        return Twist(Vector3(vels[0, 0], vels[1, 0], 0), Vector3(0, 0, vels[2, 0]))

    def update_publish(self, encoder_counts):
        twist = self.update(encoder_counts)
        if twist is not None:
            self.publish_odom(twist)
