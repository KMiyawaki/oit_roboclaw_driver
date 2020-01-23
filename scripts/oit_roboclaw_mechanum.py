#!/usr/bin/env python
# -*- coding: utf_8 -*-

import copy
import math
import numpy as np
import os
from serial import Serial
from threading import Lock
import traceback

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int32MultiArray

from oit.roboclaw.driver import Driver
from oit.roboclaw.constants import Constants
from oit.roboclaw.device_map import DeviceMap
from oit.roboclaw.utils import clamp_abs
from oit.roboclaw.encoder_odom_4_wheels_omni import EncoderOdom4WheelsOmni
from oit.roboclaw.kinematic_parameters import build_kinematic_parameters

# https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf


class OITRoboClawMechanum(object):
    def __init__(self, kinematic_parameters, tread, wheel_base, topic_name_cmd_vel, topic_name_motor_velocities, topic_name_encoder_counts):
        self.__kinematic_parameters = kinematic_parameters
        self.__wheel_size = 4
        self.__tread = tread
        self.__wheel_base = wheel_base
        val = self.__tread / 2.0 + self.__wheel_base / 2.0
        self.__velocity_to_wheel_vels = np.matrix([[+1, -1, -val],
                                                   [+1, +1, +val],
                                                   [+1, +1, -val],
                                                   [+1, -1, +val]])

        self.__twist = Twist()
        self.__encoder_counts = []
        self.__lock = Lock()
        self.__encoder_odom = EncoderOdom4WheelsOmni(
            self.__kinematic_parameters, self.__velocity_to_wheel_vels, '/odom')
        rospy.Subscriber(topic_name_cmd_vel, Twist,
                         self.__cmd_vel_callback, queue_size=1)
        rospy.Subscriber(topic_name_encoder_counts, Int32MultiArray,
                         self.__encoder_callback, queue_size=1)
        self.__pub_velocities = rospy.Publisher(
            topic_name_motor_velocities, Int32MultiArray, queue_size=1)

    def spin(self):
        twist = None
        encs = None
        self.__lock.acquire()
        twist = copy.deepcopy(self.__twist)
        encs = copy.deepcopy(self.__encoder_counts)
        self.__lock.release()

        vel_wheels = self.__twist_to_wheel_velocities(twist)
        rospy.logdebug("Wheel vels:%f, %f, %f, %f",
                       vel_wheels[0], vel_wheels[1], vel_wheels[2], vel_wheels[3])
        vel_ticks = Int32MultiArray()
        vel_ticks.data = [0, 0, 0, 0]

        for i in range(0, self.__wheel_size):
            vel_ticks.data[i] = int(vel_wheels[i] *
                                    self.__kinematic_parameters.ticks_per_meter)
        self.__pub_velocities.publish(vel_ticks)
        rospy.logdebug("Linear vel_x:" + str(twist.linear.x) + ", Linear vel_y:" + str(twist.linear.y) +
                       ", Angular vel:" + str(math.degrees(twist.angular.z)) +
                       ", vel_ticks: " + ','.join(map(str, vel_ticks.data)))
        if len(encs) < self.__wheel_size:
            rospy.logwarn("Encoder count is less than " +
                          str(self.__wheel_size))
        else:
            self.__encoder_odom.update_publish(encs)

    def __twist_to_wheel_velocities(self, twist):
        velocities = np.matrix(
            [twist.linear.x, twist.linear.y, twist.angular.z]).transpose()
        vel_wheels = self.__velocity_to_wheel_vels * velocities
        return [vel_wheels[0, 0], vel_wheels[1, 0], vel_wheels[2, 0], vel_wheels[3, 0]]

    def __cmd_vel_callback(self, twist):
        self.__lock.acquire()
        self.__twist = twist
        rospy.logdebug("Recv:%f, %f, %f", self.__twist.linear.x, self.__twist.linear.y,
                       math.degrees(self.__twist.angular.z))
        self.__clamp_velocities()
        rospy.logdebug("Clamped vels:%f, %f, %f", self.__twist.linear.x, self.__twist.linear.y,
                       math.degrees(self.__twist.angular.z))
        self.__lock.release()

    def __encoder_callback(self, data):
        self.__lock.acquire()
        self.__encoder_counts = data.data
        self.__lock.release()

    def __clamp_velocities(self):
        self.__twist.linear.x = clamp_abs(
            self.__kinematic_parameters.linear_max, self.__twist.linear.x)
        self.__twist.linear.y = clamp_abs(
            self.__kinematic_parameters.linear_max, self.__twist.linear.y)
        self.__twist.angular.z = clamp_abs(
            self.__kinematic_parameters.angular_max, self.__twist.angular.z)


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)

    process_rate = rospy.get_param("~process_rate", 20.0)
    # https://ja.wikipedia.org/wiki/%E3%83%88%E3%83%AC%E3%83%83%E3%83%89
    tread = rospy.get_param("~tread")
    # https://ja.wikipedia.org/wiki/%E3%83%9B%E3%82%A4%E3%83%BC%E3%83%AB%E3%83%99%E3%83%BC%E3%82%B9
    wheel_base = rospy.get_param("~wheel_base")
    ticks_per_meter = rospy.get_param("~ticks_per_meter")
    linear_max = rospy.get_param("~linear_max")
    angular_max = rospy.get_param("~angular_max")
    node = OITRoboClawMechanum(build_kinematic_parameters(
        ticks_per_meter, linear_max, angular_max), tread, wheel_base, "/cmd_vel", "/oit_roboclaw_driver_node/velocities", "/oit_roboclaw_driver_node/encoders")

    rate = rospy.Rate(process_rate)
    rospy.loginfo("Start %s with process rate %f Hz",
                  node_name, process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
    rospy.loginfo("Exiting %s", node_name)


if __name__ == '__main__':
    main()
