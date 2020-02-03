#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import os
from serial import Serial
from threading import Lock
import traceback

import rospy
from std_msgs.msg import Int32MultiArray, Int32

from oit.roboclaw.driver import Driver
from oit.roboclaw.constants import Constants
from oit.roboclaw.device_map import DeviceMap
from oit.roboclaw.device_conf import DeviceConf


class OITRoboClawDriverNode(object):
    def __init__(self, device_map):
        self.__device_map = device_map
        for rc in self.__device_map.get_roboclaws():
            for _ in range(0, 5):
                try:
                    rc.reset_quadrature_encoder_counters()
                    rospy.logwarn("Reset encoder_counters. ")
                    break
                except:
                    rospy.logwarn("Failed to reset encoder_counters(). Retry.")
        self.__motor_velocities = []
        self.__lock = Lock()

        self.__sub_motor_vels = rospy.Subscriber(
            "~velocities", Int32MultiArray, self.__cb_motor_velocities, queue_size=5)
        self.__pub_encoder_ticks = rospy.Publisher(
            "~encoders", Int32MultiArray, queue_size=5)
        self.__pub_emergency = rospy.Publisher(
            "~emergency", Int32, queue_size=5)

    def spin(self):
        node_name = rospy.get_name()
        emergency = 0
        try:
            data = Int32MultiArray()
            data.data = self.__device_map.get_encoder_ticks()
            self.__pub_encoder_ticks.publish(data)
            if self.__device_map.is_emergency_pushed() == True:
                emergency = 1
        except:
            rospy.logerr(node_name + ": exception while reading encoder counts.")
            # rospy.logerr(traceback.format_exc())

        self.__pub_emergency.publish(emergency)

        self.__lock.acquire()
        motor_vels = copy.deepcopy(self.__motor_velocities)
        self.__lock.release()

        try:
            self.__device_map.set_motor_vels(motor_vels)
        except:
            rospy.logerr(node_name + ": exception while setting motor velocities.")
            # rospy.logerr(traceback.format_exc())

    def stop(self):
        rospy.loginfo("Stopping RoboClaw motors.")
        for rc in self.__device_map.get_roboclaws():
            rc.stop_all_motors()

    def __cb_motor_velocities(self, data):
        rospy.logdebug("Recv velocity:" + str(data.data))
        self.__lock.acquire()
        self.__motor_velocities = copy.deepcopy(data.data)
        self.__lock.release()


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    device_map = None
    process_rate = rospy.get_param("~process_rate", 20.0)
    motor_accel = rospy.get_param("~mtor_acceleration", 2000)
    try:
        device_map = DeviceMap(DeviceConf(), motor_accel)
    except:
        rospy.logerr(node_name + ":Failed to build device map.")
        rospy.logerr(traceback.format_exc())
        exit(1)
    node = OITRoboClawDriverNode(device_map)
    rate = rospy.Rate(process_rate)
    rospy.loginfo("Start %s with process rate %f Hz",
                  node_name, process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
    node.stop()

    for serial in device_map.get_serials():
        serial.close()

    rospy.loginfo("Exiting %s", node_name)


if __name__ == '__main__':
    main()
