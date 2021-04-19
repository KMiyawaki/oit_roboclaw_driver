# -*- coding: utf_8 -*-

from collections import namedtuple
import rospy
from oit.roboclaw.constants import Constants


def build_serial_setting(port, baud_rate, timeout, write_timeout, inter_byte_timeout):
    factory = namedtuple("SerialSetting",
                         ['port', 'baud_rate', 'timeout', 'write_timeout', 'inter_byte_timeout', 'roboclaw_settings'])
    return factory(port, baud_rate, timeout, write_timeout, inter_byte_timeout, [])


def build_roboclaw_setting(address, serial_interval_after_write, serial_interval_after_read, motor_index_of_connector_M1, motor_index_of_connector_M2):
    factory = namedtuple("RoboClawSetting",
                         ['address', 'serial_interval_after_write', 'serial_interval_after_read', 'motor_index_of_connector_M1', 'motor_index_of_connector_M2'])
    return factory(address, serial_interval_after_write, serial_interval_after_read, motor_index_of_connector_M1, motor_index_of_connector_M2)


class DeviceConf(object):
    def __init__(self, max_serial_port_size=Constants.MAX_SERIAL_PORT_SIZE, max_roboclaw_size=Constants.MAX_ROBOCLAW_SIZE):
        self.__max_serial_port_size = max_serial_port_size
        self.__max_roboclaw_size = max_roboclaw_size
        self.build()

    def __str__(self):
        text = ""
        for s in self.serial_settings:
            text += str(s) + "\n"
        return text

    def clear(self):
        self.motor_index = []
        self.serial_settings = []

    def build(self):
        self.clear()
        for i in range(0, self.__max_serial_port_size):
            serial_param_name = "~serial_" + str(i)
            if rospy.has_param(serial_param_name + "/baud_rate") == False:
                break
            self.serial_settings.append(
                self.__build_serial_setting(serial_param_name))
        if len(self.serial_settings) == 0:
            rospy.logerr("Can't find any serial settings in roboclaw_driver.")
        self.__check_motor_index()

    def __build_serial_setting(self, serial_param_name):
        port = rospy.get_param(serial_param_name + "/port")
        baud_rate = rospy.get_param(serial_param_name + "/baud_rate")
        timeout = rospy.get_param(serial_param_name + "/timeout")
        write_timeout = rospy.get_param(serial_param_name + "/write_timeout")
        inter_byte_timeout = rospy.get_param(
            serial_param_name + "/inter_byte_timeout")
        serial_setting = build_serial_setting(
            port, baud_rate, timeout, write_timeout, inter_byte_timeout)

        for i in range(0, self.__max_roboclaw_size):
            roboclaw_param_name = serial_param_name + "/roboclaw_" + str(i)
            if rospy.has_param(roboclaw_param_name + "/address") == False:
                break
            serial_setting.roboclaw_settings.append(
                self.__build_roboclaw_setting(roboclaw_param_name))
            if len(serial_setting.roboclaw_settings) == 0:
                rospy.logerr(
                    "Can't find any roboclaw settings under '" + serial_param_name + "'.")
        return serial_setting

    def __build_roboclaw_setting(self, roboclaw_param_name):
        address = rospy.get_param(roboclaw_param_name + "/address")
        serial_interval_after_write = rospy.get_param(
            roboclaw_param_name + "/serial_interval_after_write")
        serial_interval_after_read = rospy.get_param(
            roboclaw_param_name + "/serial_interval_after_read")
        motor_index_of_connectors = []
        for i in range(0, 2):
            motor_index_of_connectors.append(rospy.get_param(
                roboclaw_param_name + "/motor_index_of_connector_M" + str(i + 1), None))
            if motor_index_of_connectors[i] is not None:
                self.motor_index.append(motor_index_of_connectors[i])
        roboclaw_setting = build_roboclaw_setting(
            address, serial_interval_after_write, serial_interval_after_read, motor_index_of_connectors[Constants.CONNECTOR_M1], motor_index_of_connectors[Constants.CONNECTOR_M2])
        return roboclaw_setting

    def __check_motor_index(self):
        self.motor_index.sort()
        for i in range(0, len(self.motor_index)):
            if i != self.motor_index[i]:
                rospy.logerr("RoboClaw motor index is invailed. ")
                rospy.logerr("Your motor index:" +  ','.join(map(str, self.motor_index)))
