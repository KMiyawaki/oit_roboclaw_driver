# -*- coding: utf_8 -*-
from collections import namedtuple


class Constants(object):
    CONNECTOR_M1 = 0
    CONNECTOR_M2 = 1
    FORWARD = 0
    BACKWARDS = 1
    WHEEL_LEFT = 0
    WHEEL_RIGHT = 1
    MAX_SERIAL_PORT_SIZE = 4
    MAX_ROBOCLAW_SIZE = 8

    # Status bit masks. pp.73
    MASK_NORMAL = 0x0000
    M1_OVERCURRENT_WARNING = 0X0001
    M2_OVERCURRENT_WARNING = 0X0002
    E_STOP = 0X0004
    TEMPERATURE_ERROR = 0X0008
    TEMPERATURE2_ERROR = 0X0010
    MAIN_BATTERY_HIGH_ERROR = 0X0020
    LOGIC_BATTERY_HIGH_ERROR = 0X0040
    LOGIC_BATTERY_LOW_ERROR = 0X0080
    M1_DRIVER_FAULT = 0X0100
    M2_DRIVER_FAULT = 0X0200
    MAIN_BATTERY_HIGH_WARNING = 0X0400
    MAIN_BATTERY_LOW_WARNING = 0X0800
    TERMPERATURE_WARNING = 0X1000
    TEMPERATURE2_WARNING = 0X2000
    M1_HOME = 0X4000
    M2_HOME = 0X8000


def build_pid_qpps_settings_data():
    return namedtuple('PID', ('P', 'I', 'D', 'QPPS'))


def motor_connector_str(motor_connector):
    if motor_connector == Constants.CONNECTOR_M1:
        return "M1"
    elif motor_connector == Constants.CONNECTOR_M2:
        return "M2"
    return "UnKnown"


def status_str(status):
    err_string = ""
    if status == Constants.MASK_NORMAL:
        err_string = 'Normal'
    else:
        if status & Constants.M1_OVERCURRENT_WARNING > 0:
            err_string += 'Warning: High Current - Motor 1\n'
        if status & Constants.M2_OVERCURRENT_WARNING > 0:
            err_string += 'Warning: High Current - Motor 2\n'
        if status & Constants.E_STOP > 0:
            err_string += 'Emergency Stop Triggered\n'
        if status & Constants.TEMPERATURE_ERROR > 0:
            err_string += 'Error: High Temperature - Sensor 1\n'
        if status & Constants.TEMPERATURE2_ERROR > 0:
            err_string += 'Error: High Temperature - Sensor 2\n'
        if status & Constants.MAIN_BATTERY_HIGH_ERROR > 0:
            err_string += 'Error: High Voltage - Main Battery\n'
        if status & 0x0040 > 0:
            err_string += 'Error: High Voltage - Logic Battery\n'
        if status & 0x0080 > 0:
            err_string += 'Error: Low Voltage - Logic Battery\n'
        if status & 0x0100 > 0:
            err_string += 'Driver Fault - Motor 1 Driver\n'
        if status & 0x0200 > 0:
            err_string += 'Driver Fault - Motor 2 Driver\n'
        if status & 0x0400 > 0:
            err_string += 'Warning: High Voltage - Main Battery\n'
        if status & 0x0800 > 0:
            err_string += 'Warning: Low Voltage - Main Battery\n'
        if status & 0x1000 > 0:
            err_string += 'Warning: High Temperature - Sensor 1\n'
        if status & 0x2000 > 0:
            err_string += 'Warning: High Temperature - Sensor 2\n'
        if status & 0x4000 > 0:
            err_string += 'Home - Motor 1\n'
        if status & 0x8000 > 0:
            err_string += 'Home - Motor 2\n'
    return err_string
