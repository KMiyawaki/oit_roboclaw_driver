#!/usr/bin/env python
# -*- coding: utf_8 -*-

import traceback
from serial import Serial
from time import sleep, time
import struct
from PyCRC.CRCCCITT import CRCCCITT as crc

from oit.roboclaw.driver import Driver
from oit.roboclaw.constants import Constants, status_str
from oit.roboclaw.cmd import Cmd


def main():
    sp = None
    try:
        sp = Serial(
            '/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00', 38400, timeout=0.1, write_timeout=0.1, inter_byte_timeout=0.01)
        rc = Driver(0x80, sp)

        print(rc.read_firmware_version())
        print(str(rc.read_main_battery_voltage_level()) + ' V')
        print(str(rc.read_temperature()) + ' Celsius, ' +
              str(rc.read_temperature_2()) + ' Celsius')
        crnts = rc.read_motor_currents()
        print(str('Current1:' + str(crnts[Constants.CONNECTOR_M1])) + ', ' +
              str('Current2:' + str(crnts[Constants.CONNECTOR_M2])))
        print(str('Enc1:' + str(rc.read_encoder_count(Constants.CONNECTOR_M1))) + ', ' +
              str('Enc2:' + str(rc.read_encoder_count(Constants.CONNECTOR_M2))))
        encs = rc.read_encoder_counters()
        print(str('Enc1:' + str(encs[Constants.CONNECTOR_M1])) + ', ' +
              str('Enc2:' + str(encs[Constants.CONNECTOR_M2])))
        print('M1 Settings:' +
              str(rc.read_motor_velocity_pid_qpps_settings(Constants.CONNECTOR_M1)))
        print('M2 Settings:' +
              str(rc.read_motor_velocity_pid_qpps_settings(Constants.CONNECTOR_M2)))
        print('Status:' + status_str(rc.read_status()))

        rc.drive_m1m2_with_signed_speed_acceleration(2000, -1200, 1200)

        for _ in range(0, 3):
            sleep(1)
            crnts = rc.read_motor_currents()
            print(str('Current1:' + str(crnts[Constants.CONNECTOR_M1])) + ', ' +
                  str('Current2:' + str(crnts[Constants.CONNECTOR_M2])))

    except:
        print(traceback.format_exc())

    rc.stop_all_motors()
    sp.close()


if __name__ == '__main__':
    main()
