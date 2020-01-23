#!/usr/bin/env python
# -*- coding: utf_8 -*-

import copy
import traceback
from serial import Serial
from time import sleep, time
from oit.roboclaw.driver import Driver
from oit.roboclaw.constants import Constants


def main():
    sp = None
    try:
        sp = Serial(
            '/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00', 38400, timeout=0.1, write_timeout=0.1, inter_byte_timeout=0.005)
        rcs = [Driver(sp, 0x80, 0.005, 0.005), Driver(sp, 0x81, 0.005, 0.005)]

        for rc in rcs:
            print("*-- Board " + str(rc.get_address()) + " --*")
            print(rc.read_firmware_version())
            print(str(rc.read_main_battery_voltage_level()) + ' V')
            print(str(rc.read_temperature()) + ' Celsius, ' +
                  str(rc.read_temperature_2()) + ' Celsius')
            rc.reset_quadrature_encoder_counters()
            rc.drive_m1m2_with_signed_speed_acceleration(2000, -100, 100)

        for i in range(0, 100):
            for rc in rcs:
                try:
                    print(str(rc.get_address()))
                    print(rc.read_encoder_counters())
                    # rc.drive_m1m2_with_signed_speed_acceleration(2000, -100, 100)
                except:
                    print(traceback.format_exc())
            sleep(0.01)
        for rc in rcs:
            rc.stop_all_motors()
    except:
        print(traceback.format_exc())

    sp.close()


if __name__ == '__main__':
    main()
