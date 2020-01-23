# -*- coding: utf-8 -*-
from serial import Serial
from oit.roboclaw.device_conf import DeviceConf
from oit.roboclaw.driver import Driver
from oit.roboclaw.constants import Constants


class DeviceMap(object):
    def __init__(self, device_conf, motor_accl):
        self.clear()
        self.device_conf = device_conf
        self.__motor_accl = motor_accl
        self.__motor_index_to_roboclaw_connector = [
            None] * len(self.device_conf.motor_index)
        for serial_setting in self.device_conf.serial_settings:
            self.__build_serial(serial_setting)

    def __build_serial(self, serial_setting):
        serial = Serial(serial_setting.port, serial_setting.baud_rate, timeout=serial_setting.timeout,
                        write_timeout=serial_setting.write_timeout, inter_byte_timeout=serial_setting.inter_byte_timeout)
        self.__serials.append(serial)
        for roboclaw_setting in serial_setting.roboclaw_settings:
            self.__build_roboclaw(serial, roboclaw_setting)

    def __build_roboclaw(self, serial_port, roboclaw_setting):
        roboclaw = Driver(serial_port, roboclaw_setting.address,
                          roboclaw_setting.serial_interval_after_read, roboclaw_setting.serial_interval_after_write)
        roboclaw_index = len(self.__roboclaws)
        self.__roboclaws.append(roboclaw)
        if roboclaw_setting.motor_index_of_connector_M1 is not None:
            self.__add_map_entry(roboclaw_setting.motor_index_of_connector_M1,
                                 roboclaw_index, Constants.CONNECTOR_M1)
        if roboclaw_setting.motor_index_of_connector_M2 is not None:
            self.__add_map_entry(roboclaw_setting.motor_index_of_connector_M2,
                                 roboclaw_index, Constants.CONNECTOR_M2)

    def __add_map_entry(self, motor_index, roboclaw_index, motor_connector):
        self.__motor_index_to_roboclaw_connector[motor_index] = (
            roboclaw_index, motor_connector)
        self.__roboclaw_connector_to_motor_index[(
            roboclaw_index, motor_connector)] = motor_index

    def clear(self):
        self.__serials = []
        self.__roboclaws = []
        self.__motor_index_to_roboclaw_connector = []
        self.__roboclaw_connector_to_motor_index = {}

    def set_motor_vels(self, vels):
        size = len(vels)
        vel_packs = []
        for i in range(0, self.get_roboclaw_size()):
            vel_packs.append([0, 0])

        for i in range(0, size):
            roboclaw_connector = self.get_roboclaw_connector_from_motor_index(
                i)
            roboclaw_index = roboclaw_connector[0]
            motor_connector = roboclaw_connector[1]
            vel_packs[roboclaw_index][motor_connector] = vels[i]

        size = len(vel_packs)
        for i in range(0, size):
            v = vel_packs[i]
            self.__roboclaws[i].drive_m1m2_with_signed_speed_acceleration(
                self.__motor_accl, v[Constants.CONNECTOR_M1], v[Constants.CONNECTOR_M2])

    def get_encoder_ticks(self):
        size = self.get_roboclaw_size()
        results = [0] * self.get_motor_size()
        for i in range(0, size):
            ticks = self.__roboclaws[i].read_encoder_counters()
            for j in range(0, len(ticks)):
                try:
                    motor_index = self.get_motor_index_from_roboclaw_connector(
                        i, j)
                    results[motor_index] = ticks[j]
                except:
                    pass
        return results

    def is_emergency_pushed(self):
        size = self.get_roboclaw_size()
        results = []
        for i in range(0, size):
            if self.__roboclaws[i].is_emergency_pushed() == True:
                return True
        return False

    def get_roboclaw_connector_from_motor_index(self, motor_index):
        return self.__motor_index_to_roboclaw_connector[motor_index]

    def get_motor_index_from_roboclaw_connector(self, roboclaw_index, motor_connector):
        return self.__roboclaw_connector_to_motor_index[(roboclaw_index, motor_connector)]

    def get_motor_size(self):
        return len(self.__motor_index_to_roboclaw_connector)

    def get_roboclaw_size(self):
        return len(self.__roboclaws)

    def get_roboclaws(self):
        return self.__roboclaws

    def get_serials(self):
        return self.__serials
