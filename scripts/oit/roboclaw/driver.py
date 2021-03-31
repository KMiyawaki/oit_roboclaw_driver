# -*- coding: utf-8 -*-
import binascii
import serial
import struct
from time import sleep, time

from cmd import Cmd
from constants import Constants, build_pid_qpps_settings_data


class Driver(object):
    def __init__(self, serial, address, serial_interval_after_read, serial_interval_after_write):  # read 0.015, write 0.005
        self.__serial = serial
        self.__address = address
        self.__serial_interval_after_write = serial_interval_after_write
        self.__serial_interval_after_read = serial_interval_after_read

    def __show_bytes(self, bytes):
        result = "["
        for byte in bytes:
            result += str(ord(byte)) + ","
        return result[:-1] + "]"

    def get_port(self):
        return self.__serial

    def get_address(self):
        return self.__address

    def set_address(self, address):
        self.__address = address

    def close(self):
        if self.is_open() == False:
            return

    def open(self):
        if self.is_open() == True:
            return
        self.__serial.open()

    def is_open(self):
        return self.__serial.is_open

    def __write_internal(self, data):
        self.__serial.write(data)
        self.__serial.flushOutput()
        sleep(self.__serial_interval_after_write)

    def __read_internal(self, size):
        recv = self.__serial.read(size)
        self.__serial.flushInput()
        sleep(self.__serial_interval_after_read)
        return recv

    def get_data_no_crc(self, cmd, fmt):
        cmd_bytes = struct.pack('>BB', self.get_address(), cmd)
        self.open()
        self.__write_internal(cmd_bytes)
        return_bytes = self.__read_internal(struct.calcsize(fmt) + 2)
        # print(self.__show_bytes(return_bytes))
        crc_expect = struct.unpack('>H', return_bytes[-2:])[0]
        return return_bytes

    def get_data(self, cmd, fmt):
        cmd_bytes = struct.pack('>BB', self.get_address(), cmd)
        self.open()
        self.__write_internal(cmd_bytes)
        return_bytes = self.__read_internal(struct.calcsize(fmt) + 2)
        crc_actual = binascii.crc_hqx(cmd_bytes + return_bytes[:-2], 0)
        crc_expect = struct.unpack('>H', return_bytes[-2:])[0]
        if crc_actual != crc_expect:
            raise ValueError('CRC failed. return_bytes=' + self.__show_bytes(return_bytes) +
                             ', crc_actual=' + format(crc_actual, '0x') + ", crc_expect=" + format(crc_expect, '0x'))
        return struct.unpack(fmt, return_bytes[:-2])

    def send_command(self, cmd, fmt=None, *data):
        cmd_bytes = struct.pack('>BB', self.get_address(), cmd)
        if fmt is not None and len(data) > 0:
            data_bytes = struct.pack(fmt, *data)
            cmd_bytes = cmd_bytes + data_bytes
        write_crc = binascii.crc_hqx(cmd_bytes, 0) & 0xFFFF
        crc_bytes = struct.pack('>H', write_crc)
        self.open()
        self.__write_internal(cmd_bytes + crc_bytes)
        verification = self.__read_internal(1)
        recv = struct.unpack('>B', verification)[0]
        if 0xff != recv:
            sleep(1)
            raise ValueError(
                'Invailed recv value. (' + str(recv) + ') != 0xFF')

    # Commands 0 - 7 Compatibility Commands

    def __drive(self, motor_number, direction, value):
        cmd = -1
        if motor_number == Constants.CONNECTOR_M1:
            if direction == Constants.FORWARD:
                cmd = Cmd.M1FORWARD
            elif direction == Constants.BACKWARDS:
                cmd = Cmd.M1BACKWARD
        elif motor_number == Constants.CONNECTOR_M2:
            if direction == Constants.FORWARD:
                cmd = Cmd.M2FORWARD
            elif direction == Constants.BACKWARDS:
                cmd = Cmd.M2BACKWARD
        else:
            raise ValueError(
                'UnKnown Motor Number (' + str(motor_number) + ')')
        if cmd < 0:
            raise ValueError(
                'UnKnown Direction (' + str(direction) + ')')
        if value < 0 or value > 127:
            raise ValueError(
                'Invailed speed value (' + str(value) + ')')
        self.send_command(cmd, '>B', value)

    def drive_forward(self, motor_number, value):
        self.__drive(motor_number, Constants.FORWARD, value)

    def drive_backwards(self, motor_number, value):
        self.__drive(motor_number, Constants.BACKWARDS, value)

    def stop_motor(self, motor_number):
        self.drive_forward(motor_number, 0)

    def stop_all_motors(self):
        self.stop_motor(Constants.CONNECTOR_M1)
        self.stop_motor(Constants.CONNECTOR_M2)

    # Advance Packet Serial Commands
    def read_firmware_version(self):
        cmd_bytes = struct.pack('>BB', self.get_address(), Cmd.GETVERSION)
        self.open()
        self.__write_internal(cmd_bytes)
        recv_data = self.__serial.read_until('\0', 48)
        _ = self.__read_internal(2)
        return recv_data

    def read_main_battery_voltage_level(self):
        value = self.get_data(Cmd.GETMBATT, '>H')
        return value[0] / 10.0

    def read_motor_currents(self):
        value = self.get_data(Cmd.GETCURRENTS, '>HH')
        return [value[0] / 100.0, value[1] / 100.0]

    def read_temperature(self):
        value = self.get_data(Cmd.GETTEMP, '>H')
        return value[0] / 10.0

    def read_temperature_2(self):
        value = self.get_data(Cmd.GETTEMP2, '>H')
        return value[0] / 10.0

    # Encoder Commands
    def read_encoder_count(self, motor_number):
        cmd = -1
        if motor_number == Constants.CONNECTOR_M1:
            cmd = Cmd.GETM1ENC
        elif motor_number == Constants.CONNECTOR_M2:
            cmd = Cmd.GETM2ENC
        else:
            raise ValueError(
                'UnKnown Motor Number (' + str(motor_number) + ')')
        value = self.get_data(cmd, '>iB')
        count = value[0]
        # status = value[1]
        # Currently, this function doesn't check over/underflow.
        return count

    def read_encoder_speed(self, motor_number):
        cmd = -1
        if motor_number == Constants.CONNECTOR_M1:
            cmd = Cmd.GETM1SPEED
        elif motor_number == Constants.CONNECTOR_M2:
            cmd = Cmd.GETM2SPEED
        else:
            raise ValueError(
                'UnKnown Motor Number (' + str(motor_number) + ')')
        value = self.get_data(cmd, '>iB')
        pulse = value[0]
        _ = value[1]  # ??? direction
        return pulse

    def read_encoder_counters(self):
        return self.get_data(Cmd.GETENCCOUNTERS, '>ii')

    def reset_quadrature_encoder_counters(self):
        self.send_command(Cmd.RESETENC, '>')

    # Advance Motor Control
    def drive_with_signed_speed(self, motor_number, pulses_per_second):
        cmd = -1
        if motor_number == Constants.CONNECTOR_M1:
            cmd = Cmd.M1SPEED
        elif motor_number == Constants.CONNECTOR_M2:
            cmd = Cmd.M2SPEED
        else:
            raise ValueError(
                'UnKnown Motor Number (' + str(motor_number) + ')')
        self.send_command(cmd, '>i', pulses_per_second)

    def drive_with_signed_speed_acceleration(self, motor_number, accel, speed):
        cmd = -1
        if motor_number == Constants.CONNECTOR_M1:
            cmd = Cmd.M1SPEEDACCEL
        elif motor_number == Constants.CONNECTOR_M2:
            cmd = Cmd.M2SPEEDACCEL
        else:
            raise ValueError(
                'UnKnown Motor Number (' + str(motor_number) + ')')
        self.send_command(cmd, '>Ii', accel, speed)

    def drive_m1m2_with_signed_speed_acceleration(self, accel, speed_m1, speed_m2):
        self.send_command(Cmd.MIXEDSPEEDACCEL, '>Iii',
                          accel, speed_m1, speed_m2)

    def read_motor_velocity_pid_qpps_settings(self, motor_number):
        cmd = -1
        if motor_number == Constants.CONNECTOR_M1:
            cmd = Cmd.READM1PID
        elif motor_number == Constants.CONNECTOR_M2:
            cmd = Cmd.READM2PID
        else:
            raise ValueError(
                'UnKnown Motor Number (' + str(motor_number) + ')')
        PID = build_pid_qpps_settings_data()
        recv = self.get_data(cmd, '>IIII')
        return PID(recv[0] / 65536.0, recv[1] / 65536.0, recv[2] / 65536.0, recv[3])

    # Temporary implementation.
    def is_emergency_pushed(self):
        data = self.get_data_no_crc(Cmd.GETERROR, '>H')
        for d in data:
            if ord(d) > 0:
                return True
        return False
