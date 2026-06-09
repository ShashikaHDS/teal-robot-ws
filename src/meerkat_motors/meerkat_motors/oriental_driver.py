#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import minimalmodbus

# Registers (your mapping)
_FEEDBACK_SPEED_REG_UPPER = 206
_FEEDBACK_SPEED_REG_LOWER = 207

_FEEDBACK_VOLTAGE_REG_UPPER = 327
_FEEDBACK_VOLTAGE_REG_LOWER = 326

# Command registers / constants
MIN_SPEED = 80
MAX_SPEED = 3150
OPERATION_REGISTER_0 = 1153
WRITE_SPEED_REGISTER = 125

MOTOR_FWD = 56
MOTOR_REV = 24
STOP = 48
BRAKE = 40
CLEAR_ALARM = 128

# Alarm registers
ALARM_READ_REGISTER_UPPER = 128
ALARM_READ_REGISTER_LOWER = 129

# Error codes (unchanged)
OPERATION_PREVENTION_ERROR = 70
COMMUNICATION_ERROR = 132
COMMUNICATION_TIMEOUT_ERROR = 133
SENSOR_ERROR_AT_POWER_ON = 66
MAIN_CIRCUIT_OVERHEAT = 33
OVER_VOLTAGE = 34
UNDER_VOLTAGE = 37
SENSOR_ERROR = 40
OVERLOAD = 48
OVERSPEED = 49
EPPROM_ERROR = 65
CPU_ERROR = 240
OVER_CURRENT = 32


def _read_s32_from_u16(hi_u16: int, lo_u16: int) -> int:
    """Combine two 16-bit registers into signed 32-bit integer."""
    raw = ((hi_u16 & 0xFFFF) << 16) | (lo_u16 & 0xFFFF)
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw


class ModBus(object):
    def __init__(self, _port, _slave_address):
        self._port = _port
        self._slave_address = _slave_address

        self.instrument = minimalmodbus.Instrument(self._port, self._slave_address)
        self.instrument.serial.baudrate = 115200
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = minimalmodbus.serial.PARITY_EVEN
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 0.035
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.debug = False

        print(f"Successfully Connected to Slave Address {self._slave_address} ...")

    def writeSpeed(self, speed):
        """speed is signed RPM."""
        try:
            if MIN_SPEED <= speed <= MAX_SPEED:
                self.instrument.write_register(OPERATION_REGISTER_0, int(speed))
                self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_FWD)

            elif -MAX_SPEED <= speed <= -MIN_SPEED:
                self.instrument.write_register(OPERATION_REGISTER_0, int(-speed))
                self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_REV)

            else:
                self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)

        except minimalmodbus.NoResponseError:
            # retry once
            if MIN_SPEED <= speed <= MAX_SPEED:
                self.instrument.write_register(OPERATION_REGISTER_0, int(speed))
                self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_FWD)
            elif -MAX_SPEED <= speed <= -MIN_SPEED:
                self.instrument.write_register(OPERATION_REGISTER_0, int(-speed))
                self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_REV)
            else:
                self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)
            print("Write Speed Error (NoResponseError)")

    def readSpeed(self):
        """
        Read signed 32-bit speed from (UPPER, LOWER).
        Your original code subtracted registers; this combines them properly.
        """
        try:
            hi = self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)
            lo = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER)
            return _read_s32_from_u16(hi, lo)
        except minimalmodbus.NoResponseError:
            hi = self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)
            lo = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER)
            print("Failed to read speed from instrument (retry ok)")
            return _read_s32_from_u16(hi, lo)

    def readVoltage(self):
        """
        Read signed 32-bit voltage raw value from (UPPER, LOWER).
        Many drives report voltage in 0.1V units; your ROS node divides by 10.
        """
        try:
            hi = self.instrument.read_register(_FEEDBACK_VOLTAGE_REG_UPPER)
            lo = self.instrument.read_register(_FEEDBACK_VOLTAGE_REG_LOWER)
            return _read_s32_from_u16(hi, lo)
        except minimalmodbus.NoResponseError:
            hi = self.instrument.read_register(_FEEDBACK_VOLTAGE_REG_UPPER)
            lo = self.instrument.read_register(_FEEDBACK_VOLTAGE_REG_LOWER)
            print("Failed to read voltage from instrument (retry ok)")
            return _read_s32_from_u16(hi, lo)

    def stop(self):
        try:
            return self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)
        except minimalmodbus.NoResponseError:
            return self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)

    def breakMotor(self):
        return self.instrument.write_register(WRITE_SPEED_REGISTER, BRAKE)

    def closeSerial(self):
        print(f"Closing port {self.instrument.serial.port}")
        self.instrument.serial.close()

    def clearAlarm(self):
        return self.instrument.write_register(WRITE_SPEED_REGISTER, CLEAR_ALARM)

    def checkAlarm(self):
        try:
            hi = self.instrument.read_register(ALARM_READ_REGISTER_UPPER)
            lo = self.instrument.read_register(ALARM_READ_REGISTER_LOWER)
            error = _read_s32_from_u16(hi, lo)
        except minimalmodbus.NoResponseError:
            hi = self.instrument.read_register(ALARM_READ_REGISTER_UPPER)
            lo = self.instrument.read_register(ALARM_READ_REGISTER_LOWER)
            error = _read_s32_from_u16(hi, lo)

        if error in [
            OPERATION_PREVENTION_ERROR,
            COMMUNICATION_ERROR,
            COMMUNICATION_TIMEOUT_ERROR,
            SENSOR_ERROR_AT_POWER_ON,
            MAIN_CIRCUIT_OVERHEAT,
            OVER_VOLTAGE,
            UNDER_VOLTAGE,
            SENSOR_ERROR,
            OVERLOAD,
            OVERSPEED,
            EPPROM_ERROR,
            CPU_ERROR,
            OVER_CURRENT
        ]:
            print(f"ALARM {error}... clearing")
            self.clearAlarm()


class OrienDriver(object):
    def __init__(self, _port):
        self._port = _port
        print(f"Connecting to port {self._port} ...")

    def initialize(self, _slave_address):
        return ModBus(self._port, _slave_address)
