#!/usr/bin/env python
# -*- coding: utf-8 -*-

import minimalmodbus
import contextlib

_WRITE_REGISTER = 125
_FEEDBACK_SPEED_REG_LOWER = 207
_FEEDBACK_SPEED_REG_UPPER = 206

_WRITE_REGISTER_SPEED = 1153
_FWD_DEC = 56
_REV_DEC = 24

_MIN_RPM = 80
_MAX_RPM = 3150

_FEEDBACK_VOLTAGE_REG_LOWER = 326
_FEEDBACK_VOLATGE_REG_UPPER = 327

# NEW REGISTER VALUES
INVERTER_VOLTAGE_READ_REGISTER = 326
MIN_SPEED = 80
MAX_SPEED = 3150
OPERATION_REGISTER_0 = 1153
WRITE_SPEED_REGISTER = 125
MOTOR_FWD = 56
MOTOR_REV = 24
STOP = 48
SPEED_READ_REGISTER = 206
BRAKE = 40
CLEAR_ALARM = 128

#ERRORS
ALARM_READ_REGISTER_LOWER = 129
ALARM_READ_REGISTER_UPPER = 128

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


speed = 0
voltage = 0

class ModBus(object):

    """
        ModBus class for talking to instruments (slaves).
        Uses the minimalmodbus python library.
    Args:
        * port (str): The serial port name, for example ``/dev/ttyUSB0`` (Linux),
          ``/dev/tty.usbserial`` (OS X) or ``COM4`` (Windows).
        * slaveaddress (int): Slave address in the range 1 to 247 (use decimal numbers,
          not hex). Address 0 is for broadcast, and 248-255 are reserved.
    """

    def __init__(self, _port, _slave_address):

        self._port = _port
        self._slave_address = _slave_address


        self.instrument = minimalmodbus.Instrument(self._port, self._slave_address)
        self.instrument.serial.baudrate = 115200
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity  = minimalmodbus.serial.PARITY_EVEN
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout  = 0.035
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.debug = False
            

        print("Successfully Connected to Slave Address {} ...".format(self._slave_address))

    def writeSpeed(self, speed):
        try :
            if (speed >= MIN_SPEED and speed <= MAX_SPEED):
                try:
                    self.instrument.write_register(OPERATION_REGISTER_0, speed)
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(OPERATION_REGISTER_0, speed)
                try:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_FWD) # run motor forward with default acceleration
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_FWD) 

            elif (speed <= -MIN_SPEED and speed >= -MAX_SPEED):
                try:
                    self.instrument.write_register(OPERATION_REGISTER_0, -speed)
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(OPERATION_REGISTER_0, -speed)
                try:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_REV) # run motor backward with default acceleration
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_REV) 

            else:
                # self.instrument.write_register(_WRITE_REGISTER, 2) # use this if wants to stop instant (not recommended)
                try:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, STOP) # stop motor with default deceleartion
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, STOP) 
        except minimalmodbus.NoResponseError:
            if (speed >= MIN_SPEED and speed <= MAX_SPEED):
                try:
                    self.instrument.write_register(OPERATION_REGISTER_0, speed)
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(OPERATION_REGISTER_0, speed)
                try:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_FWD) # run motor forward with default acceleration
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_FWD) 

            elif (speed <= -MIN_SPEED and speed >= -MAX_SPEED):
                try:
                    self.instrument.write_register(OPERATION_REGISTER_0, -speed)
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(OPERATION_REGISTER_0, -speed)
                try:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_REV) # run motor backward with default acceleration
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, MOTOR_REV) 

            else:
                # self.instrument.write_register(_WRITE_REGISTER, 2) # use this if wants to stop instant (not recommended)
                try:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, STOP) # stop motor with default deceleartion
                except minimalmodbus.NoResponseError:
                    self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)
            print("Write Speed Error")
            pass


    def readSpeed(self):
        global speed
        try:
            speed = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER)-self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)
        except minimalmodbus.NoResponseError:
            speed = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER)-self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)
            print("Failed to read from instrument")
        return speed
    
    def readVoltage(self):
        global voltage
        try: 
            voltage = self.instrument.read_register(_FEEDBACK_VOLTAGE_REG_LOWER)-self.instrument.read_register(_FEEDBACK_VOLATGE_REG_UPPER)
        except minimalmodbus.NoResponseError:
            voltage = self.instrument.read_register(_FEEDBACK_VOLTAGE_REG_LOWER)-self.instrument.read_register(_FEEDBACK_VOLATGE_REG_UPPER)
            print("Failed to read from instrument")
        return voltage
    
    def stop(self):
        try:
            return self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)
        except minimalmodbus.NoResponseError:
            return self.instrument.write_register(WRITE_SPEED_REGISTER, STOP)
    
    def breakMotor(self):
        return self.instrument.write_register(WRITE_SPEED_REGISTER, BRAKE)
    
    def closeSerial(self):
        print("Closing port {}".format(self.instrument.serial.port))
        self.instrument.serial.close()

    def clearAlarm(self):
        return self.instrument.write_register(WRITE_SPEED_REGISTER, CLEAR_ALARM)
    
    def checkAlarm(self):
        error = self.instrument.read_register(ALARM_READ_REGISTER_LOWER) - self.instrument.read_register(ALARM_READ_REGISTER_UPPER) 
        if error == OPERATION_PREVENTION_ERROR:
            print("OPERATION_PREVENTION_ERROR...clearning")
            self.clearAlarm()
            
        elif error == COMMUNICATION_ERROR:
            print("COMMUNICATION_ERROR...clearning")
            self.clearAlarm()
            
        elif error == SENSOR_ERROR_AT_POWER_ON:
            print("SENSOR_ERROR_AT_POWER_ON...clearning")
            self.clearAlarm()

        elif error == COMMUNICATION_TIMEOUT_ERROR:
            print("COMMUNICATION_TIMEOUT_ERROR...clearning")
            self.clearAlarm()

        elif error == MAIN_CIRCUIT_OVERHEAT:
            print("MAIN_CIRCUIT_OVERHEAT...clearning")
            self.clearAlarm()

        elif error == OVER_VOLTAGE:
            print("OVER_VOLTAGE...clearning")
            self.clearAlarm()

        elif error == UNDER_VOLTAGE:
            print("UNDER_VOLTAGE...clearning")
            self.clearAlarm()

        elif error == SENSOR_ERROR:
            print("SENSOR_ERROR...clearning")
            self.clearAlarm()

        elif error == OVERLOAD:
            print("OVERLOAD...clearning")
            self.clearAlarm()

        elif error == OVERSPEED:
            print("OVERSPEED...clearning")
            self.clearAlarm()

        elif error == EPPROM_ERROR:
            print("EPPROM_ERROR...clearning")
            self.clearAlarm()


        elif error == CPU_ERROR:
            print("CPU_ERROR...clearning")
            self.clearAlarm()

        elif error == OVER_CURRENT:
            print("OVER_CURRENT...clearning")
            self.clearAlarm()

class OrienDriver(object):
    def __init__(self, _port): 
        self._port = _port

        print("Connecting to port {} ...".format(self._port))

    def initialize(self, _slave_address):
        return ModBus(self._port, _slave_address)