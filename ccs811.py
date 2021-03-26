#!/usr/bin/env python

import os
import smbus2 as smbus
from collections import OrderedDict
from logging import basicConfig, getLogger, DEBUG, FileHandler, Formatter
from time import sleep

CCS811_ADDRESS  =  0x5B
CCS811_ADDRESS_PRE  =  0x5A

CCS811_STATUS = 0x00
CCS811_MEAS_MODE = 0x01
CCS811_ALG_RESULT_DATA = 0x02
CCS811_HW_ID = 0x20

CCS811_DRIVE_MODE_IDLE = 0x00
CCS811_DRIVE_MODE_1SEC = 0x01
CCS811_DRIVE_MODE_10SEC = 0x02
CCS811_DRIVE_MODE_60SEC = 0x03
CCS811_DRIVE_MODE_250MS = 0x04

CCS811_BOOTLOADER_APP_START = 0xF4

CCS811_HW_ID_CODE = 0x81

class CCS811:

    def __init__(self, mode = CCS811_DRIVE_MODE_1SEC, address = CCS811_ADDRESS,address_pre = CCS811_ADDRESS_PRE):

        if mode not in [CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS]:
            raise ValueError('Unexpected mode value {0}.  Set mode to one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC or CCS811_DRIVE_MODE_250MS'.format(mode))

        self._address = address
        self._address_pre = address_pre
        self._bus = smbus.SMBus(1)
        self._decide_address = ""

        self._status = Bitfield([('ERROR' , 1), ('unused', 2), ('DATA_READY' , 1), ('APP_VALID', 1), ('unused2' , 2), ('FW_MODE' , 1)])

        self._meas_mode = Bitfield([('unused', 2), ('INT_THRESH', 1), ('INT_DATARDY', 1), ('DRIVE_MODE', 3)])

        self._error_id = Bitfield([('WRITE_REG_INVALID', 1), ('READ_REG_INVALID', 1), ('MEASMODE_INVALID', 1), ('MAX_RESISTANCE', 1), ('HEATER_FAULT', 1), ('HEATER_SUPPLY', 1)])

        self._TVOC = 0
        self._eCO2 = 0
        #check that the HW id is correct
        if self.readU8(CCS811_HW_ID) != CCS811_HW_ID_CODE:
            raise Exception("Device ID returned is not correct! Please check your wiring.")

        self.writeList(CCS811_BOOTLOADER_APP_START, [])
        sleep(0.1)

        #make sure there are no errors and we have entered application mode
        if self.checkError():
            raise Exception("Device returned an Error! Try removing and reapplying power to the device and running the code again.")
        if not self._status.FW_MODE:
            raise Exception("Device did not enter application mode! If you got here, there may be a problem with the firmware on your sensor.")

        self.disableInterrupt()

        self.setDriveMode(mode)


    def disableInterrupt(self):
        self._meas_mode.INT_DATARDY = 1
        self.write8(CCS811_MEAS_MODE, self._meas_mode.get())

    def setDriveMode(self, mode):
        self._meas_mode.DRIVE_MODE = mode
        self.write8(CCS811_MEAS_MODE, self._meas_mode.get())

    def available(self):
        self._status.set(self.readU8(CCS811_STATUS))
        if not self._status.DATA_READY:
            return False
        else:
            return True

    def readData(self):
        if not self.available():
            return False
        else:
            buf = self.readList(CCS811_ALG_RESULT_DATA, 8)
            self._eCO2 = (buf[0] << 8) | (buf[1])
            self._TVOC = (buf[2] << 8) | (buf[3])
            if self._status.ERROR:
                return buf[5]
            else:
                return 0

    def getTVOC(self):
        return self._TVOC

    def geteCO2(self):
        return self._eCO2,self._decide_address

    def checkError(self):
        self._status.set(self.readU8(CCS811_STATUS))
        return self._status.ERROR

    def readU8(self, register):
        try:
            result = self._bus.read_byte_data(self._address, register) & 0xFF
            self._decide_address = "0xB5"
        except:
            result = self._bus.read_byte_data(self._address_pre, register) & 0xFF
            self._decide_address = "0xA5"
        return result

    def write8(self, register, value):
        value = value & 0xFF
        try:
            self._bus.write_byte_data(self._address, register, value)
        except:
            self._bus.write_byte_data(self._address_pre, register, value)

    def readList(self, register, length):
        try:
            results = self._bus.read_i2c_block_data(self._address, register, length)
        except:
            results = self._bus.read_i2c_block_data(self._address_pre, register, length)
        return results

    def writeList(self, register, data):
        try:
            self._bus.write_i2c_block_data(self._address, register, data)

        except:
            self._bus.write_i2c_block_data(self._address_pre, register, data)
            
class Bitfield:
    def __init__(self, _structure):
        self._structure = OrderedDict(_structure)
        for key, value in self._structure.items():
            setattr(self, key, 0)

    def get(self):
        fullreg = 0
        pos = 0
        for key, value in self._structure.items():
            fullreg = fullreg | ( (getattr(self, key) & (2**value - 1)) << pos )
            pos = pos + value

        return fullreg

    def set(self, data):
        pos = 0
        for key, value in self._structure.items():
            setattr(self, key, (data >> pos) & (2**value - 1))
            pos = pos + value
