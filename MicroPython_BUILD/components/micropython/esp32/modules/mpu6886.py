#
# This file is part of MicroPython MPU6886 driver
# Copyright (c) 2018 Mika Tuupola
#
# Port from MPU9250 to MPU6886 by
# Copyright (c) 2020 Martin Korinek
#
# Licensed under the MIT license:
#   http://www.opensource.org/licenses/mit-license.php
#

"""
MicroPython I2C driver for MPU6886 6-axis motion tracking device
"""

__version__ = "0.1.0-dev"

# pylint: disable=import-error
import ustruct
import utime
from machine import I2C, Pin
from micropython import const
from math import sqrt
# pylint: enable=import-error

# register addresses
_GYRO_CONFIG = const(0x1b)
_ACCEL_CONFIG = const(0x1c)
_ACCEL_CONFIG2 = const(0x1d)
_INT_PIN_CFG = const(0x37)
_ACCEL_XOUT_H = const(0x3b)
_ACCEL_XOUT_L = const(0x3c)
_ACCEL_YOUT_H = const(0x3d)
_ACCEL_YOUT_L = const(0x3e)
_ACCEL_ZOUT_H = const(0x3f)
_ACCEL_ZOUT_L = const(0x40)
_TEMP_OUT_H = const(0x41)
_TEMP_OUT_L = const(0x42)
_GYRO_XOUT_H = const(0x43)
_GYRO_XOUT_L = const(0x44)
_GYRO_YOUT_H = const(0x45)
_GYRO_YOUT_L = const(0x46)
_GYRO_ZOUT_H = const(0x47)
_GYRO_ZOUT_L = const(0x48)

_USER_CTRL = const(0x6A)
_PWR_MGMT_1 = const(0x6B)
_PWR_MGMT_2 = const(0x6C)
_CONFIG = const(0x1A)
_FIFO_EN = const(0x23)

_WHO_AM_I = const(0x75)



#_ACCEL_FS_MASK = const(0b00011000)
ACCEL_FS_SEL_2G = const(0b00000000)
ACCEL_FS_SEL_4G = const(0b00001000)
ACCEL_FS_SEL_8G = const(0b00010000)
ACCEL_FS_SEL_16G = const(0b00011000)

_ACCEL_SO_2G = 16384 # 1 / 16384 ie. 0.061 mg / digit
_ACCEL_SO_4G = 8192 # 1 / 8192 ie. 0.122 mg / digit
_ACCEL_SO_8G = 4096 # 1 / 4096 ie. 0.244 mg / digit
_ACCEL_SO_16G = 2048 # 1 / 2048 ie. 0.488 mg / digit

#_GYRO_FS_MASK = const(0b00011000)
GYRO_FS_SEL_250DPS = const(0b00000000)
GYRO_FS_SEL_500DPS = const(0b00001000)
GYRO_FS_SEL_1000DPS = const(0b00010000)
GYRO_FS_SEL_2000DPS = const(0b00011000)

_GYRO_SO_250DPS = 131
_GYRO_SO_500DPS = 62.5
_GYRO_SO_1000DPS = 32.8
_GYRO_SO_2000DPS = 16.4

# Used for enablind and disabling the i2c bypass access
_I2C_BYPASS_MASK = const(0b00000010)
_I2C_BYPASS_EN = const(0b00000010)
_I2C_BYPASS_DIS = const(0b00000000)

SF_G = 1
SF_M_S2 = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 57.295779578552 # 1 rad/s is 57.295779578552 deg/s

class MPU6886:
    """Class which provides interface to MPU6886 6-axis motion tracking device."""
    def __init__(
            self, i2c, address=0x68,
            accel_fs=ACCEL_FS_SEL_2G, gyro_fs=GYRO_FS_SEL_250DPS,
            accel_sf=SF_G, gyro_sf=SF_RAD_S
        ):
        self.i2c = i2c
        self.address = address

        if self.whoami != 0x19:
            raise RuntimeError("MPU6886 not found in I2C bus.")

        self._accel_so = self._accel_fs(accel_fs)
        self._gyro_so = self._gyro_fs(gyro_fs)
        self._accel_sf = accel_sf
        self._gyro_sf = gyro_sf

        # power up sequence
        print("Power up MPU6886 sequence (9s)")
        self._register_char(_PWR_MGMT_1, 0x00)
        utime.sleep(3)
        self._register_char(_PWR_MGMT_1, 0x80)
        utime.sleep(3)
        self._register_char(_PWR_MGMT_1, 0x01)
        utime.sleep(3)


    @property
    def acc(self):
        """
        Acceleration measured by the sensor. By default will return a
        3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
        return values in g if constructor was provided `accel_sf=SF_M_S2`
        parameter.
        """
        so = self._accel_so
        sf = self._accel_sf

        xyz = self._register_three_shorts(_ACCEL_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    @property
    def gyro(self):
        """
        X, Y, Z radians per second as floats.
        """
        so = self._gyro_so
        sf = self._gyro_sf

        xyz = self._register_three_shorts(_GYRO_XOUT_H)
        return tuple([value / so * sf for value in xyz])

    def _is_stationary(self):
        # could also be solved with interrupt from mpu
        return (sqrt(sum([val*val for val in self.acc]))) < 1.1

    def get_dir(self):
        if self._is_stationary():
            if -1.1 < self.acc[0] < -0.9:
                print('x')
            elif 0.9 < self.acc[0] < 1.1:
                print('-x')
            elif -1.1 < self.acc[1] < -0.9:
                print('-y')
            elif 0.9 < self.acc[1] < 1.1:
                print('y')
            elif -1.1 < self.acc[2] < -0.9:
                print('-z')
            elif 0.9 < self.acc[2] < 1.1:
                print('z')
            else:
                print('inclined')
        else:
            print('Moving!')


    @property
    def whoami(self):
        """ Value of the whoami register. """
        return self._register_char(_WHO_AM_I)

    def _register_short(self, register, value=None, buf=bytearray(2)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return ustruct.unpack(">h", buf)[0]

        ustruct.pack_into(">h", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _register_three_shorts(self, register, buf=bytearray(6)):
        self.i2c.readfrom_mem_into(self.address, register, buf)
        return ustruct.unpack(">hhh", buf)

    def _register_char(self, register, value=None, buf=bytearray(1)):
        if value is None:
            self.i2c.readfrom_mem_into(self.address, register, buf)
            return buf[0]

        ustruct.pack_into("<b", buf, 0, value)
        return self.i2c.writeto_mem(self.address, register, buf)

    def _accel_fs(self, value):
        self._register_char(_ACCEL_CONFIG, value)

        # Return the sensitivity divider
        if ACCEL_FS_SEL_2G == value:
            return _ACCEL_SO_2G
        elif ACCEL_FS_SEL_4G == value:
            return _ACCEL_SO_4G
        elif ACCEL_FS_SEL_8G == value:
            return _ACCEL_SO_8G
        elif ACCEL_FS_SEL_16G == value:
            return _ACCEL_SO_16G

    def _gyro_fs(self, value):
        self._register_char(_GYRO_CONFIG, value)

        # Return the sensitivity divider
        if GYRO_FS_SEL_250DPS == value:
            return _GYRO_SO_250DPS
        elif GYRO_FS_SEL_500DPS == value:
            return _GYRO_SO_500DPS
        elif GYRO_FS_SEL_1000DPS == value:
            return _GYRO_SO_1000DPS
        elif GYRO_FS_SEL_2000DPS == value:
            return _GYRO_SO_2000DPS

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        pass
