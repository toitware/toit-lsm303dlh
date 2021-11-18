// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import lsm303dlh show *
import device show FlashStore

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  accelerometer := Accelerometer (bus.device Accelerometer.I2C_ADDRESS)
  magnetometer := Magnetometer (bus.device Magnetometer.I2C_ADDRESS)

  accelerometer.enable
  magnetometer.enable

  100.repeat:
    acceleration := accelerometer.read
    print "Acceleration (in m/s²): $acceleration"

    field := magnetometer.read
    print "Magnetic field (in microtesla): $field"

    field2 := magnetometer.read --raw
    print "Magnetic field (raw): $field2"

    sleep --ms=500
