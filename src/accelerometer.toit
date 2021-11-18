// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import math

/**
Driver for the accelerometer of the LSM303DLH module.
*/
class Accelerometer:
  static I2C_ADDRESS ::= 0b0011000  // 7.1.2.
  static I2C_ADDRESS_ALT ::= 0b0011001  // 7.1.2.

  /** Low power 0.5Hz output data rate. */
  static RATE_0_5HZ   ::= 0b01000
  /** Low power 1Hz output data rate. */
  static RATE_1HZ   ::= 0b01100
  /** Low power 2Hz output data rate. */
  static RATE_2HZ   ::= 0b10000
  /** Low power 5Hz output data rate. */
  static RATE_5HZ   ::= 0b10100
  /** Low power 10Hz output data rate. */
  static RATE_10HZ   ::= 0b11000
  /** Normal power 50Hz output data rate. */
  static RATE_50HZ   ::= 0b00100
  /** Normal power 100Hz output data rate. */
  static RATE_100HZ  ::= 0b00101
  /** Normal power 400Hz output data rate. */
  static RATE_400HZ  ::= 0b00110
  /** Normal power 1000Hz output data rate. */
  static RATE_1000HZ ::= 0b00111

  static RANGE_2G  ::= 0
  static RANGE_4G  ::= 1
  static RANGE_8G  ::= 3

  // 8. Register mapping.
  static CTRL_REG1_A_ ::= 0x20
  static CTRL_REG4_A_ ::= 0x23
  static OUT_X_L_A_ ::= 0x28
  static OUT_X_H_A_ ::= 0x29
  static OUT_Y_L_A_ ::= 0x2A
  static OUT_Y_H_A_ ::= 0x2B
  static OUT_Z_L_A_ ::= 0x2C
  static OUT_Z_H_A_ ::= 0x2D

  /**
  Standard acceleration due to gravity.
  In m/s².
  */
  static GRAVITY_STANDARD_ ::= 9.80665

  reg_ /serial.Registers

  constructor dev/serial.Device:
    reg_ = dev.registers

  /**
  Enables the sensor.

  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE_0_5HZ
  - $RATE_1HZ
  - $RATE_2HZ
  - $RATE_5HZ
  - $RATE_10HZ
  - $RATE_50HZ
  - $RATE_100HZ
  - $RATE_400HZ
  - $RATE_1000HZ

  The $range parameter defines the measured acceleration range.
  Valid values for $range are:
  - $RANGE_2G: +-2G (19.61 m/s²)
  - $RANGE_4G: +-4G (39.23 m/s²)
  - $RANGE_8G: +-8G (78.45 m/s²)
  */
  enable -> none
      --rate  /int = RATE_100HZ
      --range /int = RANGE_2G:

    // 9.1.1. CTRL_REG1_A.
    VALID_RATES ::= [RATE_0_5HZ, RATE_1HZ, RATE_2HZ, RATE_5HZ, RATE_10HZ, RATE_50HZ, RATE_100HZ, RATE_400HZ, RATE_1000HZ]
    if not VALID_RATES.contains rate: throw "INVALID_RATE"

    // We always enable all three axes.
    axes_bits := 0b111

    reg1_value := (rate << 3) |  axes_bits

    // 7.1.4. CTRL_REG4_A.
    // Table 27. CTRL_REG4_A description.
    // Set Block data update to 0. (continuous update).
    // Set Big/little endian data selection to 0. (LSB at lower address).
    reg4_value := 0

    if range != RANGE_2G and range != RANGE_4G and range != RANGE_8G: throw "INVALID_RANGE"
    reg4_value |= range << 4

    reg_.write_u8 CTRL_REG1_A_ reg1_value
    reg_.write_u8 CTRL_REG4_A_ reg4_value

    sleep --ms=10


  /**
  Disables the accelerometer.
  Initiates a power-down of the peripheral. It is safe to call $enable
    to restart the accelerometer.
  */
  disable:
    // Fundamentally we only care for the rate-bits: as long as they
    // are 0, the device is disabled.
    // It's safe to change the other bits as well.
    reg_.write_u8 CTRL_REG1_A_ 0x00

  /**
  Reads the x, y and z axis.
  The returned values are in in m/s².
  */
  read -> math.Point3f:
    x_low  := reg_.read_u8 OUT_X_L_A_
    x_high := reg_.read_i8 OUT_X_H_A_
    y_low  := reg_.read_u8 OUT_Y_L_A_
    y_high := reg_.read_i8 OUT_Y_H_A_
    z_low  := reg_.read_u8 OUT_Z_L_A_
    z_high := reg_.read_i8 OUT_Z_H_A_

    // Only 12 bits are used.
    x := (x_high << 4) + (x_low >> 4)
    y := (y_high << 4) + (y_low >> 4)
    z := (z_high << 4) + (z_low >> 4)

    // The scaling (range) affects the value, so we need to read that one.
    // We could also cache the current scaling so we don't need to do yet
    // another I2C call.
    range := read_range

    // Section 2.1, table3:
    // The linear acceleration sensitivity depends on the range:
    // - RANGE_2G:   1mg/LSB
    // - RANGE_4G:   2mg/LSB
    // - RANGE_8G:   3.9mg/LSB
    gain := ?
    if range == RANGE_2G: gain = 1.0
    else if range == RANGE_4G: gain = 2.0
    else: gain = 3.9

    factor := gain * (GRAVITY_STANDARD_ / 1000.0)  // Constant folded because it's one expression.
    return math.Point3f
        x * factor
        y * factor
        z * factor

  /**
  Reads the current range setting of the sensor.
  Returns $RANGE_2G, $RANGE_4G, or $RANGE_8G.
  */
  read_range -> int:
    reg4 := reg_.read_u8 CTRL_REG4_A_
    return (reg4 >> 4) & 0b11
