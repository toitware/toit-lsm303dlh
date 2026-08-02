// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import io
import math

/**
Driver for the accelerometer of the LSM303DLH module.
*/
class Accelerometer:
  static I2C-ADDRESS ::= 0b0011000  // 7.1.2.
  static I2C-ADDRESS-ALT ::= 0b0011001  // 7.1.2.

  /** Low power 0.5Hz output data rate. */
  static RATE-0-5HZ ::= 0b01000
  /** Low power 1Hz output data rate. */
  static RATE-1HZ ::= 0b01100
  /** Low power 2Hz output data rate. */
  static RATE-2HZ ::= 0b10000
  /** Low power 5Hz output data rate. */
  static RATE-5HZ ::= 0b10100
  /** Low power 10Hz output data rate. */
  static RATE-10HZ ::= 0b11000
  /** Normal power 50Hz output data rate. */
  static RATE-50HZ ::= 0b00100
  /** Normal power 100Hz output data rate. */
  static RATE-100HZ ::= 0b00101
  /** Normal power 400Hz output data rate. */
  static RATE-400HZ ::= 0b00110
  /** Normal power 1000Hz output data rate. */
  static RATE-1000HZ ::= 0b00111

  static RANGE-2G  ::= 0
  static RANGE-4G  ::= 1
  static RANGE-8G  ::= 3

  // 8. Register mapping.
  static CTRL-REG1-A_ ::= 0x20
  static CTRL-REG4-A_ ::= 0x23
  static OUT-X-L-A_ ::= 0x28
  static OUT-X-H-A_ ::= 0x29
  static OUT-Y-L-A_ ::= 0x2A
  static OUT-Y-H-A_ ::= 0x2B
  static OUT-Z-L-A_ ::= 0x2C
  static OUT-Z-H-A_ ::= 0x2D

  static BDU-BIT_ ::= 1 << 7
  static AUTO-INCREMENT-BIT_ ::= 1 << 7

  /**
  Standard acceleration due to gravity.
  In m/s².
  */
  static GRAVITY-STANDARD_ ::= 9.80665

  reg_ /serial.Registers
  range_ /int := 0

  constructor dev/serial.Device:
    reg_ = dev.registers

  /**
  Enables the sensor.

  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE-0-5HZ
  - $RATE-1HZ
  - $RATE-2HZ
  - $RATE-5HZ
  - $RATE-10HZ
  - $RATE-50HZ
  - $RATE-100HZ
  - $RATE-400HZ
  - $RATE-1000HZ

  The $range parameter defines the measured acceleration range.
  Valid values for $range are:
  - $RANGE-2G: +-2G (19.61 m/s²)
  - $RANGE-4G: +-4G (39.23 m/s²)
  - $RANGE-8G: +-8G (78.45 m/s²)
  */
  enable -> none
      --rate  /int = RATE-100HZ
      --range /int = RANGE-2G:

    // 9.1.1. CTRL_REG1_A.
    VALID-RATES ::= [RATE-0-5HZ, RATE-1HZ, RATE-2HZ, RATE-5HZ, RATE-10HZ, RATE-50HZ, RATE-100HZ, RATE-400HZ, RATE-1000HZ]
    if not VALID-RATES.contains rate: throw "INVALID_RATE"

    // We always enable all three axes.
    axes-bits := 0b111

    reg1-value := (rate << 3) |  axes-bits

    // 9.1.4. CTRL_REG4_A.
    // Table 27. CTRL_REG4_A description.
    // Set Big/little endian data selection to 0. (LSB at lower address).
    reg4-value := 0
    // Prevent an update while output bytes are being read.
    reg4-value |= BDU-BIT_

    if range != RANGE-2G and range != RANGE-4G and range != RANGE-8G: throw "INVALID_RANGE"
    reg4-value |= range << 4
    range_ = range

    reg_.write-u8 CTRL-REG1-A_ reg1-value
    reg_.write-u8 CTRL-REG4-A_ reg4-value

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
    reg_.write-u8 CTRL-REG1-A_ 0x00

  /**
  Reads the x, y and z axis.
  The returned values are in in m/s².
  */
  read -> math.Point3f:
    raw := read-raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

    // Only 12 bits are used.
    x >>= 4
    y >>= 4
    z >>= 4

    // Section 2.1, table3:
    // The linear acceleration sensitivity depends on the range:
    // - RANGE_2G:   1mg/LSB
    // - RANGE_4G:   2mg/LSB
    // - RANGE_8G:   3.9mg/LSB
    gain := ?
    if range_ == RANGE-2G: gain = 1.0
    else if range_ == RANGE-4G: gain = 2.0
    else: gain = 3.9

    factor := gain * (GRAVITY-STANDARD_ / 1000.0)  // Constant folded because it's one expression.
    return math.Point3f
        x * factor
        y * factor
        z * factor

  /**
  Reads the current range setting of the sensor.
  Returns $RANGE-2G, $RANGE-4G, or $RANGE-8G.
  */
  read-range -> int:
    reg4 := reg_.read-u8 CTRL-REG4-A_
    return (reg4 >> 4) & 0b11

  read-raw_ -> List:
    bytes := reg_.read-bytes (OUT-X-L-A_ | AUTO-INCREMENT-BIT_) 6
    return [
      io.LITTLE-ENDIAN.int16 bytes 0,
      io.LITTLE-ENDIAN.int16 bytes 2,
      io.LITTLE-ENDIAN.int16 bytes 4,
    ]
