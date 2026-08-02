// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import io
import math

/**
Driver for the magnetometer of the LSM303DLH module.
*/
class Magnetometer:
  static I2C-ADDRESS ::= 0b11110  // 7.1.3.

  // 8. Register mapping.
  static CRA-REG-M_ ::= 0x00
  static CRB-REG-M_ ::= 0x01
  static MR-REG-M_ ::= 0x02
  static OUT-X-H-M_ ::= 0x03
  static OUT-X-L-M_ ::= 0x04
  static OUT-Y-H-M_ ::= 0x05
  static OUT-Y-L-M_ ::= 0x06
  static OUT-Z-H-M_ ::= 0x07
  static OUT-Z-L-M_ ::= 0x08
  static IRA-REG-M_ ::= 0x0A
  static IRB-REG-M_ ::= 0x0B
  static IRC-REG-M_ ::= 0x0C

  static RATE-0-75HZ ::= 0
  static RATE-1-5HZ  ::= 1
  static RATE-3HZ    ::= 2
  static RATE-7-5HZ  ::= 3
  static RATE-15HZ   ::= 4
  static RATE-30HZ   ::= 5
  static RATE-75HZ   ::= 6

  static RANGE-1-3G ::= 1
  static RANGE-1-9G ::= 2
  static RANGE-2-5G ::= 3
  static RANGE-4-0G ::= 4
  static RANGE-4-7G ::= 5
  static RANGE-5-6G ::= 6
  static RANGE-8-1G ::= 7

  static GAUSS-TO-MICROTESLA_ ::= 100.0

  reg_ /serial.Registers
  gain-xy_ /int := -1
  gain-z_  /int := -1

  constructor dev/serial.Device:
    reg_ = dev.registers

    // There is no who-am-i register on the magnetometer, but the
    // IRx_REG_M registers seem to be constant (x being A, B and C).
    // Section 6. Table 17.
    value := reg_.read-u24-be IRA-REG-M_
    if value != 0x483433: throw "INVALID_CHIP"

  enable -> none
      --rate  /int = RATE-7-5HZ
      --range /int = RANGE-1-3G:
    if not 0 <= rate < 7: throw "INVALID_RATE"
    if not 1 <= range < 8: throw "INVALID_RANGE"

    // 9.2.1, Table 58.
    cra-value := rate << 2
    reg_.write-u8 CRA-REG-M_ cra-value

    // 9.2.2, Table 62.
    // Most significant bits of CRB_REG_M defines the range.
    reg_.write-u8 CRB-REG-M_ (range << 5)
    // Remember the gains (given by the range).
    if range == RANGE-1-3G:
      gain-xy_ = 1055
      gain-z_ = 950
    else if range == RANGE-1-9G:
      gain-xy_ = 795
      gain-z_ = 710
    else if range == RANGE-2-5G:
      gain-xy_ = 635
      gain-z_ = 570
    else if range == RANGE-4-0G:
      gain-xy_ = 430
      gain-z_ = 385
    else if range == RANGE-4-7G:
      gain-xy_ = 375
      gain-z_ = 335
    else if range == RANGE-5-6G:
      gain-xy_ = 320
      gain-z_ = 285
    else if range == RANGE-8-1G:
      gain-xy_ = 230
      gain-z_ = 205

    // 9.2.3, Table 65.
    // Continuous-conversion mode.
    reg_.write-u8 MR-REG-M_ 0x0

  disable -> none:
    // 7.2.3, Table 78.
    // Sleep mode.
    reg_.write-u8 MR-REG-M_ 0b11

  /**
  Reads the magnetic field.
  The returned values are in micro-tesla.
  If a value is out of range, $float.INFINITY is used. In this case
    changing the range (see $enable) might be an option to allow the
    sensor to measure the magnetic field.
  */
  read -> math.Point3f:
    raw := read-raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

    x-converted := x * GAUSS-TO-MICROTESLA_ / gain-xy_
    y-converted := y * GAUSS-TO-MICROTESLA_ / gain-xy_
    z-converted := z * GAUSS-TO-MICROTESLA_ / gain-z_

    // 9.2.4 Check for saturation.
    if x == -4096: x-converted = float.INFINITY
    if y == -4096: y-converted = float.INFINITY
    if z == -4096: z-converted = float.INFINITY

    return math.Point3f
        x-converted
        y-converted
        z-converted

  /**
  Reads the raw magnetic field values.

  A value of -4096 means that the sensor was saturated at that range. Consider
    calling the $enable function with a wider range.
  */
  read --raw -> List:
    if not raw: throw "INVALID_ARGUMENT"
    return read-raw_

  read-raw_ -> List:
    bytes := reg_.read-bytes OUT-X-H-M_ 6
    return [
      io.BIG-ENDIAN.int16 bytes 0,
      io.BIG-ENDIAN.int16 bytes 2,
      io.BIG-ENDIAN.int16 bytes 4,
    ]
