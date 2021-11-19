// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import math

/**
Driver for the magnetometer of the LSM303DLH module.
*/
class Magnetometer:
  static I2C_ADDRESS ::= 0b11110  // 7.1.3.

  // 8. Register mapping.
  static CRA_REG_M_ ::= 0x00
  static CRB_REG_M_ ::= 0x01
  static MR_REG_M_ ::= 0x02
  static OUT_X_H_M_ ::= 0x03
  static OUT_X_L_M_ ::= 0x04
  static OUT_Y_H_M_ ::= 0x05
  static OUT_Y_L_M_ ::= 0x06
  static OUT_Z_H_M_ ::= 0x07
  static OUT_Z_L_M_ ::= 0x08
  static IRA_REG_M_ ::= 0x0A
  static IRB_REG_M_ ::= 0x0B
  static IRC_REG_M_ ::= 0x0C

  static RATE_0_75HZ ::= 0
  static RATE_1_5HZ  ::= 1
  static RATE_3HZ    ::= 2
  static RATE_7_5HZ  ::= 3
  static RATE_15HZ   ::= 4
  static RATE_30HZ   ::= 5
  static RATE_75HZ   ::= 6

  static RANGE_1_3G ::= 1
  static RANGE_1_9G ::= 2
  static RANGE_2_5G ::= 3
  static RANGE_4_0G ::= 4
  static RANGE_4_7G ::= 5
  static RANGE_5_6G ::= 6
  static RANGE_8_1G ::= 7

  static GAUSS_TO_MICROTESLA_ ::= 100.0

  reg_ /serial.Registers
  gain_xy_ /int := -1
  gain_z_  /int := -1

  constructor dev/serial.Device:
    reg_ = dev.registers

    // There is no who-am-i register on the magnetometer, but the
    // IRx_REG_M registers seem to be constant (x being A, B and C).
    // Section 6. Table 17.
    value := reg_.read_u24_be IRA_REG_M_
    if value != 0x483433: throw "INVALID_CHIP"

  enable -> none
      --rate  /int = RATE_30HZ
      --range /int = RANGE_1_3G
      :
    if not 0 <= rate < 7: throw "INVALID_RATE"
    if not 1 <= range < 8: throw "INVALID_RANGE"

    // 9.2.1, Table 58.
    cra_value := rate << 2
    reg_.write_u8 CRA_REG_M_ cra_value

    // 9.2.2, Table 62.
    // Most significant bits of CRB_REG_M defines the range.
    reg_.write_u8 CRB_REG_M_ (range << 5)
    // Remember the gains (given by the range).
    if range == RANGE_1_3G:
      gain_xy_ = 1055
      gain_z_ = 950
    else if range == RANGE_1_9G:
      gain_xy_ = 795
      gain_z_ = 710
    else if range == RANGE_2_5G:
      gain_xy_ = 635
      gain_z_ = 570
    else if range == RANGE_4_0G:
      gain_xy_ = 430
      gain_z_ = 385
    else if range == RANGE_4_7G:
      gain_xy_ = 375
      gain_z_ = 335
    else if range == RANGE_5_6G:
      gain_xy_ = 320
      gain_z_ = 285
    else if range == RANGE_8_1G:
      gain_xy_ = 230
      gain_z_ = 205

    // 9.2.3, Table 65.
    // Continuous-conversion mode.
    reg_.write_u8 MR_REG_M_ 0x0

  disable -> none:
    // 7.2.3, Table 78.
    // Sleep mode.
    reg_.write_u8 MR_REG_M_ 0b11

  /**
  Reads the magnetic field.
  The returned values are in micro-tesla.
  If a value is out of range, $float.INFINITY is used. In this case
    changing the range (see $enable) might be an option to allow the
    sensor to measure the magnetic field.
  */
  read -> math.Point3f:
    x := reg_.read_i16_be OUT_X_H_M_
    y := reg_.read_i16_be OUT_Y_H_M_
    z := reg_.read_i16_be OUT_Z_H_M_

    x_converted := x * GAUSS_TO_MICROTESLA_ / gain_xy_
    y_converted := y * GAUSS_TO_MICROTESLA_ / gain_xy_
    z_converted := z * GAUSS_TO_MICROTESLA_ / gain_z_

    // 9.2.4 Check for saturation.
    if x == -4096: x_converted = float.INFINITY
    if y == -4096: y_converted = float.INFINITY
    if z == -4096: z_converted = float.INFINITY

    return math.Point3f
        x_converted
        y_converted
        z_converted

  /**
  Reads the raw magnetic field values.
  */
  read --raw -> List:
    if not raw: throw "INVALID_ARGUMENT"
    return [
      reg_.read_i16_be OUT_X_H_M_,
      reg_.read_i16_be OUT_Z_H_M_,
      reg_.read_i16_be OUT_Y_H_M_,
    ]
