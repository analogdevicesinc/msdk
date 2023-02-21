/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_PAG7920_REGS_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_PAG7920_REGS_H_

// clang-format off
// I2C address configuration
#define PAG7920_I2C_ADDR_GPIO2_FLOAT    0x35
#define PAG7920_I2C_ADDR_GPIO2_HIGH     0x25
#define PAG7920_I2C_ADDR_GPIO2_LOW      0x40
#define PAG7920_I2C_ADDR_BROADCAST      0x60

// Register address definitions
#define REG_BANK            (0xEF)
#define REG_UPDATE          (0xEB)
#define REG_MODE            (0x30)
#define REG_TRIG            (0x31)
#define REG_RESET           (0xEE)
#define REG_RWOI_EN         (0x55)
#define REG_RWOI_HSIZE_L    (0x21)
#define REG_RWOI_HSIZE_H    (0x22)
#define REG_RWOI_VSIZE_L    (0x23)
#define REG_RWOI_VSIZE_H    (0x24)
#define REG_RWOI_HSTART_L   (0x25)
#define REG_RWOI_HSTART_H   (0x26)
#define REG_RWOI_VSTART_L   (0x27)
#define REG_RWOI_VSTART_H   (0x28)
#define REG_MIDH            (0x01)
#define REG_MIDL            (0x00)
#define REG_PIDH            (0x01)
#define REG_PIDL            (0x00)
#define REG_AE_MAX_GAIN_L   (0x42)
#define REG_AE_MAX_GAIN_H   (0x43)

// Settings
#define MAX_V_SIZE          240
#define MAX_H_SIZE          320
#define R_AE_MAX_GAIN_1     29
// clang-format on

#endif // LIBRARIES_MISCDRIVERS_CAMERA_PAG7920_REGS_H_
