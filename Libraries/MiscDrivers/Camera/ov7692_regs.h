/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#ifndef LIBRARIES_MISCDRIVERS_CAMERA_OV7692_REGS_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_OV7692_REGS_H_

// clang-format off
// Register address definitions.
#define REG0C       (0x0C)
#define REG12       (0x12)
#define REG0E       (0x0E)
#define PIDH        (0x0a)
#define PIDL        (0x0b)

#define MIDH        (0x1c)
#define MIDL        (0x1d)

#define HSTART      (0x17)
#define HSIZE       (0x18)
#define VSTART      (0x19)
#define VSIZE       (0x1A)

#define OH_HIGH     (0xcc)
#define OH_LOW      (0xcd)
#define OV_HIGH     (0xce)
#define OV_LOW      (0xcf)

#define XSC_MAN_HIGH    (0xc4)
#define XSC_MAN_LOW     (0xc5)
#define YSC_MAN_HIGH    (0xc6)
#define YSC_MAN_LOW     (0xc7)

#define IVERTICAL_LOW   (0xcb)

#define REG14   (0x14)
#define REG81   (0x81)
#define REG82   (0x82)
#define REG28   (0x28)
#define REGD2   (0xd2)
#define REGDA   (0xda)
#define REGDB   (0xdb)

#define REG_LUM2   (0x6d)
#define REG_LUM1   (0x6e)
#define REG_LUM0   (0x6f)

// Register value definitions.
#define REG12_RESET (0x80)

#define COLOR_RGB565    (0x06)
#define COLOR_RGB444    (0x0e)
#define COLOR_YUV422    (0x00)
#define COLOR_BAYER     (0x01)

#define VERTICAL_FLIP       (0X80)
#define HORIZONTAL_FLIP     (0X40)

#define SLEEP_MODE_ENABLE   (0x08)
// clang-format on

#endif // LIBRARIES_MISCDRIVERS_CAMERA_OV7692_REGS_H_
