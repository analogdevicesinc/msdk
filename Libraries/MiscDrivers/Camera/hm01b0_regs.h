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
#ifndef LIBRARIES_MISCDRIVERS_CAMERA_HM01B0_REGS_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_HM01B0_REGS_H_

// clang-format off
#define HM01B0_I2C_SLAVE_ADDR   0x24

#define MODEL_H        (0x0000)
#define MODEL_L        (0x0001)
#define REVISION       (0x0002)
#define SW_RESET       (0x0103)

#define MODE_SELECT         (0x0100)
#define STANDBY_MODE        0x0
#define STREAMING_MODE      0x1
#define STREAMING_2_MODE    0x3
#define STREAMING_3_MODE    0x5

#define IMAGE_ORIENTATION   (0x0101)
#define H_MIRROR            0x1
#define V_FLIP              0x2

#define FRAME_LENGTH_H    (0x0340)  //default 562
#define FRAME_LENGTH_L    (0x0341)
#define LINE_LENGTH_H     (0x0342)  //default 370
#define LINE_LENGTH_L     (0x0343)

#define QVGA_WIN           (0x0310)

#define TEST_PATTERN_MODE  (0x0601)
//clang-format on

#endif // LIBRARIES_MISCDRIVERS_CAMERA_HM01B0_REGS_H_
