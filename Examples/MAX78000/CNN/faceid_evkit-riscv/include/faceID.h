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

#ifndef _FACEID_H_
#define _FACEID_H_

#define CAMERA_FREQ (10 * 1000 * 1000)
#define LP_MODE_ENABLE
#define LP_MODE 4 // 0:NO SLEEP, 1:SLEEP, 2:LPM, 3:UPM, 4:STANDBY, 5:BACKUP, 6:POWERDOWN
#define LP_TIME 500 //ms
// Mailboxes
#define ARM_MAILBOX_SIZE 1
#define RISCV_MAILBOX_SIZE 16
// Mailbox commands
#define START_FACEID 1
#define STOP_FACEID 2
#define IMAGE_READY 3
#define RESULT_READY 4

#define CAPTURE_X 70
#define CAPTURE_Y 290
#define SKIP_X 60
#define SKIP_Y 290
#define RUN_X 160
#define RUN_Y 290
#define BACK_X 0
#define BACK_Y 280

#define IMAGE_XRES 170 //200
#define IMAGE_YRES 130 //150

#define HEIGHT 160
#define WIDTH 120
#define THICKNESS 4
#define IMAGE_H 130 //150
#define IMAGE_W 170 //200
#define FRAME_COLOR 0x535A

#define X_START 45
#define Y_START 30

#define BYTE_PER_PIXEL 2

#define X_IMAGE_START 45
#define Y_IMAGE_START 30
#define LOW_LIGHT_THRESHOLD 20
#define FAST_FIFO // if defined, it uses fast fifo instead of fifo
// Data input: HWC (little data): 160x120x3
#define DATA_SIZE_IN (160 * 120 * 3)

#endif // _FACEID_H_
