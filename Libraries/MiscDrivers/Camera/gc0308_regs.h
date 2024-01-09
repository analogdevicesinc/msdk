/******************************************************************************
 *
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_GC0308_REGS_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_GC0308_REGS_H_

#define GC0308_SENSOR_WIDTH 640
#define GC0308_SENSOR_HEIGHT 480

#define VGA_WIDTH 640
#define VGA_HEIGHT 480

#define QVGA_WIDTH 320
#define QVGA_HEIGHT 240

#define CIF_WIDTH 352
#define CIF_HEIGHT 288

#define QCIF_WIDTH 176
#define QCIF_HEIGHT 144

#define GC0308_I2C_SLAVE_ADDR 0x21

#define CHIP_ID (0x00)
#define HORIZONTAL_BLANKING (0x01)
#define VERTICAL_BLANKING (0x02)
#define EXPOSURE_MSB (0x03) // Exposure time as number of line processing time
#define EXPOSURE_LSB (0x04)
#define ROW_START_MSB (0x05) // Starting row of the pixel array
#define ROW_START_LSB (0x06)
#define COLUMN_START_MSB (0x07) // Starting column of the pixel array
#define COLUMN_START_LSB (0x08)
#define WINDOW_HEIGHT_MSB (0x09) // Image height in pixels
#define WINDOW_HEIGHT_LSB (0x0a)
#define WINDOW_WIDTH_MSB (0x0b) // Image width in pixels
#define WINDOW_WIDTH_LSB (0x0c)
#define VS_ST (0x0d) // Start to first valid HSync time in row-time units
#define VS_ET (0x0e) // Last HSync to frame-end time in row-time units
#define VB_HB (0x0f) // Vb MSB4 and HB MSB4
#define RSH_WIDTH (0x10) // RESTG width (x2), SH width (x2)
#define TSP_WIDTH (0x11) // TX width, space width (x2)
#define SH_DELAY (0x12)

#define AAAA_ENABLE (0x22) // Auto exposure, auto white balance,
#define SPECIAL_EFFECT (0x23) // Edge map, CbCr fixed enable, inverse color
#define OUTPUT_FORMAT (0x24) // ISP chroma and output data format modes
#define OUTPUT_EN (0x25)
#define SYNC_MODE (0x26)
#define CLK_DIV_MODE (0x28)
#define BYPASS_MODE (0x29)
#define CLOCK_GATING_EN (0x2a)

#define CROP_WIN_MODE (0x46)
#define CROP_WIN_Y1 (0x47)
#define CROP_WIN_X1 (0x48)
#define CROP_WIN_HEIGHT_MSB (0x49)
#define CROP_WIN_HEIGHT_LSB (0x4a)
#define CROP_WIN_WIDTH_MSB (0x4b)
#define CROP_WIN_WIDTH_LSB (0x4c)

#define AWB_R_GAIN (0x5a)
#define AWB_G_GAIN (0x5b)
#define AWB_B_GAIN (0x5c)

#define LSC_DECREASE_LEVEL1 (0x5d)
#define LSC_DECREASE_LEVEL2 (0x5e)
#define LSC_DECREASE_LEVEL3 (0x5f)

#define AEC_MODE1 (0xd0)
#define AEC_MODE2 (0xd1)
#define AEC_MODE3 (0xd2)
#define AEC_TARGET_Y (0xd3)
#define AEC_Y_AVERAGE (0xd4)
#define AEC_HIGH_LOW_RANGE (0xd5)
#define AEC_IGNORE (0xd6)
#define AEC_SKIN_OFFSET_R_OFFS (0xd9)
#define AEC_SKIN_G_OFFS_B_OFFS (0xda)

#define BANKSEL_RESET (0xfe)

// PAGE1

#define SUBSAMPLE (0x54)
#define SUB_MODE (0x55)
#define SUB_ROW_N1 (0x56)
#define SUB_ROW_N2 (0x57)
#define SUB_COL_N1 (0x58)
#define SUB_COL_N2 (0x59)

#define SW_RESET (0xfe)
#define OUTPUT_EN (0x25)

#define HB_H (0x01) // Horizontal blanking, unit vertical pixel clock
#define VB_H (0x02) // Vertical blanking
#define EXPOSURE_H (0x03)
#define EXPOSURE_L (0x04)
#define ROW_START_H (0x05) // Defines the staring row of the pixel array
#define ROW_START_L (0x06)
#define COLUMN_START_H (0x07) // Defines the starting column of the pixel array
#define COLUMN_START_L (0x08)
#define WINDOW_HEIGHT_H (0x09) // Defines image height, default 488
#define WINDOW_HEIGHT_L (0x0a)
#define WINDOW_WIDTH_H (0x0b) // Defines image width, default 646
#define WINDOW_WIDTH_L (0x0c)

#define SH_DELAY (0x12) // Sanple-hold delay time after row finish

#define OUTPUT_FORMAT (0x24) // Image output format selection
#define OUTPUT_EN (0x25) // Synchronization signal output mode enable control
#define SYNC_MODE (0x26) // Synchronization signal output mode control

#define CLK_DIV_MODE (0x28) // Clock divisor control

#define DEBUG_MODE2 (0x2e) // Debug Mode 2.   Bit1 and Bit0 sets Test Image Type

#endif // LIBRARIES_MISCDRIVERS_CAMERA_GC0308_REGS_H_
