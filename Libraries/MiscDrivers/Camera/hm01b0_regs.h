/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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
