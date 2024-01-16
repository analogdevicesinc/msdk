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
