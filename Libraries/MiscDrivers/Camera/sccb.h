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

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_SCCB_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_SCCB_H_

#include <stdint.h>
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

int sccb_init(void);
int sccb_vssel(mxc_gpio_vssel_t vssel);
int sccb_scan(void);
int sccb_read_byt(uint8_t slv_addr, uint8_t reg, uint8_t *byt);
int sccb_write_byt(uint8_t slv_addr, uint8_t reg, uint8_t byt);
int sccb_read_reg16(uint8_t slv_addr, uint16_t reg, uint8_t *byte);
int sccb_write_reg16(uint8_t slv_addr, uint16_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_CAMERA_SCCB_H_
