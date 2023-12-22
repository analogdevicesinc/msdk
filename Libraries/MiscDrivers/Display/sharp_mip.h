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

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_SHARP_MIP_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_SHARP_MIP_H_

#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "display.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************* DEFINES ******************************/
#define BUF_SIZE(w, h) (((h) * (2 + (w >> 3))) + 2)

/********************************* TYPE DEFINES ******************************/
typedef struct {
    uint8_t row;
    uint8_t col;
    mxc_gpio_regs_t *on_off_port;
    uint32_t on_off_pin;
} sharp_mip_init_param_t;

typedef struct {
    display_comm_api comm_api;
    sharp_mip_init_param_t init_param;
} sharp_mip_dev;

/********************************* Function Prototypes **************************/
int sharp_mip_configure(sharp_mip_dev *dev, sharp_mip_init_param_t *init_param,
                        display_comm_api *comm_api);
int sharp_mip_init(sharp_mip_dev *dev);
void sharp_mip_onoff(sharp_mip_dev *dev, int on);
void sharp_mip_flush_area(sharp_mip_dev *dev, const display_area_t *area, const uint8_t *data);
void sharp_mip_set_buffer_pixel_util(sharp_mip_dev *dev, uint8_t *buf, uint16_t buf_w, uint16_t x,
                                     uint16_t y, uint8_t color, uint8_t is_opaque);
void sharp_mip_com_inversion(sharp_mip_dev *dev, int inversion_on);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_DISPLAY_SHARP_MIP_H_
