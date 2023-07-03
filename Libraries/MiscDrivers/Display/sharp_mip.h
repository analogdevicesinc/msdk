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

#ifndef DRIVER_SHARP_MIPH_
#define DRIVER_SHARP_MIPH_

#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "display.h"

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

#endif /* DRIVER_SHARP_MIPH_ */
