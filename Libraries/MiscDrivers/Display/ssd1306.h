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

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_SSD1306_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_SSD1306_H_

#include <stdio.h>
#include <stdint.h>
#include "display.h"

/********************************* DEFINES ******************************/
#define BIT_SET(a, b) ((a) |= (1U << (b)))
#define BIT_CLEAR(a, b) ((a) &= ~(1U << (b)))

#define SSD1306_SET_BUFFER_PIXEL_UTIL(buf, buf_w, buf_max, x, y, color, opa) \
    uint16_t byte_index = x + ((y >> 3) * buf_w);                            \
    uint8_t bit_index = y & 0x7;                                             \
    if (byte_index >= buf_max) {                                             \
        return;                                                              \
    }                                                                        \
                                                                             \
    if (color == 0 && opa) {                                                 \
        BIT_SET(buf[byte_index], bit_index);                                 \
    } else {                                                                 \
        BIT_CLEAR(buf[byte_index], bit_index);                               \
    }

/********************************* TYPE DEFINES ******************************/

typedef struct {
    uint8_t row; // Datasheet Page 38.
    uint8_t col; // Datasheet Page 38.
    display_rotation_t rotation;
    uint8_t color_invert;
} ssd1306_init_param_t;

typedef struct {
    display_comm_api comm_api;
    ssd1306_init_param_t init_param;
} ssd1306_dev;

/********************************* Function Prototypes **************************/
int ssd1306_configure(ssd1306_dev *dev, ssd1306_init_param_t *init_param,
                      display_comm_api *comm_api);
int ssd1306_init(ssd1306_dev *dev);
void ssd1306_set_cursor(ssd1306_dev *dev, uint8_t row, uint16_t col);
void ssd1306_send_pixels(ssd1306_dev *dev, uint8_t *payload, uint32_t payloadlen);
void ssd1306_flush_area(ssd1306_dev *dev, const display_area_t *area, const uint8_t *data);

void ssd1306_set_buffer_pixel_util(uint8_t *buf, uint16_t buf_w, uint32_t buf_max, uint16_t x,
                                   uint16_t y, uint8_t color, uint8_t is_opaque);

#endif /* DRIVER_SSD1306_H_ */
