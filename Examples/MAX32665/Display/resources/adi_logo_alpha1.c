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

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_ADI_LOGO
#define LV_ATTRIBUTE_IMG_ADI_LOGO
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_ADI_LOGO uint8_t
    adi_logo_map[] = {
        0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff,
        0xfc, 0xff, 0xff, 0xff, 0xfc, 0xfc, 0xff, 0xff, 0xfc, 0xfc, 0x3f, 0xff, 0xfc, 0xfc, 0x1f,
        0xff, 0xfc, 0xfc, 0x07, 0xff, 0xfc, 0xfc, 0x01, 0xff, 0xfc, 0xfc, 0x00, 0x7f, 0xfc, 0xfc,
        0x00, 0x3f, 0xfc, 0xfc, 0x00, 0x0f, 0xfc, 0xfc, 0x00, 0x03, 0xfc, 0xfc, 0x00, 0x00, 0xfc,
        0xfc, 0x00, 0x00, 0xfc, 0xfc, 0x00, 0x01, 0xfc, 0xfc, 0x00, 0x07, 0xfc, 0xfc, 0x00, 0x1f,
        0xfc, 0xfc, 0x00, 0x7f, 0xfc, 0xfc, 0x00, 0xff, 0xfc, 0xfc, 0x03, 0xff, 0xfc, 0xfc, 0x0f,
        0xff, 0xfc, 0xfc, 0x1f, 0xff, 0xfc, 0xfc, 0x7f, 0xff, 0xfc, 0xfd, 0xff, 0xff, 0xfc, 0xff,
        0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc,
    };

const lv_img_dsc_t adi_logo = {
    .header.cf = LV_IMG_CF_ALPHA_1BIT,
    .header.always_zero = 0,
    .header.reserved = 0,
    .header.w = 30,
    .header.h = 30,
    .data_size = 120,
    .data = adi_logo_map,
};
