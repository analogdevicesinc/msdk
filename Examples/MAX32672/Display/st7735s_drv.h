/**
 * @file    st7735s_drv.h
 * @brief   Sitronix ST7735S LCD controller driver
 *          
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef EXAMPLES_MAX32672_DISPLAY_ST7735S_DRV_H_
#define EXAMPLES_MAX32672_DISPLAY_ST7735S_DRV_H_

typedef struct {
    uint8_t cmd; /* Controller command # */
    uint8_t delay; /* Delay in ms after executing this command/data sequence */
    uint8_t len; /* Length of data */
    uint8_t *data;
} st7735s_regcfg_t;

typedef struct {
    void (*delayfn)(uint32_t ms); /* Platform-specific delay function */
    int (*sendfn)(uint8_t *cmd, unsigned int cmd_len, uint8_t *data,
                  unsigned int data_len); /* Platform-specific transmit function */
    st7735s_regcfg_t *regcfg; /* Panel-specific configuration */
    unsigned int ncfgs; /* Number of elements in the regcfg array */
} st7735s_cfg_t;

int st7735s_pixel(uint32_t x, uint32_t y, uint32_t z);
int st7735s_write_pixels(uint8_t *data, unsigned int len);
int st7735s_xyloc(uint8_t row, uint8_t col);
int st7735s_init(st7735s_cfg_t *cfg);

#endif // EXAMPLES_MAX32672_DISPLAY_ST7735S_DRV_H_
