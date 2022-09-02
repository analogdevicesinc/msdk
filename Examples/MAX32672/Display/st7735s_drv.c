/**
 * @file    st7735s_drv.c
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mxc_errors.h"

#include "st7735s_regs.h"
#include "st7735s_drv.h"

static st7735s_cfg_t panel_cfg;

int st7735s_pixel(uint32_t x, uint32_t y, uint32_t z)
{
    uint8_t tx_data[4];

    /* Memory Write */
    tx_data[0] = ST7735S_RAMWR;
    tx_data[1] = (x & 0x3f) << 2;
    tx_data[2] = (y & 0x3f) << 2;
    tx_data[3] = (z & 0x3f) << 2;

    return panel_cfg.sendfn(tx_data, 1, tx_data + 1, 3);
}

int st7735s_write_pixels(uint8_t* data, unsigned int len)
{
    uint8_t cmd = ST7735S_RAMWR;

    return panel_cfg.sendfn(&cmd, 1, data, len);
}

int st7735s_xyloc(uint8_t row, uint8_t col)
{
    uint8_t tx_data[5];
    int ret;

    /* Column Address Set */
    tx_data[0] = ST7735S_CASET;
    tx_data[1] = 0;
    tx_data[2] = 0x02 + col;
    tx_data[3] = 0;
    tx_data[4] = 0x81;

    if ((ret = panel_cfg.sendfn(tx_data, 1, tx_data + 1, 4)) != E_NO_ERROR) {
        return ret;
    }

    /* Row Address Set */
    tx_data[0] = ST7735S_RASET;
    tx_data[1] = 0;
    tx_data[2] = 0x02 + row;
    tx_data[3] = 0;
    tx_data[4] = 0x81;

    return panel_cfg.sendfn(tx_data, 1, tx_data + 1, 4);
}

int st7735s_init(st7735s_cfg_t* cfg)
{
    unsigned int i;
    st7735s_regcfg_t* rc;

    if (cfg == NULL) {
        return -1;
    } else {
        memcpy(&panel_cfg, cfg, sizeof(st7735s_cfg_t));
    }

    /* External hardware reset chip needs some time before releasing the line.  */

    cfg->delayfn(500);

    /* Step through the register configuration */
    i  = cfg->ncfgs;
    rc = cfg->regcfg;

    while (i != 0) {
        if (panel_cfg.sendfn(&rc->cmd, 1, rc->data, rc->len) != E_NO_ERROR) {
            return -1;
        }
        if (rc->delay > 0) {
            cfg->delayfn(rc->delay);
        }
        rc++;
        i--;
    }

    return E_NO_ERROR;
}
