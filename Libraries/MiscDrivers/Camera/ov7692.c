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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "camera.h"
#include "sccb.h"
#include "ov7692_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"

// clang-format off
#define cambus_writeb(addr, x)      sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x)       sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_RGB565;

static const uint8_t default_regs[][2] = {
    {0x12, 0x80}, // System reset
    {0x69, 0x52}, // BLC window selection, BLC enable (default: 0x12)
    {0x1e, 0xb3}, // AddLT1F (default: 0xb1)
    {0x48, 0x42}, // Reserved
    {0xff, 0x01}, // Select MIPI register bank
    {0xb5, 0x30}, // Power down MIPI and low power transmitter (default: 0x70)
    {0xff, 0x00}, // Select system control register bank
    {0x16, 0x03}, // (default)
    {0x0c, 0xd6}, // Vertival flip, Horizontal mirror, YU/YV swap (default: 0x16)
    {0x82, 0x03}, // YUV422 (default is 0x0)
    {0x11, 0x00}, // CLKRC, Internal clock pre-scalar divide by 1 (default: 0x0)
    {0x12, 0x06}, // RGB565 output format (default: 0x0)
    {0xc3, 0x80}, // (default)
    {0x81, 0x3f}, // sde_en, uv_adj_en, scale_v_en, scale_h_en, uv_avg_en, cmx_en (default: 0x41)
    {0x16, 0x07}, // Enable slowed PCLK for YUV size less than QVGA (default: 0x03)
    {0x31, 0x82}, // System clock divider=2, PLL loop divider=2 (default: 0x83)
    {0x37, 0x00}, // PCLK is the same as system clock, no divider (default: 0x0c)
    {0x3e, 0x20}, // (default)
    {0x5e, 0x10}, // Divided PCLK (default: 0x00)
    {0x64, 0x11}, // PCLK is same as SCLK (default: 0x11)
    {0x69, 0x02}, // BLC window selection=0, BLC enabled (default: 0x12)
    {0xc4, 0x01}, // xsc_man[10:8] (default: 0x02)
    {0xc5, 0x80}, // xsc_man[7:0] (default: 0x00)
    {0xc6, 0x01}, // ysc_man[10:8] (default: 0x02)
    {0xc7, 0x80}, // ysc_man[7:0] (default: 0x00)
    {0xc8, 0x02}, // High 2 bits of horizontal input size (default: 0x02)
    {0xc9, 0x80}, // Low 8 bits of horizontal input size (default: 0x80)
    {0xca, 0x01}, // Ninth bit of vertical input size (default: 0x01)
    {0xcb, 0xe0}, // Low 8 bits of vertical input size (default: 0xe0)
    {0xcc, 0x00}, // High 2 bits of horizontal output size (default: 0x02)
    {0xcd, 0x40}, // Low 8 bits of horizontal output size=64 line width (default: 0x80)
    {0xce, 0x00}, // Ninth bit of vertical output size (default: 0x01)
    {0xcf, 0x40}, // Low 8 bits of vertical output size=64 lines high (default: 0x80)
    {0x13, 0xef}, // Fast AGC/AEC, unlimited step size, banding filter ON,
    // Tp level exposure ON, Auto AEC/AGC/AWB (default: 0xe5)
    {0x14, 0x30}, // Automatic gain ceiling = 16x, Auto 50/60 selection (default: 0x30)
    {0x70, 0x10}, // Low light limit enable (default: 0x0)
    {0x71, 0x00}, // (default: 0x0)
    {0x72, 0x0a}, // Low light threshold, (default: 0x0)
    {0x73, 0x02}, // Low light threshold, counter (default: 0x0)
    {0x74, 0x28}, // Threshold for low sum value (default: 0x20)
    {0x75, 0x98}, // Threshold for high sum value (default: 0x70)
    {0x76, 0x00}, // Low threshold of light meter [15:8] (default: 0x0)
    {0x77, 0x64}, // Low threshold of light meter [7:0] (default: 0x0)
    {0x78, 0x01}, // High threshold of light meter [15:8] (default: 0x01)
    {0x79, 0xc2}, // High threshold of light meter [7:0] (default: 0x2c)
    {0x0e, 0x00}, // Return to Normal mode
    {0xee, 0xee}  // End of register list marker 0xee
};
// clang-format on

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;

    g_slv_addr = 0x3c;

    if (g_slv_addr == -1) {
        return -1;
    }

    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int *id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;

    ret |= cambus_readb(PIDH, &id_high);
    ret |= cambus_readb(PIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int get_manufacture_id(int *id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;

    ret |= cambus_readb(MIDH, &id_high);
    ret |= cambus_readb(MIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int dump_registers(void)
{
    int ret = 0;
    uint8_t byt = 0;
    uint32_t i;
    uint8_t buf[64] = { 0 };
    uint8_t *ptr = buf;

    for (i = 0;; i++) {
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }

        if (i == 256) {
            break;
        }

        ret = cambus_readb(i, &byt);

        if (ret == 0) {
            ret = snprintf((char *)ptr, sizeof(buf), " %02X", byt);
            // ^ TODO(all): Improve this, track free space in array

            if (ret < 0) {
                return ret;
            }

            ptr += 3; // XX + space
        } else {
            *ptr++ = '!';
            *ptr++ = '!';
            *ptr++ = ' ';
        }
    }

    return ret;
}

static int reset(void)
{
    int ret = 0;
    uint8_t value;
    ret |= cambus_writeb(REG12, REG12_RESET);

    // Read from the register, when the reset bit is cleared then reset is done.
    for (value = 0xff; value != 0;) {
        ret |= cambus_readb(REG12, &value);
        MXC_Delay(10000);
    }

    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xee); i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }

    return ret;
}

static int sleep(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG0E, &reg);

    if (ret == 0) {
        if (enable) {
            reg |= SLEEP_MODE_ENABLE;
        } else {
            reg &= ~SLEEP_MODE_ENABLE;
        }

        // Write back register
        ret |= cambus_writeb(REG0E, reg);
    }

    return ret;
}

static int read_reg(uint8_t reg_addr, uint8_t *reg_data)
{
    *reg_data = 0xff;

    if (cambus_readb(reg_addr, reg_data) != 0) {
        return -1;
    }

    return 0;
}

static int write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    return cambus_writeb(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret = 0;

    g_pixelformat = pixformat;

    switch (pixformat) {
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        ret |= cambus_writeb(REG12, COLOR_YUV422);
        break;

    case PIXFORMAT_RGB444:
        ret |= cambus_writeb(REG12, COLOR_RGB444);
        break;

    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        ret |= cambus_writeb(REG12, COLOR_RGB565);
        break;

    case PIXFORMAT_BAYER:
        ret |= cambus_writeb(REG12, COLOR_BAYER);
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static int get_pixformat(pixformat_t *pixformat)
{
    int ret = 0;
    *pixformat = g_pixelformat;
    return ret;
}

static int set_framesize(int width, int height)
{
    int ret = 0;
    uint8_t input_factor_4_3[] = { 0x02, 0x80, 0x01, 0xe0 }; // 640 x 480
    uint8_t input_factor_1_1[] = { 0x01, 0xe0, 0x01, 0xe0 }; // 480 x 480
    //    uint8_t input_factor_small[] = { 0x01, 0xbf, 0x01, 0xbf }; // 447 x 447
    uint8_t *input_factor_ptr = input_factor_4_3;

    // Check and see if the target resolution is very small
    // that is less than 42 x 42, if so then apply and
    // x and y scaling factor.
    if ((width == height) || (width < 42) || (height < 42)) {
        input_factor_ptr = input_factor_1_1;
    }

    // Image typically outputs one line short, add a line to account.
    height = height + 1;
    // Apply passed in resolution as output resolution.
    ret |= cambus_writeb(OH_HIGH, (width >> 8) & 0xff);
    ret |= cambus_writeb(OH_LOW, (width >> 0) & 0xff);
    ret |= cambus_writeb(OV_HIGH, (height >> 8) & 0xff);
    ret |= cambus_writeb(OV_LOW, (height >> 0) & 0xff);

    // Apply the appropriate input image factor.
    ret |= cambus_writeb(0xc8, input_factor_ptr[0]);
    ret |= cambus_writeb(0xc9, input_factor_ptr[1]);
    ret |= cambus_writeb(0xca, input_factor_ptr[2]);
    ret |= cambus_writeb(0xcb, input_factor_ptr[3]);
    return ret;
}

static int set_windowing(int width, int height, int hsize, int vsize)
{
    /* Note: width and height is used to control scaling size of the image
       width: horizontal input size
       height: vertical input size
       hsize: horizontal size of cropped image
       vsize: vertical size of cropped image
    */
    int ret = 0;

    if (width < hsize || height < vsize) {
        ret = -1;
    }

    ret |= cambus_writeb(0x11, 0x0);
    ret |= cambus_writeb(0x51, 0x7f);
    ret |= cambus_writeb(0x50, 0x99);
    ret |= cambus_writeb(0x21, 0x23);
    ret |= cambus_writeb(0x20, 0x00);
    // Apply passed in resolution as input resolution.
    ret |= cambus_writeb(0xc8, (width >> 8) & 0xff);
    ret |= cambus_writeb(0xc9, (width >> 0) & 0xff);
    ret |= cambus_writeb(0xca, (height >> 8) & 0xff);
    ret |= cambus_writeb(0xcb, (height >> 0) & 0xff);

    // Apply passed in hsize & vsize as output resolution.
    ret |= cambus_writeb(OH_HIGH, (hsize >> 8) & 0xff);
    ret |= cambus_writeb(OH_LOW, (hsize >> 0) & 0xff);
    ret |= cambus_writeb(OV_HIGH, (vsize >> 8) & 0xff);
    ret |= cambus_writeb(OV_LOW, (vsize >> 0) & 0xff);

    // adjust center position
    ret |= cambus_writeb(0x9, 0x10);
    ret |= cambus_writeb(HSTART, 0x55);
    ret |= cambus_writeb(VSTART, 0xc2);

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;

    switch (level) {
    case -2:
        ret = cambus_writeb(0xd5, 0x20);
        ret = cambus_writeb(0xd4, 0x18);
        ret = cambus_writeb(0xd3, 0x48);
        ret = cambus_writeb(0xd2, 0x04);
        break;
    case -1:
        ret = cambus_writeb(0xd5, 0x20);
        ret = cambus_writeb(0xd4, 0x1c);
        ret = cambus_writeb(0xd3, 0x20);
        ret = cambus_writeb(0xd2, 0x04);
        break;
    case 0: // default
        ret = cambus_writeb(0xd5, 0x20);
        ret = cambus_writeb(0xd4, 0x20);
        ret = cambus_writeb(0xd3, 0x00);
        ret = cambus_writeb(0xd2, 0x04);
        break;
    case 1:
        ret = cambus_writeb(0xd5, 0x20);
        ret = cambus_writeb(0xd4, 0x24);
        ret = cambus_writeb(0xd3, 0x00);
        ret = cambus_writeb(0xd2, 0x04);
        break;
    case 2:
        ret = cambus_writeb(0xd5, 0x20);
        ret = cambus_writeb(0xd4, 0x28);
        ret = cambus_writeb(0xd3, 0x00);
        ret = cambus_writeb(0xd2, 0x04);
        break;
    default:
        return -1;
    }

    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;

    switch (level) {
    case -2:
        ret = cambus_writeb(0x24, 0x70);
        ret = cambus_writeb(0x25, 0x60);
        ret = cambus_writeb(0x26, 0xa2);
        break;
    case -1:
        ret = cambus_writeb(0x24, 0x78);
        ret = cambus_writeb(0x25, 0x70);
        ret = cambus_writeb(0x26, 0xa2);
        break;
    case 0: // default
        ret = cambus_writeb(0x24, 0x86);
        ret = cambus_writeb(0x25, 0x76);
        ret = cambus_writeb(0x26, 0xb3);
        break;
    case 1:
        ret = cambus_writeb(0x24, 0xa0);
        ret = cambus_writeb(0x25, 0x98);
        ret = cambus_writeb(0x26, 0xc4);
        break;
    case 2:
        ret = cambus_writeb(0x24, 0xa8);
        ret = cambus_writeb(0x25, 0xa0);
        ret = cambus_writeb(0x26, 0xc4);
        break;
    default:
        return -1;
    }

    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;

    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG14, &reg);
    reg &= 0x8f; // clear bits [6:4]

    switch (gainceiling) {
    case GAINCEILING_2X:
        reg |= 0 << 4;
        ret |= cambus_writeb(REG14, reg);
        break;
    case GAINCEILING_4X:
        reg |= 1 << 4;
        ret |= cambus_writeb(REG14, reg);
        break;
    case GAINCEILING_8X:
        reg |= 2 << 4;
        ret |= cambus_writeb(REG14, reg);
        break;
    case GAINCEILING_16X: // default
        reg |= 3 << 4;
        ret |= cambus_writeb(REG14, reg);
        break;
    case GAINCEILING_32X:
        reg |= 4 << 4;
        ret |= cambus_writeb(REG14, reg);
    case GAINCEILING_64X:
        reg |= 5 << 4;
        ret |= cambus_writeb(REG14, reg);
    case GAINCEILING_128X:
        reg |= 6 << 4;
        ret |= cambus_writeb(REG14, reg);
        break;
    default:
        return -1;
    }

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG82, &reg);

    if (enable) {
        reg |= 0x0c; // enable constant colorbar
    } else {
        reg &= 0xf3; // disable constant colorbar
    }

    ret |= cambus_writeb(REG82, reg);
    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG0C, &reg);

    if (enable) {
        reg |= HORIZONTAL_FLIP;
    } else {
        reg &= ~HORIZONTAL_FLIP;
    }

    ret |= cambus_writeb(REG0C, reg);
    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;

    if (enable) {
        ret |= cambus_writeb(REG81, 0x3f);
        ret |= cambus_writeb(REG28, 0x82);
        ret |= cambus_writeb(REGD2, 0x00);
        ret |= cambus_writeb(REGDA, 0x80);
        ret |= cambus_writeb(REGDB, 0x80);
    } else {
        ret |= cambus_writeb(REG81, 0x3f);
        ret |= cambus_writeb(REG28, 0x02);
        ret |= cambus_writeb(REGD2, 0x00);
        ret |= cambus_writeb(REGDA, 0x80);
        ret |= cambus_writeb(REGDB, 0x80);
    }

    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG0C, &reg);

    if (enable) {
        reg |= VERTICAL_FLIP;
    } else {
        reg &= ~VERTICAL_FLIP;
    }

    ret |= cambus_writeb(REG0C, reg);
    return ret;
}

static int get_luminance(int *lum)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG_LUM0, &reg);
    *lum = reg;
    ret |= cambus_readb(REG_LUM1, &reg);
    *lum |= ((int)reg) << 8;
    ret |= cambus_readb(REG_LUM2, &reg);
    *lum |= ((int)reg & 0x0F) << 16;

    return ret;
}

// clang-format off
/******************************** Public Functions ***************************/
int sensor_register(camera_t* camera)
{
    // Initialize sensor structure.
    camera->init                = init;
    camera->get_slave_address   = get_slave_address;
    camera->get_product_id      = get_product_id;
    camera->get_manufacture_id  = get_manufacture_id;
    camera->dump_registers      = dump_registers;
    camera->reset               = reset;
    camera->sleep               = sleep;
    camera->read_reg            = read_reg;
    camera->write_reg           = write_reg;
    camera->set_pixformat       = set_pixformat;
    camera->get_pixformat       = get_pixformat;
    camera->set_framesize       = set_framesize;
    camera->set_windowing       = set_windowing;
    camera->set_contrast        = set_contrast;
    camera->set_brightness      = set_brightness;
    camera->set_saturation      = set_saturation;
    camera->set_gainceiling     = set_gainceiling;
    camera->set_colorbar        = set_colorbar;
    camera->set_hmirror         = set_hmirror;
    camera->set_vflip           = set_vflip;
    camera->set_negateimage     = set_negateimage;
    camera->get_luminance       = get_luminance;
    return 0;
}
// clang-format on
