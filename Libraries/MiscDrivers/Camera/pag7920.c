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
#include "pag7920_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"

// PixArt PAG7920LT camera sensor, monochrome, 320x240 resolution

// clang-format off
#define cambus_writeb(addr, x)      sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x)       sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_RGB565;

//320x240 6MHz VDDIO=1.8V 5FPS
// Frame Rate = PXCLK_freq / (2*R_Frame_Time)
static const uint8_t default_regs[][2] = {
    {0xEF, 0x00},
    {0x1D, 0xC1},
    {0x64, 0x03},
    {0x45, 0x44},
    {0x12, 0x20},
    {0x0C, 0xC8},
    {0x11, 0xC8},
    {0x42, 0xC8},
    {0x43, 0xC8},
    {0x44, 0xC8},
    {0xAF, 0x21}, // VSYNC active low, HSYNC active high
    {0x69, 0x14},
    {0xEF, 0x00},
    //{0x4C, 0xE0}, //{R_Frame_Time[7:0]}; for 10fps (R_Frame_Time = 300000)
    //{0x4D, 0x93}, //{R_Frame_Time[15:8]};
    //{0x4E, 0x04}, //{R_Frame_Time[23:16]};
    {0x4C, 0xC0}, //{R_Frame_Time[7:0]}; for 5fps (R_Frame_Time = 600000)
    {0x4D, 0x27}, //{R_Frame_Time[15:8]};
    {0x4E, 0x09}, //{R_Frame_Time[23:16]};
    {0x4F, 0x00}, //{R_Frame_Time[27:24]};
    {0xEF, 0x02},
    {0x02, 0x64},
    {0x03, 0x08},
    {0xEF, 0x04},
    {0x2C, 0x71},
    {0x2D, 0x0B},
    {0x2E, 0x00},
    {0x30, 0x31},
    {0x31, 0x80},
    {0x40, 0x1D},
    {0x41, 0x00},
    {0x42, 0xD0},
    {0x43, 0x01},
    {0x44, 0xF0},
    {0x45, 0x00},
    {0x46, 0x00},
    {0x47, 0x00},
    //{0x48, 0xD0}, //{R_AE_MaxExpo[7:0]}; 290000 Must < R_Frame_Time for 10fps
    //{0x49, 0x6C}, //{R_AE_MaxExpo[15:8]};
    //{0x4A, 0x04}, //{R_AE_MaxExpo[23:16]};
    {0x48, 0xB0}, //{R_AE_MaxExpo[7:0]}; 590000 Must < R_Frame_Time for 5fps
    {0x49, 0x00}, //{R_AE_MaxExpo[15:8]};
    {0x4A, 0x09}, //{R_AE_MaxExpo[23:16]};
    {0x4B, 0x00}, //{R_AE_MaxExpo[27:24]};
    {0x51, 0x1D},
    {0x52, 0x00},
    {0x53, 0xC0},
    {0x54, 0xD4},
    {0x55, 0x01},
    {0x56, 0x00},
    {0xEF, 0x01},
    {0xC4, 0x02},
    {0xC6, 0x40},
    {0xC7, 0x01},
    {0xC8, 0xF0},
    {0xC9, 0x00},
    {0xEF, 0x02},
    {0x11, 0x03},
    {0x19, 0xFC},
    {0x1A, 0x00},
    {0x21, 0x40},
    {0x22, 0x01},
    {0x23, 0xF0},
    {0x24, 0x00},
    {0x27, 0x0C},
    {0x28, 0x00},
    {0x2D, 0xF0},
    {0x2E, 0x00},
    {0x56, 0x40},
    {0x57, 0x01},
    {0x58, 0xFE},
    {0x59, 0x00},
    {0xEF, 0x01},
    {0x33, 0x12},
    {0x3B, 0x0C},
    {0x40, 0x2D},
    {0xD9, 0x0C},
    {0xDB, 0x19},
    {0xDD, 0x19},
    {0xEF, 0x02},
    {0xA5, 0x00},
    {0xA6, 0x2D},
    {0xEF, 0x00},
    {0x09, 0x10},
    {0x18, 0x01},
    {0x2F, 0x44},
    {0x37, 0x04},
    {0x38, 0x06},
    {0x3F, 0x01},
    {0x55, 0x01},
    {0x66, 0x01},
    {0xEF, 0x01},
    {0x03, 0x00},
    {0x04, 0xAB},
    {0x07, 0x02},
    {0x0A, 0x00},
    {0x0B, 0x54},
    {0x0F, 0x1F},
    {0x11, 0x1E},
    {0x13, 0x20},
    {0x16, 0x00},
    {0x17, 0xA9},
    {0x36, 0x00},
    {0x37, 0x02},
    {0x4D, 0x02},
    {0x56, 0xB8},
    {0x57, 0x01},
    {0x58, 0xB8},
    {0x59, 0x01},
    {0x62, 0x00},
    {0x63, 0x06},
    {0x69, 0x08},
    {0x6A, 0x08},
    {0x6B, 0x1B},
    {0x6C, 0x1B},
    {0x76, 0x06},
    {0x77, 0x09},
    {0x78, 0x02},
    {0x79, 0x03},
    {0x84, 0x2D},
    {0x86, 0x28},
    {0x87, 0x2D},
    {0x89, 0x28},
    {0x8A, 0x4B},
    {0x8C, 0x46},
    {0x8D, 0x4B},
    {0x8F, 0x46},
    {0x9D, 0x1E},
    {0xA0, 0x00},
    {0xD1, 0xB9},
    {0xD2, 0x00},
    {0xEF, 0x02},
    {0x92, 0x11},
    {0x93, 0x01},
    {0xC3, 0xB9},
    {0xC4, 0x00},
    {0xC5, 0xB9},
    {0xC6, 0x00},
    {0xD1, 0x45},
    {0xEF, 0x00},
    {0x30, 0x01},
    {0xEB, 0x80},
    {0xEE, 0xEE}  // End of register list marker 0xEE
};
// clang-format on

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;

    // Set I2C address based on GPIO2 pin configuration
    g_slv_addr = PAG7920_I2C_ADDR_GPIO2_LOW;

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

    // Set register bank 0
    ret |= cambus_writeb(REG_BANK, 0);
    // Read product ID
    ret |= cambus_readb(REG_PIDH, &id_high);
    ret |= cambus_readb(REG_PIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int get_manufacture_id(int *id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;

    // Set register bank 0
    ret |= cambus_writeb(REG_BANK, 0);
    // Read manufacture ID
    ret |= cambus_readb(REG_MIDH, &id_high);
    ret |= cambus_readb(REG_MIDL, &id_low);
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

    // Set register bank 0
    ret |= cambus_writeb(REG_BANK, 0x00);
    // Trigger SW reset
    ret |= cambus_writeb(REG_RESET, 0xFF);
    MXC_Delay(100000);

    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xEE); i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }

    return ret;
}

static int sleep(int enable)
{
    int ret = 0;

    if (enable) {
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        // Enable suspend state
        ret |= cambus_writeb(REG_MODE, 0x00);
        // Update flag
        ret |= cambus_writeb(REG_UPDATE, 0x80);
    } else {
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        // Enable continuous  state
        ret |= cambus_writeb(REG_MODE, 0x01);
        // Update flag
        ret |= cambus_writeb(REG_UPDATE, 0x80);
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

    switch (pixformat) {
    case PIXFORMAT_YUV422:
    case PIXFORMAT_RGB444:
    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        ret = -1;
        break;
    case PIXFORMAT_GRAYSCALE:
    case PIXFORMAT_BAYER:
        break;
    default:
        ret = -1;
        break;
    }

    g_pixelformat = pixformat;
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

    if (width > MAX_H_SIZE || height > MAX_V_SIZE) {
        ret = -1;
    }

    // Set register bank 2
    ret |= cambus_writeb(REG_BANK, 0x02);
    // Set horizontal and vertical sizes
    ret |= cambus_writeb(REG_RWOI_HSIZE_L, width & 0xFF);
    ret |= cambus_writeb(REG_RWOI_HSIZE_H, width >> 8);
    ret |= cambus_writeb(REG_RWOI_VSIZE_L, height & 0xFF);
    ret |= cambus_writeb(REG_RWOI_VSIZE_H, 0);

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

    if (width > MAX_H_SIZE || height > MAX_V_SIZE) {
        ret = -1;
    }

    if (width < hsize || height < vsize) {
        ret = -1;
    }

    // Set register bank 2
    ret |= cambus_writeb(REG_BANK, 0x02);
    // Apply window of interest
    ret |= cambus_writeb(REG_RWOI_HSIZE_L, hsize & 0xFF);
    ret |= cambus_writeb(REG_RWOI_HSIZE_H, hsize >> 8);
    ret |= cambus_writeb(REG_RWOI_VSIZE_L, vsize & 0xFF);
    ret |= cambus_writeb(REG_RWOI_VSIZE_H, 0);
    // Adjust center position
    ret |= cambus_writeb(REG_RWOI_HSTART_L, ((width - hsize) >> 1) & 0xFF);
    ret |= cambus_writeb(REG_RWOI_HSTART_H, 0);
    ret |= cambus_writeb(REG_RWOI_VSTART_L, ((height - vsize) >> 1) & 0xFF);
    ret |= cambus_writeb(REG_RWOI_VSTART_H, 0);
    // Enable window of interest
    ret |= cambus_writeb(REG_RWOI_EN, 0x01);
    // Update flag
    ret |= cambus_writeb(REG_UPDATE, 0x80);

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;

    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;

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

    // Set register bank 4
    ret |= cambus_writeb(REG_BANK, 0x04);

    // Set maximum AE gain
    switch (gainceiling) {
    case GAINCEILING_2X:
        ret |= cambus_writeb(REG_AE_MAX_GAIN_L, 0x3A); // 2*29=58, R_AE_MAX_GAIN_x1 = 29
        ret |= cambus_writeb(REG_AE_MAX_GAIN_H, 0x00);
        break;
    case GAINCEILING_4X:
        ret |= cambus_writeb(REG_AE_MAX_GAIN_L, 0x74); // 4*29=116
        ret |= cambus_writeb(REG_AE_MAX_GAIN_H, 0x00);
        break;
    case GAINCEILING_8X:
        ret |= cambus_writeb(REG_AE_MAX_GAIN_L, 0xE8); // 8*29=132
        ret |= cambus_writeb(REG_AE_MAX_GAIN_H, 0x00);
        break;
    case GAINCEILING_16X:
        ret |= cambus_writeb(REG_AE_MAX_GAIN_L, 0xD0); // 16*29=464
        ret |= cambus_writeb(REG_AE_MAX_GAIN_H, 0x01);
        break;
    case GAINCEILING_32X:
        ret |= cambus_writeb(REG_AE_MAX_GAIN_L, 0xA0); // 32*29=928
        ret |= cambus_writeb(REG_AE_MAX_GAIN_H, 0x03);
    case GAINCEILING_64X:
    case GAINCEILING_128X:
    default:
        return -1;
    }

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;

    if (enable) {
        // Set register bank 4
        ret |= cambus_writeb(REG_BANK, 0x04);
        // Set test pattern "Ramp"
        ret |= cambus_writeb(0x00, 0x01);
        ret |= cambus_writeb(0x0A, 0x04);
    }

    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;

    if (enable) {
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        ret |= cambus_writeb(0x09, 0x02);
        // Set register bank 2
        ret |= cambus_writeb(REG_BANK, 0x02);
        ret |= cambus_writeb(0xD9, 0x02);
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        // Update flag
        ret |= cambus_writeb(REG_UPDATE, 0x80);
        // Enable continuous  state
        ret |= cambus_writeb(REG_MODE, 0x01);
    } else {
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        ret |= cambus_writeb(0x09, 0x00);
        // Set register bank 1
        ret |= cambus_writeb(REG_BANK, 0x01);
        ret |= cambus_writeb(0xC2, 0x00);
        ret |= cambus_writeb(0xCB, 0x04);
        ret |= cambus_writeb(0xCE, 0x00);
        // Set register bank 2
        ret |= cambus_writeb(REG_BANK, 0x02);
        // Set register bank 0
        ret |= cambus_writeb(0xD9, 0x00);
        ret |= cambus_writeb(REG_BANK, 0x00);
        // Update flag
        ret |= cambus_writeb(REG_UPDATE, 0x80);
        // Enable continuous  state
        ret |= cambus_writeb(REG_MODE, 0x01);
    }

    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;

    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;

    if (enable) {
        // Set register bank 1
        ret |= cambus_writeb(REG_BANK, 0x01);
        // Enable vertical flip
        ret |= cambus_writeb(0xC2, 0xF7);
        ret |= cambus_writeb(0xCB, 0xF3);
        ret |= cambus_writeb(0xCE, 0x04);
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        // Update flag
        ret |= cambus_writeb(REG_UPDATE, 0x80);
        // Enable continuous  state
        ret |= cambus_writeb(REG_MODE, 0x01);
    } else {
        // Set register bank 0
        ret |= cambus_writeb(REG_BANK, 0x00);
        ret |= cambus_writeb(0x09, 0x00);
        // Set register bank 1
        ret |= cambus_writeb(REG_BANK, 0x01);
        ret |= cambus_writeb(0xC2, 0x00);
        ret |= cambus_writeb(0xCB, 0x04);
        ret |= cambus_writeb(0xCE, 0x00);
        // Set register bank 2
        ret |= cambus_writeb(REG_BANK, 0x02);
        ret |= cambus_writeb(0xD9, 0x00);
        ret |= cambus_writeb(REG_BANK, 0x00);
        // Update flag
        ret |= cambus_writeb(REG_UPDATE, 0x80);
        // Enable continuous  state
        ret |= cambus_writeb(REG_MODE, 0x01);
    }

    return ret;
}

static int get_luminance(int *lum)
{
    int ret = 0;

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
