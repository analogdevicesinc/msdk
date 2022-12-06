/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#include "hm01b0_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"

// clang-format off
#define cambus_write(addr, x)     sccb_write_reg16(g_slv_addr, addr, x)
#define cambus_read(addr, x)      sccb_read_reg16(g_slv_addr, addr, x)

//324x244
static const uint16_t default_regs[][2] = {
    {0x0103, 0x0},
    {0x0100, 0x00},
    {0x1003, 0x08},
    {0x1007, 0x08},
    {0x3044, 0x0A},
    {0x3045, 0x00},
    {0x3047, 0x0A},
    {0x3050, 0xC0},
    {0x3051, 0x42},
    {0x3052, 0x50},
    {0x3053, 0x00},
    {0x3054, 0x03},
    {0x3055, 0xF7},
    {0x3056, 0xF8},
    {0x3057, 0x29},
    {0x3058, 0x1F},
    {0x3059, 0x02}, // 8 bit width
    //{0x3059,0x1E},
    {0x3064, 0x00},
    {0x3065, 0x04},
    {0x1000, 0x43},
    {0x1001, 0x40},
    {0x1002, 0x32},
    {0x0350, 0x7F},
    {0x1006, 0x01},
    {0x1008, 0x00},
    {0x1009, 0xA0},
    {0x100A, 0x60},
    {0x100B, 0x90},
    {0x100C, 0x40},
    {0x3022, 0x01},

    {0x1012, 0x01}, // vsync shift
    //{0x1012,0x00}, // no vsync shift
    //{0x1012,0x02}, // Hsync shift

    {0x2000, 0x07},
    {0x2003, 0x00},
    {0x2004, 0x1C},
    {0x2007, 0x00},
    {0x2008, 0x58},
    {0x200B, 0x00},
    {0x200C, 0x7A},
    {0x200F, 0x00},
    {0x2010, 0xB8},
    {0x2013, 0x00},
    {0x2014, 0x58},
    {0x2017, 0x00},
    {0x2018, 0x9B},
    {0x2100, 0x01},
    {0x2101, 0x5F},
    {0x2102, 0x0A},
    {0x2103, 0x03},
    {0x2104, 0x05},
    {0x2105, 0x02},
    {0x2106, 0x14},
    {0x2107, 0x02},
    {0x2108, 0x03},
    {0x2109, 0x03},
    {0x210A, 0x00},
    {0x210B, 0x80},
    {0x210C, 0x40},
    {0x210D, 0x20},
    {0x210E, 0x03},
    {0x210F, 0x00},
    {0x2110, 0x85},
    {0x2111, 0x00},
    {0x2112, 0xA0},
    {0x2150, 0x03},

    {0x0340, 0x01}, // frame_length = 378
    {0x0341, 0x7A},
    {0x0342, 0x01}, // line_length = 375
    {0x0343, 0x77},

    {0x3010, 0x01}, // bit[0]: enable QVGA

    /*
        {0x0340,0x01}, // frame_length = 240      QVGA
        {0x0341,0x04},
        {0x0342,0x01}, // line_length = 320
        {0x0343,0x78},
    */
    /*
        {0x0340,0x00}, // frame_length = 120      QQVGA
        {0x0341,0x80},
        {0x0342,0x00}, // line_length = 160
        {0x0343,0xd7},
    */
#if 0
    {0x0383, 0x01}, // Read out for x[1:0] full
    {0x0387, 0x01}, // Read out for y[1:0] full
    {0x0390, 0x00}, // no binning
#else
    {0x0383, 0x03}, // Horizontal BIN2 timing enable
    {0x0387, 0x03}, // Vertical BIN2 timing enable
    {0x0390, 0x03}, // Vertical/Horizontal binning
#endif

    {0x3011, 0x70}, // 8 bit mode
    {0x3060, 0x0B}, //vt_sys_div=1, vt_reg_div=1
    //{0x3060,0x00},  // vt_sys_div=8, vt_reg_div=4
    //{0x3060,0x0A},  // vt_sys_div=2, vt_reg_div=1 default
    //{0x3060,0x04}, // vt_sys_div=8, vt_reg_div=8
    //{0x3060,0x1A}, // vt_sys_div=2, vt_reg_div=1
    {0x3067, 0x01},  // MCLK mode

    {0x0101, 0x01},  // Hmiror
    //{0x0101,0x03}, // Hmiror, Vflip

    {0x3022, 0x00}, // VSYNC advance
    {0x3023, 0x00}, // HSYNC advance

    //{0x0601,0x01},  // test pattern: color bar
    //{0x0601,0x11},  // test pattern: walking 1

    {0x0104, 0x01}, // group parameter hold
    {0x0100, 0x01}, // start streaming mode
    {0xFFFF, 0xFF},
};
//clang-format on

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_BAYER;

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;
#if 1
    g_slv_addr = sccb_scan();

    if (g_slv_addr == -1) {
        return -1;
    }

#else
    g_slv_addr = HM01B0_I2C_SLAVE_ADDR;
#endif
    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int* id)
{
    int ret = 0;
    uint8_t rev;

    ret |= cambus_read(REVISION, &rev);
    *id = (int)rev;
    return ret;
}

static int get_manufacture_id(int* id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;

    ret |= cambus_read(MODEL_H, &id_high);
    ret |= cambus_read(MODEL_L, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int dump_registers(void)
{
    int ret = 0;
    uint8_t byt = 0;
    uint32_t i;
    uint8_t buf[64] = {0};
    uint8_t* ptr = buf;

<<<<<<< HEAD
    for (i = 0;; i++) {
=======
    for (i = 0; ; i++) {
>>>>>>> parent of 6f9145bfb... clang-format bot reformatting.
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }

        if (i == 0x3401) {
            break;
        }

        ret = cambus_read(i, &byt);

        if (ret == 0) {
            ret = snprintf((char*)ptr, sizeof(buf), " %02X", byt);
            // ^ TODO(all): Improve this, track free space in array

            if (ret < 0) {
                return ret;
            }

            ptr += 3;// XX + space
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

    ret |= cambus_write(SW_RESET, 0);
    MXC_Delay(10000);
    ret |= cambus_write(SW_RESET, 0xFF);

#if 1

    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xFFFF); i++) {
        ret |= cambus_write(default_regs[i][0], (uint8_t)default_regs[i][1]);
        //printf("reg: 0x%04x , val: 0x%02x\r\n",default_regs[i][0], default_regs[i][1]);
    }

#endif

    return ret;
}

static int sleep(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_read(MODE_SELECT, &reg);

    if (ret == 0) {
        if (enable) {
            reg = STANDBY_MODE;
        } else {
            reg = STREAMING_MODE;
        }

        // Write back register
        ret |= cambus_write(MODE_SELECT, reg);
    }

    return ret;
}

static int read_reg(uint16_t reg_addr, uint8_t* reg_data)
{
    *reg_data = 0xff;

    if (cambus_read(reg_addr, reg_data) != 0) {
        return -1;
    }

    return 0;
}

static int write_reg(uint16_t reg_addr, uint8_t reg_data)
{
    return cambus_write(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret = 0;

    g_pixelformat = pixformat;

    return ret;
}

static int get_pixformat(pixformat_t* pixformat)
{
    int ret = 0;
    *pixformat = g_pixelformat;
    return ret;
}

static int set_framesize(int width, int height)
{
    int ret = 0;
#if 0
    // Image typically outputs one line short, add a line to account.
    //height = height + 1;
    // Apply passed in resolution as output resolution.
    ret |= cambus_write(FRAME_LENGTH_H, (width >> 8) & 0xff);
    ret |= cambus_write(FRAME_LENGTH_L, (width >> 0) & 0xff);
    ret |= cambus_write(LINE_LENGTH_H, (height >> 8) & 0xff);
    ret |= cambus_write(LINE_LENGTH_L, (height >> 0) & 0xff);
#endif
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

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;

    if (enable) {
        ret |= cambus_write(TEST_PATTERN_MODE, 0x1);
    } else {
        ret |= cambus_write(TEST_PATTERN_MODE, 0x0);
    }

    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_read(IMAGE_ORIENTATION, &reg);

    if (enable) {
        reg |= H_MIRROR;
    } else {
        reg &= ~H_MIRROR;
    }

    ret |= cambus_write(IMAGE_ORIENTATION, reg);

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
    uint8_t reg;

    ret = cambus_read(IMAGE_ORIENTATION, &reg);

    if (enable) {
        reg |= V_FLIP;
    } else {
        reg &= ~V_FLIP;
    }

    ret |= cambus_write(IMAGE_ORIENTATION, reg);

    return ret;
}

static  int get_luminance(int* lum)
{
    int ret = 0;

    *lum = 0xFFFFFF;
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
