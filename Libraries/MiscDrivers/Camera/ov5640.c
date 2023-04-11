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
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mipi_camera.h"
#include "sccb.h"
#include "ov5640_regs.h"

// clang-format off
#define cambus_write(addr, x)     sccb_write_reg16(g_slv_addr, addr, x)
#define cambus_read(addr, x)      sccb_read_reg16(g_slv_addr, addr, x)
#define cambus_writeb(addr, x)      sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x)       sccb_read_byt(g_slv_addr, addr, x)

static struct camera_reg default_regs[] = {
    // Reset
    {0x3102, 0x11},
    {0x3008, 0x82},
    {0xffff, 150},
    {0x3008, 0x02},
    {0xffff, 150},

    {0x3008, 0x82},
    {0xffff, 150},

    // SCCB Control
    {0x3103, 0x03},

    // SC Pad Out Enable
    {0x3017, 0xe0},
    {0x3018, 0x00},

    // SC PLL Control
    // {0x3034, 0x1a}, // 10-bit RAW mode
    {0x3034, 0x18}, // 8-bit RAW mode
    {0x3008, 0x02},
    {0x3035, 0x14},
    {0x3036, 0x00},
    {0x3037, 0x10},

    // SCCB Pad Clock Divide
    {0x3108, 0x01},
    {0x3630, 0x36},
    {0x3631, 0x0e},
    {0x3632, 0xe2},
    {0x3633, 0x12},
    {0x3621, 0xe0},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3901, 0x0a},
    {0x3731, 0x12},
    {0x3600, 0x08},
    {0x3601, 0x33},
    {0x302d, 0x60},
    {0x3620, 0x52},
    {0x371b, 0x20},
    {0x471c, 0x50},

    // AEC Control
    {0x3a13, 0x43},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3635, 0x13},
    {0x3636, 0x03},
    {0x3634, 0x40},
    {0x3622, 0x01},

    // 5060Hz Control
    {0x3c01, 0x34},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c06, 0x00},
    {0x3c07, 0x00},
    {0x3c08, 0x01},
    {0x3c09, 0x2c},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},
    {0x3820, 0x06},
    {0x3821, 0x00},

    // Active pixel array is 2592x1944

    // TIMING HS, X_ADDR_ST, X address start [11:8]
    {0x3800, 0x01},
    // TIMING HS, X_ADDR_ST, X address start [7:0]
    {0x3801, 0x5a},
    // TIMING VS, Y_ADDR_ST, Y address start [11:8]
    {0x3802, 0x00},
    // TIMING VS, Y_ADDR_ST, Y address start [7:0]
    {0x3803, 0x16},

    // TIMING HW, X_ADDR_END, X address end [11:8]
    {0x3804, 0x08},
    // TIMING HW, X_ADDR_END, X address end [7:0]
    {0x3805, 0xc6},
    // TIMING VH, Y_ADDR_END, Y address end [11:8]
    {0x3806, 0x07},
    // TIMING VH, Y_ADDR_END, Y address end [7:0]
    {0x3807, 0x82},

    // X_OUPUT_SIZE
    {0x3808, 0x00},
    {0x3809, 0x20},

    // Y_OUTPUT_SIZE
    {0x380a, 0x00},
    {0x380b, 0x20},

    // TIMING HTS, Total horizontal size [11:8]
    {0x380c, 0x0f},

    // TIMING HTS, Total horizontal size [7:0]
    {0x380d, 0xff},

    // TIMING VTS, Total vertical size [11:8]
    {0x380e, 0x04},

    // TIMING VTS, Total vertical size [7:0]
    {0x380f, 0xd8},

    {0x3810, 0x00},
    {0x3811, 0x04},
    {0x3812, 0x00},
    {0x3813, 0x04},

    {0x3618, 0x00},
    {0x3612, 0x29},
    {0x3708, 0x64},
    {0x3709, 0x52},
    {0x370c, 0x03},

    // AEC/AGC Power Down Domain Control
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0e, 0x03},
    {0x3a0d, 0x04},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},

    // BLC Control
    {0x4001, 0x02},
    {0x4004, 0x02},

    // System Control
    {0x3000, 0x00},
    {0x3002, 0x1c},
    {0x3004, 0xff},
    {0x3006, 0xc3},
    {0x300e, 0x58},
    {0x302e, 0x00},
    {0x4740, 0x22},

    // Format Control
    {0x4300, 0x61}, // RGB565 ({r[4:0]g[5:3]},{g[2:0],b[4:0]})

    // ISP Top Control
    {0x501f, 0x01},

    // DVP Control
    {0x4713, 0x03},

    // JPEG Control
    {0x4407, 0x04},
    {0x440e, 0x00},

    // VFIFO Control
    {0x460b, 0x35},
    {0x460c, 0x23},

    // MIPI Transmitter Control
    {0x3824, 0x02},

    // ISP Top Control
    {0x5000, 0xa7},
    {0x5001, 0xa3},

    // AWB Control
    {0x5180, 0xff},
    {0x5181, 0xf2},
    {0x5182, 0x00},
    {0x5184, 0x25},
    // 4-3-2023:  Settings below removed.  AWB seems better without them.
    // {0x5183, 0x14},
    // {0x5185, 0x24},
    // {0x5186, 0x09},
    // {0x5187, 0x09},
    // {0x5188, 0x09},
    // {0x5189, 0x88},
    // {0x518a, 0x54},
    // {0x518b, 0xee},
    // {0x518c, 0xb2},
    // {0x518d, 0x50},
    // {0x518e, 0x34},
    // {0x518f, 0x6b},
    // {0x5190, 0x46},
    // {0x5191, 0xf8},
    // {0x5192, 0x04},
    // {0x5193, 0x70},
    // {0x5194, 0xf0},
    // {0x5195, 0xf0},
    // {0x5196, 0x03},
    // {0x5197, 0x01},
    // {0x5198, 0x04},
    // {0x5199, 0x6c},
    // {0x519a, 0x04},
    // {0x519b, 0x00},
    // {0x519c, 0x09},
    // {0x519d, 0x2b},
    // {0x519e, 0x38},

    // CMX Control
    {0x5381, 0x1e},
    {0x5382, 0x5b},
    {0x5383, 0x08},
    {0x5384, 0x0a},
    {0x5385, 0x7e},
    {0x5386, 0x88},
    {0x5387, 0x7c},
    {0x5388, 0x6c},
    {0x5389, 0x10},
    {0x538a, 0x01},
    {0x538b, 0x98},

    // CIP Control
    {0x5300, 0x08},
    {0x5301, 0x30},
    {0x5302, 0x10},
    {0x5303, 0x00},
    {0x5304, 0x08},
    {0x5305, 0x30},
    {0x5306, 0x08},
    {0x5307, 0x16},
    {0x5309, 0x08},
    {0x530a, 0x30},
    {0x530b, 0x04},
    {0x530c, 0x06},

    // Gamma Control
    {0x5480, 0x01},
    {0x5481, 0x08},
    {0x5482, 0x14},
    {0x5483, 0x28},
    {0x5484, 0x51},
    {0x5485, 0x65},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x87},
    {0x5489, 0x91},
    {0x548a, 0x9a},
    {0x548b, 0xaa},
    {0x548c, 0xb8},
    {0x548d, 0xcd},
    {0x548e, 0xdd},
    {0x548f, 0xea},
    {0x5490, 0x1d},

    // SDE Control
    {0x5580, 0x02},
    {0x5583, 0x40},
    {0x5584, 0x10},
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},

    // LENC Control
    // 4-3-2023:  Settings below removed.  They seemed to have no performance
    // impact at all
    // {0x5800, 0x23},
    // {0x5801, 0x14},
    // {0x5802, 0x0f},
    // {0x5803, 0x0f},
    // {0x5804, 0x12},
    // {0x5805, 0x26},
    // {0x5806, 0x0c},
    // {0x5807, 0x08},
    // {0x5808, 0x05},
    // {0x5809, 0x05},
    // {0x580a, 0x08},
    // {0x580b, 0x0d},
    // {0x580c, 0x08},
    // {0x580d, 0x03},
    // {0x580e, 0x00},
    // {0x580f, 0x00},
    // {0x5810, 0x03},
    // {0x5811, 0x09},
    // {0x5812, 0x07},
    // {0x5813, 0x03},
    // {0x5814, 0x00},
    // {0x5815, 0x01},
    // {0x5816, 0x03},
    // {0x5817, 0x08},
    // {0x5818, 0x0d},
    // {0x5819, 0x08},
    // {0x581a, 0x05},
    // {0x581b, 0x06},
    // {0x581c, 0x08},
    // {0x581d, 0x0e},
    // {0x581e, 0x29},
    // {0x581f, 0x17},
    // {0x5820, 0x11},
    // {0x5821, 0x11},
    // {0x5822, 0x15},
    // {0x5823, 0x28},
    // {0x5824, 0x46},
    // {0x5825, 0x26},
    // {0x5826, 0x08},
    // {0x5827, 0x26},
    // {0x5828, 0x64},
    // {0x5829, 0x26},
    // {0x582a, 0x24},
    // {0x582b, 0x22},
    // {0x582c, 0x24},
    // {0x582d, 0x24},
    // {0x582e, 0x06},
    // {0x582f, 0x22},
    // {0x5830, 0x40},
    // {0x5831, 0x42},
    // {0x5832, 0x24},
    // {0x5833, 0x26},
    // {0x5834, 0x24},
    // {0x5835, 0x22},
    // {0x5836, 0x22},
    // {0x5837, 0x26},
    // {0x5838, 0x44},
    // {0x5839, 0x24},
    // {0x583a, 0x26},
    // {0x583b, 0x28},
    // {0x583c, 0x42},
    // {0x583d, 0xce},
    // {0x5025, 0x00},

    // AEC/AGC Power Down Domain Control
    {0x3a0f, 0x30},
    {0x3a10, 0x28},
    {0x3a1b, 0x30},
    {0x3a1e, 0x26},
    {0x3a11, 0x60},
    {0x3a1f, 0x14},

    // 50/60Hz Detector Control
    {0x3c00, 0x04},
    {0xffff, 150},

    // Software power down
    {0x3008, 0x42},
    // Enable all blocks
    {0x3000, 0x00},
    // Enable FIFOs and JPEG
    {0x3002, 0x1c},
    // Enable clocks
    {0x3004, 0xff},
    {0x3006, 0xc3},
    // Pad out enables
    {0x3017, 0xe0},
    {0x3018, 0x00},
    // Autofocus Mode
    {0x3022, 0x0d},
    // MIPI control, 2 lane
    {0x4800, 0x24},
    {0x300e, 0x45},
    {0x302e, 0x08},
    // SCLK divider
    {0x3108, 0x02},
    // PLL multiplier/divider
    {0x303d, 0x20},
    {0x303b, 0x1e},
    {0x501f, 0x01},
    // Format control (RGB Type)
    {0x3630, 0x2e},
    {0x3632, 0xe2},
    {0x3633, 0x23},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3901, 0x0a},
    {0x3731, 0x12},
    // Sensor VFLIP
    {0x3820, 0x42},
    // ISP Control
    {0x5001, 0xa7},
    // Horizontal odd/even increment
    {0x3814, 0x31},
    // Vertical odd/even increment
    {0x3815, 0x31},
    {0x3821, 0x00},
    {0x3618, 0x00},
    {0x3612, 0x29},
    {0x3708, 0x64},
    {0x3709, 0x52},
    {0x370c, 0x03},
    

    {0x3008, 0x02},

    // Delay
    {0xffff, 150},

    // End
    {0, 0}
};
//clang-format on

static int g_slv_addr;
// Default
static mipi_pixformat_t g_pixelformat = MIPI_PIXFORMAT_RGB888;

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;

    g_slv_addr = 0x3C;

    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int* id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;

    ret |= cambus_read(CHIPID_H, &id_high);
    ret |= cambus_read(CHIPID_L, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int get_manufacture_id(int* id)
{
    return E_NOT_SUPPORTED;
}

static int dump_registers(void)
{
    return E_NOT_SUPPORTED;
}

static int reset(void)
{
    int ret = 0;
    uint8_t value;

    ret = cambus_read(SYS_CTRL0, &value);
    value |= 0x82;  // Software reset
    ret |= cambus_write(SYS_CTRL0, value);
    MXC_Delay(10000);
    value &= ~0x80;
    ret |= cambus_write(SYS_CTRL0, value);

    // Write default registers
    for (int i = 0; (default_regs[i].addr != 0); i++) {
        if (default_regs[i].addr == 0xFFFF) {
            MXC_Delay(MSEC(default_regs[i].val));
        } else {
            ret |= cambus_write(default_regs[i].addr, (uint8_t)default_regs[i].val);
            ret |= cambus_read(default_regs[i].addr, &value);
            if (value != default_regs[i].val && default_regs[i].addr != 0x3102 && default_regs[i].addr != 0x3008) { // Skip validation of reset registers
                printf("Mismatch at 0x%x!  Expected 0x%x but read 0x%x\n", default_regs[i].addr, default_regs[i].val, value);
            }
        }
    }

    return ret;
}

static int sleep(int sleep)
{
    int ret = 0;

    ret = cambus_write(OV5640_SYSTEM_CTROL0, sleep ? 0x42 : 0x02);

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

static int set_pixformat(mipi_pixformat_t pixformat, uint32_t out_seq, int mux_ctrl)
{
    int ret = 0;

    g_pixelformat = pixformat;

    ret |= cambus_write(FORMAT_MUX_CTRL, mux_ctrl);

    switch (pixformat) {
    case MIPI_PIXFORMAT_RAW:
        ret |= cambus_write(FORMAT_CTRL00h, (0x00 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_Y8:
        ret |= cambus_write(FORMAT_CTRL00h, (0x01 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_YUV422:
        ret |= cambus_write(FORMAT_CTRL00h, (0x03 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_YUV420:
        ret |= cambus_write(FORMAT_CTRL00h, (0x04 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_YUV420M:
        ret |= cambus_write(FORMAT_CTRL00h, (0x05 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_RGB444_1:
        ret |= cambus_write(FORMAT_CTRL00h, (0x09 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_RGB444_2:
        ret |= cambus_write(FORMAT_CTRL00h, (0x0a << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_RGB555_1:
        ret |= cambus_write(FORMAT_CTRL00h, (0x07 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_RGB555_2:
        ret |= cambus_write(FORMAT_CTRL00h, (0x08 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_RGB565:
        ret |= cambus_write(FORMAT_CTRL00h, (0x06 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_RGB888:
        ret |= cambus_write(FORMAT_CTRL00h, (0x02 << 4) | out_seq);
        break;

    case MIPI_PIXFORMAT_BYPASS:
        ret |= cambus_write(FORMAT_CTRL00h, (0x0f << 4) | out_seq);
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static int get_pixformat(mipi_pixformat_t* pixformat)
{
    int ret = 0;
    *pixformat = g_pixelformat;
    return ret;
}

static int set_framesize(int width, int height)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    // Image typically outputs one line short, add a line to account.
    //height = height + 1;
    // Apply passed in resolution as output resolution.
    ret |= cambus_write(TIMING_DVPHO_0, (width >> 8) & 0xff);
    ret |= cambus_write(TIMING_DVPHO_1, (width >> 0) & 0xff);
    ret |= cambus_write(TIMING_DVPVO_0, (height >> 8) & 0xff);
    ret |= cambus_write(TIMING_DVPVO_1, (height >> 0) & 0xff);

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_windowing(int width, int height, int start_x, int start_y,  int hsize, int vsize)
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

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    // Apply passed in resolution as input resolution.
    ret |= cambus_write(TIMING_HTS_0, (width >> 8) & 0xff);
    ret |= cambus_write(TIMING_HTS_1, (width >> 0) & 0xff);
    ret |= cambus_write(TIMING_VTS_0, (height >> 8) & 0xff);
    ret |= cambus_write(TIMING_VTS_1, (height >> 0) & 0xff);

    // Apply passed in hsize & vsize as output resolution.
    ret |= cambus_write(TIMING_DVPHO_0, (hsize >> 8) & 0xff);
    ret |= cambus_write(TIMING_DVPHO_1, (hsize >> 0) & 0xff);
    ret |= cambus_write(TIMING_DVPVO_0, (vsize >> 8) & 0xff);
    ret |= cambus_write(TIMING_DVPVO_1, (vsize >> 0) & 0xff);

    // set window start position
    ret |= cambus_write(TIMING_HS_0, (start_x >> 8) & 0xff);
    ret |= cambus_write(TIMING_HS_1, (start_x >> 0) & 0xff);
    ret |= cambus_write(TIMING_VS_0, (start_y >> 8) & 0xff);
    ret |= cambus_write(TIMING_VS_1, (start_y >> 0) & 0xff);

    // set window size
    ret |= cambus_write(TIMING_HW_0, (hsize >> 8) & 0xff);
    ret |= cambus_write(TIMING_HW_1, (hsize >> 0) & 0xff);
    ret |= cambus_write(TIMING_VH_0, (vsize >> 8) & 0xff);
    ret |= cambus_write(TIMING_VH_1, (vsize >> 0) & 0xff);

    // Enable ISP vertical and horizontal scale
    ret |= cambus_write(ISP_CTRL01h, 0xa3);

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    switch (level) {
    case -4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x10);
        ret |= cambus_write(0x5588, 0x10);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case -3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x14);
        ret |= cambus_write(0x5588, 0x14);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case -2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x18);
        ret |= cambus_write(0x5588, 0x18);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case -1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x1C);
        ret |= cambus_write(0x5588, 0x1C);
        ret |= cambus_write(0x558a, 0x1C);
        break;

    case  0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x20);
        ret |= cambus_write(0x5588, 0x20);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x24);
        ret |= cambus_write(0x5588, 0x24);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x28);
        ret |= cambus_write(0x5588, 0x28);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x2c);
        ret |= cambus_write(0x5588, 0x2c);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x30);
        ret |= cambus_write(0x5588, 0x30);
        ret |= cambus_write(0x558a, 0x00);
        break;

    default:
        return -1;
    }

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    switch (level) {
    case -4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x40);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x08);
        break;

    case -3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x30);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x08);
        break;

    case -2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x20);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x08);
        break;

    case -1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x10);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x08);
        break;

    case  0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x00);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x10);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x20);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x30);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case  4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x40);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    default:
        return -1;
    }

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    switch (level) {
    case -4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x00);
        ret |= cambus_write(0x5584, 0x00);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case -3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x10);
        ret |= cambus_write(0x5584, 0x10);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case -2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x20);
        ret |= cambus_write(0x5584, 0x20);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case -1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x30);
        ret |= cambus_write(0x5584, 0x30);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case  0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x40);
        ret |= cambus_write(0x5584, 0x40);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case  1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x50);
        ret |= cambus_write(0x5584, 0x50);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case  2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x60);
        ret |= cambus_write(0x5584, 0x60);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case  3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x70);
        ret |= cambus_write(0x5584, 0x70);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case  4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x80);
        ret |= cambus_write(0x5584, 0x80);
        ret |= cambus_write(0x5580, 0x02);
        break;

    default:
        return -1;
    }

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_hue(int degree)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    switch (degree) {
    case -180:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x80);
        ret |= cambus_write(0x5582, 0x00);
        ret |= cambus_write(0x558a, 0x32);
        break;

    case -150:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x6f);
        ret |= cambus_write(0x5582, 0x40);
        ret |= cambus_write(0x558a, 0x32);
        break;

    case -120:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x40);
        ret |= cambus_write(0x5582, 0x6f);
        ret |= cambus_write(0x558a, 0x32);
        break;

    case -90:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x00);
        ret |= cambus_write(0x5582, 0x80);
        ret |= cambus_write(0x558a, 0x02);
        break;

    case -60:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x40);
        ret |= cambus_write(0x5582, 0x6f);
        ret |= cambus_write(0x558a, 0x02);
        break;

    case -30:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x6f);
        ret |= cambus_write(0x5582, 0x40);
        ret |= cambus_write(0x558a, 0x02);
        break;

    case 0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x80);
        ret |= cambus_write(0x5582, 0x00);
        ret |= cambus_write(0x558a, 0x01);
        break;

    case 30:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x6f);
        ret |= cambus_write(0x5582, 0x40);
        ret |= cambus_write(0x558a, 0x01);
        break;

    case 60:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x40);
        ret |= cambus_write(0x5582, 0x6f);
        ret |= cambus_write(0x558a, 0x01);
        break;

    case 90:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x00);
        ret |= cambus_write(0x5582, 0x80);
        ret |= cambus_write(0x558a, 0x31);
        break;

    case 120:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x40);
        ret |= cambus_write(0x5582, 0x6f);
        ret |= cambus_write(0x558a, 0x31);
        break;

    case 150:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x6f);
        ret |= cambus_write(0x5582, 0x40);
        ret |= cambus_write(0x558a, 0x31);
        break;

    default:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x01);
        ret |= cambus_write(0x5581, 0x80);
        ret |= cambus_write(0x5582, 0x00);
        ret |= cambus_write(0x558a, 0x01);
        break;
    }

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_gainceiling(mipi_gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t value;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    value = (gainceiling >> 8) & 0x1;
    // Set AEC gain ceiling high bit
    ret |= cambus_write(0x3A18, value);
    value = gainceiling & 0xFF;
    // Set AEC gain ceiling low bits
    ret |= cambus_write(0x3A19, value);

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    if (enable) {
        ret |= cambus_write(0x503d, 0x80);
    }

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    ret = cambus_read(TIMING_TC_REG21, &reg);

    if (enable) {
        reg |= 0x02;;
    } else {
        reg &= ~0x02;
    }

    ret |= cambus_write(TIMING_TC_REG21, reg);

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    if (enable) {
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x40);
    } else {
        ret |= cambus_write(0x5001, 0x7f);
        ret |= cambus_write(0x5580, 0x00);
    }

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;
    uint8_t reg;

    // Software power down before updating settings
    ret |= cambus_write(SYS_CTRL0, 0x42);

    ret = cambus_read(TIMING_TC_REG20, &reg);

    if (enable) {
        reg |= 0x02;
    } else {
        reg &= ~0x02;
    }

    ret |= cambus_write(TIMING_TC_REG20, reg);

    // Software power up
    ret |= cambus_write(SYS_CTRL0, 0x02);

    return ret;
}

static int get_luminance(int* lum)
{
    int ret = 0;

    *lum = 0xFFFFFF;
    return ret;
}

// clang-format off
/******************************** Public Functions ***************************/
int mipi_sensor_register(mipi_camera_t* camera)
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
    camera->set_hue             = set_hue;
    camera->set_gainceiling     = set_gainceiling;
    camera->set_colorbar        = set_colorbar;
    camera->set_hmirror         = set_hmirror;
    camera->set_vflip           = set_vflip;
    camera->set_negateimage     = set_negateimage;
    camera->get_luminance       = get_luminance;
    return 0;
}
// clang-format on
