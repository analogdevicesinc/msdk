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
#include "ov5642_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"

// clang-format off
#define cambus_write(addr, x)     sccb_write_reg16(g_slv_addr, addr, x)
#define cambus_read(addr, x)      sccb_read_reg16(g_slv_addr, addr, x)

static const uint16_t default_regs[][2] = { //qcif15
    {0x3103, 0x93}, {0x3008, 0x82}, {0x3017, 0x7f},
    {0x3018, 0xfc}, {0x3810, 0xc2}, {0x3615, 0xf0},
    {0x3000, 0x00}, {0x3001, 0x00}, {0x3002, 0x5c},
    {0x3003, 0x00}, {0x3004, 0xff}, {0x3005, 0xff},
    {0x3006, 0x43}, {0x3007, 0x37}, {0x3011, 0x08},
    {0x3010, 0x10}, {0x460c, 0x22},
    //{0x3815, 0x0A},
    {0x3815, 0x08}, //QVGA, QCIF
    //{0x3815, 0x04}, //VGA
    //{0x3815, 0x01},

    {0x370c, 0xa0}, {0x3602, 0xfc}, {0x3612, 0xff},
    {0x3634, 0xc0}, {0x3613, 0x00}, {0x3605, 0x7c},
    {0x3621, 0x09}, {0x3622, 0x60}, {0x3604, 0x40},
    {0x3603, 0xa7}, {0x3603, 0x27}, {0x4000, 0x21},
    {0x401d, 0x22}, {0x3600, 0x54}, {0x3605, 0x04},
    {0x3606, 0x3f}, {0x3c01, 0x80}, {0x5000, 0x4f},
    {0x5020, 0x04}, {0x5181, 0x79}, {0x5182, 0x00},
    {0x5185, 0x22}, {0x5197, 0x01}, {0x5001, 0xff},
    {0x5500, 0x0a}, {0x5504, 0x00}, {0x5505, 0x7f},
    {0x5080, 0x08}, {0x300e, 0x18}, {0x4610, 0x00},
    {0x471d, 0x05}, {0x4708, 0x06}, {0x3808, 0x02},
    {0x3809, 0x80}, {0x380a, 0x01}, {0x380b, 0xe0},
    {0x380e, 0x07}, {0x380f, 0xd0}, {0x501f, 0x00},
    {0x5000, 0x4f}, {0x4300, 0x30}, {0x3503, 0x07},
    {0x3501, 0x73}, {0x3502, 0x80}, {0x350b, 0x00},
    {0x3503, 0x07}, {0x3824, 0x11}, {0x3501, 0x1e},
    {0x3502, 0x80}, {0x350b, 0x7f}, {0x380c, 0x0c},
    {0x380d, 0x80}, {0x380e, 0x03}, {0x380f, 0xe8},
    {0x3a0d, 0x04}, {0x3a0e, 0x03}, {0x3818, 0xc1},
    {0x3705, 0xdb}, {0x370a, 0x81}, {0x3801, 0x80},
    {0x3621, 0x87}, {0x3801, 0x50}, {0x3803, 0x08},
    {0x3827, 0x08}, {0x3810, 0x40}, {0x3804, 0x05},
    {0x3805, 0x00}, {0x5682, 0x05}, {0x5683, 0x00},
    {0x3806, 0x03}, {0x3807, 0xc0}, {0x5686, 0x03},
    {0x5687, 0xbc}, {0x3a00, 0x78}, {0x3a1a, 0x05},
    {0x3a13, 0x30}, {0x3a18, 0x00}, {0x3a19, 0x7c},
    {0x3a08, 0x12}, {0x3a09, 0xc0}, {0x3a0a, 0x0f},
    {0x3a0b, 0xa0}, {0x350c, 0x07}, {0x350d, 0xd0},
    {0x3500, 0x00}, {0x3501, 0x00}, {0x3502, 0x00},
    {0x350a, 0x00}, {0x350b, 0x00}, {0x3503, 0x00},
    {0x528a, 0x02}, {0x528b, 0x04}, {0x528c, 0x08},
    {0x528d, 0x08}, {0x528e, 0x08}, {0x528f, 0x10},
    {0x5290, 0x10}, {0x5292, 0x00}, {0x5293, 0x02},
    {0x5294, 0x00}, {0x5295, 0x02}, {0x5296, 0x00},
    {0x5297, 0x02}, {0x5298, 0x00}, {0x5299, 0x02},
    {0x529a, 0x00}, {0x529b, 0x02}, {0x529c, 0x00},
    {0x529d, 0x02}, {0x529e, 0x00}, {0x529f, 0x02},
    {0x3a0f, 0x3c}, {0x3a10, 0x30}, {0x3a1b, 0x3c},
    {0x3a1e, 0x30}, {0x3a11, 0x70}, {0x3a1f, 0x10},
    {0x3a02, 0x00}, {0x3a03, 0x7d},
    {0x3a04, 0x00}, {0x3a14, 0x00}, {0x3a15, 0x7d},
    {0x3a16, 0x00}, {0x3a00, 0x78}, {0x3a08, 0x09},
    {0x3a09, 0x60}, {0x3a0a, 0x07}, {0x3a0b, 0xd0},
    {0x3a0d, 0x08}, {0x3a0e, 0x06}, {0x5193, 0x70},
    {0x589b, 0x04}, {0x589a, 0xc5}, {0x401e, 0x20},
    {0x4001, 0x42}, {0x401c, 0x04}, {0x528a, 0x01},
    {0x528b, 0x04}, {0x528c, 0x08}, {0x528d, 0x10},
    {0x528e, 0x20}, {0x528f, 0x28}, {0x5290, 0x30},
    {0x5292, 0x00}, {0x5293, 0x01}, {0x5294, 0x00},
    {0x5295, 0x04}, {0x5296, 0x00}, {0x5297, 0x08},
    {0x5298, 0x00}, {0x5299, 0x10}, {0x529a, 0x00},
    {0x529b, 0x20}, {0x529c, 0x00}, {0x529d, 0x28},
    {0x529e, 0x00}, {0x529f, 0x30}, {0x5282, 0x00},
    {0x5300, 0x00}, {0x5301, 0x20}, {0x5302, 0x00},
    {0x5303, 0x7c}, {0x530c, 0x00}, {0x530d, 0x0c},
    {0x530e, 0x20}, {0x530f, 0x80}, {0x5310, 0x20},
    {0x5311, 0x80}, {0x5308, 0x20}, {0x5309, 0x40},
    {0x5304, 0x00}, {0x5305, 0x30}, {0x5306, 0x00},
    {0x5307, 0x80}, {0x5314, 0x08}, {0x5315, 0x20},
    {0x5319, 0x30}, {0x5316, 0x10}, {0x5317, 0x00},
    {0x5318, 0x02}, {0x5380, 0x01}, {0x5381, 0x00},
    {0x5382, 0x00}, {0x5383, 0x4e}, {0x5384, 0x00},
    {0x5385, 0x0f}, {0x5386, 0x00}, {0x5387, 0x00},
    {0x5388, 0x01}, {0x5389, 0x15}, {0x538a, 0x00},
    {0x538b, 0x31}, {0x538c, 0x00}, {0x538d, 0x00},
    {0x538e, 0x00}, {0x538f, 0x0f}, {0x5390, 0x00},
    {0x5391, 0xab}, {0x5392, 0x00}, {0x5393, 0xa2},
    {0x5394, 0x08}, {0x5480, 0x14}, {0x5481, 0x21},
    {0x5482, 0x36}, {0x5483, 0x57}, {0x5484, 0x65},
    {0x5485, 0x71}, {0x5486, 0x7d}, {0x5487, 0x87},
    {0x5488, 0x91}, {0x5489, 0x9a}, {0x548a, 0xaa},
    {0x548b, 0xb8}, {0x548c, 0xcd}, {0x548d, 0xdd},
    {0x548e, 0xea}, {0x548f, 0x1d}, {0x5490, 0x05},
    {0x5491, 0x00}, {0x5492, 0x04}, {0x5493, 0x20},
    {0x5494, 0x03}, {0x5495, 0x60}, {0x5496, 0x02},
    {0x5497, 0xb8}, {0x5498, 0x02}, {0x5499, 0x86},
    {0x549a, 0x02}, {0x549b, 0x5b}, {0x549c, 0x02},
    {0x549d, 0x3b}, {0x549e, 0x02}, {0x549f, 0x1c},
    {0x54a0, 0x02}, {0x54a1, 0x04}, {0x54a2, 0x01},
    {0x54a3, 0xed}, {0x54a4, 0x01}, {0x54a5, 0xc5},
    {0x54a6, 0x01}, {0x54a7, 0xa5}, {0x54a8, 0x01},
    {0x54a9, 0x6c}, {0x54aa, 0x01}, {0x54ab, 0x41},
    {0x54ac, 0x01}, {0x54ad, 0x20}, {0x54ae, 0x00},
    {0x54af, 0x16}, {0x54b0, 0x01}, {0x54b1, 0x20},
    {0x54b2, 0x00}, {0x54b3, 0x10}, {0x54b4, 0x00},
    {0x54b5, 0xf0}, {0x54b6, 0x00}, {0x54b7, 0xdf},
    {0x5402, 0x3f}, {0x5403, 0x00}, {0x3406, 0x00},
    {0x5180, 0xff}, {0x5181, 0x52}, {0x5182, 0x11},
    {0x5183, 0x94}, {0x5184, 0x25}, {0x5185, 0x24},
    {0x5186, 0x06}, {0x5187, 0x08}, {0x5188, 0x08},
    {0x5189, 0x7c}, {0x518a, 0x60}, {0x518b, 0xb2},
    {0x518c, 0xb2}, {0x518d, 0x44}, {0x518e, 0x3d},
    {0x518f, 0x58}, {0x5190, 0x46}, {0x5191, 0xff},
    {0x5192, 0x00}, {0x5193, 0x70}, {0x5194, 0xf0},
    {0x5195, 0xf0}, {0x5196, 0x03}, {0x5197, 0x01},
    {0x5198, 0x04}, {0x5199, 0x12}, {0x519a, 0x04},
    {0x519b, 0x00}, {0x519c, 0x06}, {0x519d, 0x82},
    {0x519e, 0x00}, {0x5025, 0x80}, {0x3a0f, 0x38},
    {0x3a10, 0x30}, {0x3a1b, 0x3a}, {0x3a1e, 0x2e},
    {0x3a11, 0x60}, {0x3a1f, 0x10}, {0x5688, 0xa6},
    {0x5689, 0x6a}, {0x568a, 0xea}, {0x568b, 0xae},
    {0x568c, 0xa6}, {0x568d, 0x6a}, {0x568e, 0x62},
    {0x568f, 0x26}, {0x5583, 0x60}, {0x5584, 0x60},
    {0x5580, 0x02}, {0x5000, 0xcf}, {0x5800, 0x27},
    {0x5801, 0x19}, {0x5802, 0x12}, {0x5803, 0x0f},
    {0x5804, 0x10}, {0x5805, 0x15}, {0x5806, 0x1e},
    {0x5807, 0x2f}, {0x5808, 0x15}, {0x5809, 0x0d},
    {0x580a, 0x0a}, {0x580b, 0x09}, {0x580c, 0x0a},
    {0x580d, 0x0c}, {0x580e, 0x12}, {0x580f, 0x19},
    {0x5810, 0x0b}, {0x5811, 0x07}, {0x5812, 0x04},
    {0x5813, 0x03}, {0x5814, 0x03}, {0x5815, 0x06},
    {0x5816, 0x0a}, {0x5817, 0x0f}, {0x5818, 0x0a},
    {0x5819, 0x05}, {0x581a, 0x01}, {0x581b, 0x00},
    {0x581c, 0x00}, {0x581d, 0x03}, {0x581e, 0x08},
    {0x581f, 0x0c}, {0x5820, 0x0a}, {0x5821, 0x05},
    {0x5822, 0x01}, {0x5823, 0x00}, {0x5824, 0x00},
    {0x5825, 0x03}, {0x5826, 0x08}, {0x5827, 0x0c},
    {0x5828, 0x0e}, {0x5829, 0x08}, {0x582a, 0x06},
    {0x582b, 0x04}, {0x582c, 0x05}, {0x582d, 0x07},
    {0x582e, 0x0b}, {0x582f, 0x12}, {0x5830, 0x18},
    {0x5831, 0x10}, {0x5832, 0x0c}, {0x5833, 0x0a},
    {0x5834, 0x0b}, {0x5835, 0x0e}, {0x5836, 0x15},
    {0x5837, 0x19}, {0x5838, 0x32}, {0x5839, 0x1f},
    {0x583a, 0x18}, {0x583b, 0x16}, {0x583c, 0x17},
    {0x583d, 0x1e}, {0x583e, 0x26}, {0x583f, 0x53},
    {0x5840, 0x10}, {0x5841, 0x0f}, {0x5842, 0x0d},
    {0x5843, 0x0c}, {0x5844, 0x0e}, {0x5845, 0x09},
    {0x5846, 0x11}, {0x5847, 0x10}, {0x5848, 0x10},
    {0x5849, 0x10}, {0x584a, 0x10}, {0x584b, 0x0e},
    {0x584c, 0x10}, {0x584d, 0x10}, {0x584e, 0x11},
    {0x584f, 0x10}, {0x5850, 0x0f}, {0x5851, 0x0c},
    {0x5852, 0x0f}, {0x5853, 0x10}, {0x5854, 0x10},
    {0x5855, 0x0f}, {0x5856, 0x0e}, {0x5857, 0x0b},
    {0x5858, 0x10}, {0x5859, 0x0d}, {0x585a, 0x0d},
    {0x585b, 0x0c}, {0x585c, 0x0c}, {0x585d, 0x0c},
    {0x585e, 0x0b}, {0x585f, 0x0c}, {0x5860, 0x0c},
    {0x5861, 0x0c}, {0x5862, 0x0d}, {0x5863, 0x08},
    {0x5864, 0x11}, {0x5865, 0x18}, {0x5866, 0x18},
    {0x5867, 0x19}, {0x5868, 0x17}, {0x5869, 0x19},
    {0x586a, 0x16}, {0x586b, 0x13}, {0x586c, 0x13},
    {0x586d, 0x12}, {0x586e, 0x13}, {0x586f, 0x16},
    {0x5870, 0x14}, {0x5871, 0x12}, {0x5872, 0x10},
    {0x5873, 0x11}, {0x5874, 0x11}, {0x5875, 0x16},
    {0x5876, 0x14}, {0x5877, 0x11}, {0x5878, 0x10},
    {0x5879, 0x0f}, {0x587a, 0x10}, {0x587b, 0x14},
    {0x587c, 0x13}, {0x587d, 0x12}, {0x587e, 0x11},
    {0x587f, 0x11}, {0x5880, 0x12}, {0x5881, 0x15},
    {0x5882, 0x14}, {0x5883, 0x15}, {0x5884, 0x15},
    {0x5885, 0x15}, {0x5886, 0x13}, {0x5887, 0x17},
    {0x3710, 0x10}, {0x3632, 0x51}, {0x3702, 0x10},
    {0x3703, 0xb2}, {0x3704, 0x18}, {0x370b, 0x40},
    {0x370d, 0x03}, {0x3631, 0x01}, {0x3632, 0x52},
    {0x3606, 0x24}, {0x3620, 0x96}, {0x5785, 0x07},
    {0x3a13, 0x30}, {0x3600, 0x52}, {0x3604, 0x48},
    {0x3606, 0x1b}, {0x370d, 0x0b}, {0x370f, 0xc0},
    {0x3709, 0x01}, {0x3823, 0x00}, {0x5007, 0x00},
    {0x5009, 0x00}, {0x5011, 0x00}, {0x5013, 0x00},
    {0x519e, 0x00}, {0x5086, 0x00}, {0x5087, 0x00},
    {0x5088, 0x00}, {0x5089, 0x00}, {0x302b, 0x00},
    {0x3808, 0x00}, {0x3809, 0xb0}, {0x380a, 0x00},
    {0x380b, 0x90}, {0x3a00, 0x78},
    {0x4302, 0x03}, {0x4303, 0xac}, /* Clip Y range to [16..235] (*4) */
    {0x4304, 0x00}, {0x4305, 0x40},

    {0x3800, 0x1 }, // HSTART=336
    {0x3801, 0x50},
    {0x3802, 0x0 }, // VSTART=8
    {0x3803, 0x8 },
    {0x3804, 0x4 }, // HWIDTH=1174
    {0x3805, 0x96},
    {0x3806, 0x3 }, // VHEIGHT=960
    {0x3807, 0xc0},

    {0x3808, 0x0 },  // out width=176
    {0x3809, 0xb0},
    {0x380a, 0x0 },  // out height=144
    {0x380b, 0x90},

    {0x380c, 0xc },  // Total HSIZE=3200
    {0x380d, 0x80},
    {0x380e, 0x3 },  // Total VSIZE=1000
    {0x380f, 0xe8},
    {0x5001, 0x7f},  // ISP Control
    {0x5680, 0x0 },  // Avg X Start=0
    {0x5681, 0x0 },
    {0x5682, 0x4 },  // Avg X End=1174
    {0x5683, 0x96},
    {0x5684, 0x0 },  // Avg Y Start=0
    {0x5685, 0x0 },
    {0x5686, 0x3 },  // Avg Y End=960
    {0x5687, 0xc0},

    {0x501E, 0x2A}, // RGB565 dither
    {0x5002, 0xF8}, // RGB
    {0x501f, 0x01}, // ISP RGB
    {0x4300, 0x61}, // RGB565

    //  {0x503d, 0x80}, // Test pattern: Color bar
    //  {0x503e, 0x00},

    {0xffff, 0xff}
};
// clang-format on

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_RGB565;

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
    g_slv_addr = OV5642_I2C_SLAVE_ADDR;
#endif
    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int *id)
{
    int ret = 0;

    return ret;
}

static int get_manufacture_id(int *id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;

    ret |= cambus_read(OV5642_CHIPID_HIGH, &id_high);
    ret |= cambus_read(OV5642_CHIPID_LOW, &id_low);
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

    for (i = 0x3000;; i++) {
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }

        if (i == 0x603C) {
            break;
        }

        ret = cambus_read(SYS_IO_PAD_CNTRL_REG_START + i, &byt);

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

    ret = cambus_read(SYS_CNTRL_REG, &value);
    value |= RESET_MODE;
    ret |= cambus_write(SYS_CNTRL_REG, value);
    MXC_Delay(10000);
    value &= ~RESET_MODE;
    ret |= cambus_write(SYS_CNTRL_REG, value);

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

    ret = cambus_read(SYS_CNTRL_REG, &reg);

    if (ret == 0) {
        if (enable) {
            reg |= SLEEP_MODE;
        } else {
            reg &= ~SLEEP_MODE;
        }

        // Write back register
        ret |= cambus_write(SYS_CNTRL_REG, reg);
    }

    return ret;
}

static int read_reg(uint16_t reg_addr, uint8_t *reg_data)
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

    switch (pixformat) {
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        ret |= cambus_write(ISP_FORMAT_MUX_CNTRL_REG, COLOR_YUV);
        ret |= cambus_write(FORMAT_CNTRL_REG, FORMAT_YUV422);
        break;

    case PIXFORMAT_RGB444:
        ret |= cambus_write(ISP_FORMAT_MUX_CNTRL_REG, COLOR_RGB);
        ret |= cambus_write(FORMAT_CNTRL_REG, FORMAT_RGB444);
        break;

    case PIXFORMAT_RGB565:
        ret |= cambus_write(ISP_FORMAT_MUX_CNTRL_REG, COLOR_RGB);
        ret |= cambus_write(FORMAT_CNTRL_REG, FORMAT_RGB565);
        break;

    case PIXFORMAT_RGB888:
        ret |= cambus_write(ISP_FORMAT_MUX_CNTRL_REG, COLOR_RGB);
        ret |= cambus_write(FORMAT_CNTRL_REG, FORMAT_RGB888);
        break;

    case PIXFORMAT_BAYER:
        ret |= cambus_write(ISP_FORMAT_MUX_CNTRL_REG, COLOR_RAW);
        ret |= cambus_write(FORMAT_CNTRL_REG, FORMAT_RAW);
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
#if 0
    uint8_t input_factor_4_3[] = { 0x0A, 0x20, 0x07, 0x98 }; // 2592 x 1944
    uint8_t input_factor_1_1[] = { 0x07, 0x98, 0x07, 0x98 }; // 1944 x 1944
    uint8_t* input_factor_ptr = input_factor_4_3;

    // Check and see if the target resolution is very small
    // that is less than 42 x 42, if so then apply and
    // x and y scaling factor.
    if ((width == height) || (width < 42) || (height < 42)) {
        input_factor_ptr = input_factor_1_1;
    }

#endif
    // Image typically outputs one line short, add a line to account.
    //height = height + 1;
    // Apply passed in resolution as output resolution.
    ret |= cambus_write(OUT_H_WIDTH_HI_REG, (width >> 8) & 0xff);
    ret |= cambus_write(OUT_H_WIDTH_LO_REG, (width >> 0) & 0xff);
    ret |= cambus_write(OUT_V_HEIGHT_HI_REG, (height >> 8) & 0xff);
    ret |= cambus_write(OUT_V_HEIGHT_LO_REG, (height >> 0) & 0xff);
#if 0
    // Apply the appropriate input image factor.
    ret |= cambus_write(TOTAL_H_SIZE_HI_REG, input_factor_ptr[0]);
    ret |= cambus_write(TOTAL_H_SIZE_LO_REG, input_factor_ptr[1]);
    ret |= cambus_write(TOTAL_V_SIZE_HI_REG, input_factor_ptr[2]);
    ret |= cambus_write(TOTAL_V_SIZE_LO_REG, input_factor_ptr[3]);

    // Apply average window start and end
    ret |= cambus_write(AVG_X_START_HI_REG, 0);
    ret |= cambus_write(AVG_X_START_LO_REG, 0);
    ret |= cambus_write(AVG_X_END_HI_REG, input_factor_ptr[0]);
    ret |= cambus_write(AVG_X_END_LO_REG, input_factor_ptr[1]);
    ret |= cambus_write(AVG_Y_START_HI_REG, 0);
    ret |= cambus_write(AVG_Y_START_LO_REG, 0);
    ret |= cambus_write(AVG_Y_END_HI_REG, input_factor_ptr[2]);
    ret |= cambus_write(AVG_Y_END_LO_REG, input_factor_ptr[3]);

    // Enable ISP vertical and horizontal scale
    ret |= cambus_write(ISP_CNTRL_REG, 0x7f);
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
    int start_x;
    int start_y;

    if (width < hsize || height < vsize) {
        ret = -1;
    }

    // Apply passed in resolution as input resolution.
    ret |= cambus_write(TOTAL_H_SIZE_HI_REG, (width >> 8) & 0xff);
    ret |= cambus_write(TOTAL_H_SIZE_LO_REG, (width >> 0) & 0xff);
    ret |= cambus_write(TOTAL_V_SIZE_HI_REG, (height >> 8) & 0xff);
    ret |= cambus_write(TOTAL_V_SIZE_LO_REG, (height >> 0) & 0xff);

    // Apply passed in hsize & vsize as output resolution.
    ret |= cambus_write(OUT_H_WIDTH_HI_REG, (hsize >> 8) & 0xff);
    ret |= cambus_write(OUT_H_WIDTH_LO_REG, (hsize >> 0) & 0xff);
    ret |= cambus_write(OUT_V_HEIGHT_HI_REG, (vsize >> 8) & 0xff);
    ret |= cambus_write(OUT_V_HEIGHT_LO_REG, (vsize >> 0) & 0xff);

    // adjust center position
    start_x = (width - hsize) >> 1;
    start_y = (height - vsize) >> 1;

    // set window start position
    ret |= cambus_write(H_START_HI_REG, (start_x >> 8) & 0xff);
    ret |= cambus_write(H_START_LO_REG, (start_x >> 0) & 0xff);
    ret |= cambus_write(V_START_HI_REG, (start_y >> 8) & 0xff);
    ret |= cambus_write(V_START_LO_REG, (start_y >> 0) & 0xff);

    // set window size
    ret |= cambus_write(H_WIDTH_HI_REG, (hsize >> 8) & 0xff);
    ret |= cambus_write(H_WIDTH_LO_REG, (hsize >> 0) & 0xff);
    ret |= cambus_write(V_HEIGHT_HI_REG, (vsize >> 8) & 0xff);
    ret |= cambus_write(V_HEIGHT_LO_REG, (vsize >> 0) & 0xff);

    // Enable ISP vertical and horizontal scale
    ret |= cambus_write(ISP_CNTRL_REG, 0x7f);

    return ret;
}
static int set_contrast(int level)
{
    int ret = 0;

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

    case 0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x20);
        ret |= cambus_write(0x5588, 0x20);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x24);
        ret |= cambus_write(0x5588, 0x24);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x28);
        ret |= cambus_write(0x5588, 0x28);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x2c);
        ret |= cambus_write(0x5588, 0x2c);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x5587, 0x30);
        ret |= cambus_write(0x5588, 0x30);
        ret |= cambus_write(0x558a, 0x00);
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

    case 0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x00);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x10);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x20);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x30);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    case 4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5589, 0x40);
        ret |= cambus_write(0x5580, 0x04);
        ret |= cambus_write(0x558a, 0x00);
        break;

    default:
        return -1;
    }

    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;

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

    case 0:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x40);
        ret |= cambus_write(0x5584, 0x40);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case 1:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x50);
        ret |= cambus_write(0x5584, 0x50);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case 2:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x60);
        ret |= cambus_write(0x5584, 0x60);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case 3:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x70);
        ret |= cambus_write(0x5584, 0x70);
        ret |= cambus_write(0x5580, 0x02);
        break;

    case 4:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5583, 0x80);
        ret |= cambus_write(0x5584, 0x80);
        ret |= cambus_write(0x5580, 0x02);
        break;

    default:
        return -1;
    }

    return ret;
}
#if 0
static int set_hue(int degree)
{
    int ret = 0;

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

    return ret;
}

static int set_special_effect(ov5642_effect_t effect)
{
    int ret = 0;

    switch (effect) {
    case NORMAL:
        ret |= cambus_write(0x5001, 0x7f);
        ret |= cambus_write(0x5580, 0x00);
        break;

    case BLUE:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x18);
        ret |= cambus_write(0x5585, 0xa0);
        ret |= cambus_write(0x5586, 0x40);
        break;

    case GREEN:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x18);
        ret |= cambus_write(0x5585, 0x60);
        ret |= cambus_write(0x5586, 0x60);
        break;

    case RED:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x18);
        ret |= cambus_write(0x5585, 0x80);
        ret |= cambus_write(0x5586, 0xc0);
        break;

    case BLACK_WHITE:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x18);
        ret |= cambus_write(0x5585, 0x80);
        ret |= cambus_write(0x5586, 0x80);
        break;

    case NEGATIVE:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x40);
        break;

    case GRAY:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x20);
        break;

    case SEPIA:
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x18);
        ret |= cambus_write(0x5585, 0x40);
        ret |= cambus_write(0x5586, 0xa0);
        break;

    default:
        ret |= cambus_write(0x5001, 0x7f);
        ret |= cambus_write(0x5580, 0x00);
        break;
    }

    return ret;
}
#endif

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t value;

    value = (gainceiling >> 8) & 0x1;
    // Set AEC gain ceiling high bit
    ret |= cambus_write(0x3A18, value);
    value = gainceiling & 0xFF;
    // Set AEC gain ceiling low bits
    ret |= cambus_write(0x3A19, value);

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;

    if (enable) {
        ret |= cambus_write(0x503d, 0x80);
        ret |= cambus_write(0x503e, 0x00);
    }

    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_read(TIME_CNTRL_18_REG, &reg);

    if (enable) {
        reg |= HORIZONTAL_MIRROR;
    } else {
        reg &= ~HORIZONTAL_MIRROR;
    }

    ret |= cambus_write(TIME_CNTRL_18_REG, reg);

    ret |= cambus_read(ARRAY_CTRL_01_REG, &reg);
    reg &= 0xDF;
    ret |= cambus_write(ARRAY_CTRL_01_REG, reg);

    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;

    if (enable) {
        ret |= cambus_write(0x5001, 0xff);
        ret |= cambus_write(0x5580, 0x40);
    } else {
        ret |= cambus_write(0x5001, 0x7f);
        ret |= cambus_write(0x5580, 0x00);
    }

    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_read(TIME_CNTRL_18_REG, &reg);

    if (enable) {
        reg |= VERTICAL_FLIP;
    } else {
        reg &= ~VERTICAL_FLIP;
    }

    ret |= cambus_write(TIME_CNTRL_18_REG, reg);

    ret |= cambus_read(ARRAY_CTRL_01_REG, &reg);
    reg |= 0x20;
    ret |= cambus_write(ARRAY_CTRL_01_REG, reg);

    return ret;
}

static int get_luminance(int *lum)
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
