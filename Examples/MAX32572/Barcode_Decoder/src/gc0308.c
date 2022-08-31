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
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "gc0308_regs.h"
#include "max32572.h"
#include "mxc_delay.h"
#include "sccb.h"
#include "utils.h"

#if (ACTIVE_CAMERA == CAM_GC0308)

#define cambus_writeb(addr, x) sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x) sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;

    g_slv_addr = sccb_scan();

    if (g_slv_addr == -1) {
        return -1;
    }

    return ret;
}

static int get_id(void)
{
    return g_slv_addr;
}

static int dump_registers(void)
{
    int ret = 0;
    unsigned char byt = 0;
    unsigned int i, k;
    unsigned char buf[64] = { 0 };
    unsigned char* ptr = buf;
    const char* banks[2] = { "PAGE0", "PAGE1" };
    unsigned char banks_select[2] = { PAGE_0, PAGE_1 };

    for (k = 0; k < 2; k++) {
        printf("SECTION: %s\n", banks[k]);
        cambus_writeb(PAGE_SELECT, banks_select[k]);

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
                ret = sprintf((char*)ptr, " %02X", byt);

                if (ret < 0) {
                    return ret;
                }

                ptr += 3; // XX + space
            } else {
                // printf("\nREAD FAILED: reg:%X\n", i);
                *ptr++ = '!';
                *ptr++ = '!';
                *ptr++ = ' ';
            }
        }
    }

    return ret;
}

static int reset(void)
{
    int ret;

    // Reset all registers
    ret = cambus_writeb(PAGE_SELECT, 0x80);
    utils_delay_ms(100);

    ret |= cambus_writeb(0x24, 0xA2); // YUYV
    ret |= cambus_writeb(0x25, 0x0f); // enable data, pclk, hsync, vsync
    ret |= cambus_writeb(0x26, 0x32); // hsync polarity : high
    ret |= cambus_writeb(0x28, 0x10); // InternalClk = InputClk / ((1+1))

    return ret;
}

static int sleep(int enable)
{
    int ret = 0;

    return ret;
}

static int read_reg(uint16_t reg_addr)
{
    uint8_t reg;

    if (sccb_read_byt(g_slv_addr, reg_addr, &reg) != 0) {
        return -1;
    }

    return reg;
}

static int write_reg(uint16_t reg_addr, uint16_t reg_data)
{
    return sccb_write_byt(g_slv_addr, reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret;
    uint8_t reg;

    ret = cambus_readb(REG_Output_format, &reg);

    reg &= 0x1f; // clear first 5 bits

    switch (pixformat) {
    case PIXFORMAT_RGB565:
        reg |= 0x06;
        break;

    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        reg |= 0x02;
        break;

        //        case PIXFORMAT_GRAYSCALE:
        //          reg |= 0x11;
        //          break;
    case PIXFORMAT_BAYER:
        reg |= 0x17;
        break;

    default:
        return -1;
    }

    // Write back register
    return cambus_writeb(REG_Output_format, reg) | ret;
}

extern const int resolution[][2];

static int set_framesize(framesize_t framesize)
{
    int ret = 0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];

    // Write MSBs
    ret |= cambus_writeb(0x49, (h >> 8) & 0xff);
    ret |= cambus_writeb(0x4A, (h >> 0) & 0xff);

    ret |= cambus_writeb(0x4b, (w >> 8) & 0xff);
    ret |= cambus_writeb(0x4c, (w >> 0) & 0xff);

    if ((w <= 320) && (h <= 240)) {
        // Set QVGA Resolution

    } else {
        // Set VGA Resolution
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

    return ret;
}

static int set_hmirror(int enable)
{
    int ret;
    uint8_t reg;

    ret = cambus_writeb(PAGE_SELECT, PAGE_0);
    ret |= cambus_readb(0x14, &reg);

    if (enable) {
        reg &= 0xfe;
    } else {
        reg |= 0x01;
    }

    ret |= cambus_writeb(0x14, reg);

    return ret;
}

static int set_vflip(int enable)
{
    int ret;
    uint8_t reg;

    ret = cambus_writeb(PAGE_SELECT, PAGE_0);
    ret |= cambus_readb(0x14, &reg);

    if (enable) {
        reg &= 0xfd;
    } else {
        reg |= 0x02;
    }

    ret |= cambus_writeb(0x14, reg);

    return ret;
}

/******************************** Public Functions ***************************/
int sensor_register(camera_t* camera)
{
    // Initialize sensor structure.
    camera->init = init;
    camera->get_id = get_id;
    camera->dump_registers = dump_registers;
    camera->reset = reset;
    camera->sleep = sleep;
    camera->read_reg = read_reg;
    camera->write_reg = write_reg;
    camera->set_pixformat = set_pixformat;
    camera->set_framesize = set_framesize;
    camera->set_contrast = set_contrast;
    camera->set_brightness = set_brightness;
    camera->set_saturation = set_saturation;
    camera->set_gainceiling = set_gainceiling;
    camera->set_colorbar = set_colorbar;
    camera->set_hmirror = set_hmirror;
    camera->set_vflip = set_vflip;

    return 0;
}

#endif // (ACTIVE_CAMERA == CAM_GC0308)
