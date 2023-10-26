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
#include "ov7670_regs.h"
#include "mxc_delay.h"
#include "max32572.h"
#include "utils.h"

#if (ACTIVE_CAMERA == CAM_OV7670)

#define cambus_writeb(addr, x) sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x) sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;

static const uint8_t default_regs[][2] = {
    /*
     * Clock scale: 3 = 15fps
     *              2 = 20fps
     *              1 = 30fps
     */
    { REG_CLKRC, 0x2 }, /* OV: clock scale (30 fps) */
    { REG_TSLB, 0x04 }, /* OV */
    { REG_COM7, 0 }, /* VGA */
    /*
     * Set the hardware window.  These values from OV don't entirely
     * make sense - hstop is less than hstart.  But they work...
     */
    { REG_HSTART, 0x13 },
    { REG_HSTOP, 0x01 },
    { REG_HREF, 0xb6 },
    { REG_VSTRT, 0x02 },
    { REG_VSTOP, 0x7a },
    { REG_VREF, 0x0a },

    { REG_COM3, 0 },
    { REG_COM14, 0 },
    /* Mystery scaling numbers */
    { REG_SCALING_XSC, 0x3a },
    { REG_SCALING_YSC, 0x35 },
    { 0x72, 0x11 },
    { 0x73, 0xf0 },
    { 0xa2, 0x02 },
    { REG_COM10, 0x20 }, //REG_COM10 PCLK does not toggle on HBLANK.

    /* Gamma curve values */
    { 0x7a, 0x20 },
    { 0x7b, 0x10 },
    { 0x7c, 0x1e },
    { 0x7d, 0x35 },
    { 0x7e, 0x5a },
    { 0x7f, 0x69 },
    { 0x80, 0x76 },
    { 0x81, 0x80 },
    { 0x82, 0x88 },
    { 0x83, 0x8f },
    { 0x84, 0x96 },
    { 0x85, 0xa3 },
    { 0x86, 0xaf },
    { 0x87, 0xc4 },
    { 0x88, 0xd7 },
    { 0x89, 0xe8 },

    /* AGC and AEC parameters.  Note we start by disabling those features,
       then turn them only after tweaking the values. */
    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
    { REG_GAIN, 0 },
    { REG_AECH, 0 },
    { REG_COM4, 0x40 }, /* magic reserved bit */
    { REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
    { REG_BD50MAX, 0x05 },
    { REG_BD60MAX, 0x07 },
    { REG_AEW, 0x95 },
    { REG_AEB, 0x33 },
    { REG_VPT, 0xe3 },
    { REG_HAECC1, 0x78 },
    { REG_HAECC2, 0x68 },
    { 0xa1, 0x03 }, /* magic */
    { REG_HAECC3, 0xd8 },
    { REG_HAECC4, 0xd8 },
    { REG_HAECC5, 0xf0 },
    { REG_HAECC6, 0x90 },
    { REG_HAECC7, 0x94 },
    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC },

    /* Almost all of these are magic "reserved" values.  */
    { REG_COM5, 0x61 },
    { REG_COM6, 0x4b },
    { 0x16, 0x02 },
    { REG_MVFP, 0x07 },
    { 0x21, 0x02 },
    { 0x22, 0x91 },
    { 0x29, 0x07 },
    { 0x33, 0x0b },
    { 0x35, 0x0b },
    { 0x37, 0x1d },
    { 0x38, 0x71 },
    { 0x39, 0x2a },
    { REG_COM12, 0x68 },
    { 0x4d, 0x40 }, // REG_COM12 No HREF when VSYNC is low
    { 0x4e, 0x20 },
    { REG_GFIX, 0 },
    { 0x6b, 0x4a },
    { 0x74, 0x10 },
    { 0x8d, 0x4f },
    { 0x8e, 0 },
    { 0x8f, 0 },
    { 0x90, 0 },
    { 0x91, 0 },
    { 0x96, 0 },
    { 0x9a, 0 },
    { 0xb0, 0x84 },
    { 0xb1, 0x0c },
    { 0xb2, 0x0e },
    { 0xb3, 0x82 },
    { 0xb8, 0x0a },

    /* More reserved magic, some of which tweaks white balance */
    { 0x43, 0x0a },
    { 0x44, 0xf0 },
    { 0x45, 0x34 },
    { 0x46, 0x58 },
    { 0x47, 0x28 },
    { 0x48, 0x3a },
    { 0x59, 0x88 },
    { 0x5a, 0x88 },
    { 0x5b, 0x44 },
    { 0x5c, 0x67 },
    { 0x5d, 0x49 },
    { 0x5e, 0x0e },
    { 0x6c, 0x0a },
    { 0x6d, 0x55 },
    { 0x6e, 0x11 },
    { 0x6f, 0x9f }, /* "9e for advance AWB" */
    { 0x6a, 0x40 },
    { REG_BLUE, 0x40 },
    { REG_RED, 0x60 },
    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC | COM8_AWB },

    /* Matrix coefficients */
    { 0x4f, 0x80 },
    { 0x50, 0x80 },
    { 0x51, 0 },
    { 0x52, 0x22 },
    { 0x53, 0x5e },
    { 0x54, 0x80 },
    { 0x58, 0x9e },

    { REG_COM16, COM16_AWBGAIN },
    { REG_EDGE, 0 },
    { 0x75, 0x05 },
    { 0x76, 0xe1 },
    { 0x4c, 0 },
    { 0x77, 0x01 },
    { REG_COM13, 0xc3 },
    { 0x4b, 0x09 },
    { 0xc9, 0x60 },
    { REG_COM16, 0x38 },
    { 0x56, 0x40 },

    { 0x34, 0x11 },
    { REG_COM11, COM11_EXP | COM11_HZAUTO },
    { 0xa4, 0x88 },
    { 0x96, 0 },
    { 0x97, 0x30 },
    { 0x98, 0x20 },
    { 0x99, 0x30 },
    { 0x9a, 0x84 },
    { 0x9b, 0x29 },
    { 0x9c, 0x03 },
    { 0x9d, 0x4c },
    { 0x9e, 0x3f },
    { 0x78, 0x04 },

    /* Extra-weird stuff.  Some sort of multiplexor register */
    { 0x79, 0x01 },
    { 0xc8, 0xf0 },
    { 0x79, 0x0f },
    { 0xc8, 0x00 },
    { 0x79, 0x10 },
    { 0xc8, 0x7e },
    { 0x79, 0x0a },
    { 0xc8, 0x80 },
    { 0x79, 0x0b },
    { 0xc8, 0x01 },
    { 0x79, 0x0c },
    { 0xc8, 0x0f },
    { 0x79, 0x0d },
    { 0xc8, 0x20 },
    { 0x79, 0x09 },
    { 0xc8, 0x80 },
    { 0x79, 0x02 },
    { 0xc8, 0xc0 },
    { 0x79, 0x03 },
    { 0xc8, 0x40 },
    { 0x79, 0x05 },
    { 0xc8, 0x30 },
    { 0x79, 0x26 },

    { 0xff, 0xff }, /* END MARKER */
};

static const uint8_t vga_ov7670[][2] = {
    { REG_HREF, 0xF6 }, // was B6
    { REG_HSTART, 0x13 }, // HSTART
    { REG_HSTOP, 0x01 }, // HSTOP
    { REG_VSTRT, 0x02 }, // VSTART
    { REG_VSTOP, 0x7a }, // VSTOP
    { REG_VREF, 0x0a }, // VREF
    { 0xff, 0xff }, /* END MARKER */
};

static const uint8_t qvga_ov7670[][2] = {
    { REG_COM14, 0x19 }, // divide by 2
    { 0x72, 0x11 }, // downsample by 2
    { 0x73, 0xf1 }, // divide by 2
    { REG_HSTART, 0x16 }, { REG_HSTOP, 0x04 }, { REG_HREF, 0x24 }, { REG_VSTRT, 0x02 },
    { REG_VSTOP, 0x7a },  { REG_VREF, 0x0a },  { 0xff, 0xff }, /* END MARKER */
};

static const uint8_t qqvga_ov7670[][2] = {
    { REG_COM14, 0x1a }, // divide by 4
    { 0x72, 0x22 }, // downsample by 4
    { 0x73, 0xf2 }, // divide by 4
    { REG_HSTART, 0x16 }, { REG_HSTOP, 0x04 }, { REG_HREF, 0xa4 }, { REG_VSTRT, 0x02 },
    { REG_VSTOP, 0x7a },  { REG_VREF, 0x0a },  { 0xff, 0xff }, /* END MARKER */
};

static const uint8_t yuv422_ov7670[][2] = {
    { REG_COM7, 0 }, /* Selects YUV mode */
    { REG_RGB444, 0 }, /* No RGB444 please */
    { REG_COM1, 0 }, /* CCIR601 */
    { REG_COM15, COM15_R00FF },
    { REG_COM9, 0x48 }, /* 32x gain ceiling; 0x8 is reserved bit */
    { 0x4f, 0x80 }, /* "matrix coefficient 1" */
    { 0x50, 0x80 }, /* "matrix coefficient 2" */
    { 0x51, 0 }, /* vb */
    { 0x52, 0x22 }, /* "matrix coefficient 4" */
    { 0x53, 0x5e }, /* "matrix coefficient 5" */
    { 0x54, 0x80 }, /* "matrix coefficient 6" */
    { REG_COM13, COM13_GAMMA | COM13_UVSAT },
    { 0xff, 0xff },
};

static const uint8_t rgb565_ov7670[][2] = {
    { REG_COM7, COM7_RGB }, /* Selects RGB mode */
    { REG_RGB444, 0 }, /* No RGB444 please */
    { REG_COM1, 0x0 },
    { REG_COM15, COM15_RGB565 | COM15_R00FF },
    { REG_COM9, 0x6A }, /* 128x gain ceiling; 0x8 is reserved bit */
    { 0x4f, 0xb3 }, /* "matrix coefficient 1" */
    { 0x50, 0xb3 }, /* "matrix coefficient 2" */
    { 0x51, 0 }, /* vb */
    { 0x52, 0x3d }, /* "matrix coefficient 4" */
    { 0x53, 0xa7 }, /* "matrix coefficient 5" */
    { 0x54, 0xe4 }, /* "matrix coefficient 6" */
    { REG_COM13, /*COM13_GAMMA|*/ COM13_UVSAT },
    { 0xff, 0xff }, /* END MARKER */
};

static const uint8_t bayerRGB_ov7670[][2] = {
    { REG_COM7, COM7_BAYER }, { REG_COM13, 0x08 }, /* No gamma, magic rsvd bit */
    { REG_COM16, 0x3d }, /* Edge enhancement, denoise */
    { REG_REG76, 0xe1 }, /* Pix correction, magic rsvd */
    { 0xff, 0xff }, /* END MARKER */
};

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
    unsigned int i;
    unsigned char buf[64] = { 0 };
    unsigned char *ptr = buf;
    unsigned int sz = 64;

    for (i = 0;; i++) {
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
            sz = 64;
        }

        if (i == 256) {
            break;
        }

        ret = cambus_readb(i, &byt);

        if (ret == 0) {
            ret = snprintf((char *)ptr, sz, " %02X", byt);

            if (ret < 0) {
                return ret;
            }

            ptr += 3; // XX + space
            sz -= 3;
        } else {
            //printf("\nREAD FAILED: reg:%X\n", i);
            *ptr++ = '!';
            *ptr++ = '!';
            *ptr++ = ' ';
            sz -= 3;
        }
    }

    return ret;
}

static int reset(void)
{
    int ret = 0;

    ret |= cambus_writeb(REG_COM7, COM7_RESET);
    utils_delay_ms(250);

    // Write default registers
    for (int i = 0; default_regs[i][0] != 0xff; i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }

    ret |= cambus_writeb(REG_CLKRC, 10); //InternalClk = InputClk / (2*(10+1))

    return ret;
}

static int sleep(int enable)
{
    int ret = 0;
    uint8_t reg;

    ret = cambus_readb(REG_COM2, &reg);

    if (ret == 0) {
        if (enable) {
            reg |= COM2_SOFT_SLEEP_MODE;
        } else {
            reg &= ~COM2_SOFT_SLEEP_MODE;
        }

        // Write back register
        ret |= cambus_writeb(REG_COM2, reg);
    }

    return ret;
}

static int read_reg(uint16_t reg_addr)
{
    uint8_t reg;

    if (cambus_readb(reg_addr, &reg) != 0) {
        return -1;
    }

    return reg;
}

static int write_reg(uint16_t reg_addr, uint16_t reg_data)
{
    return cambus_writeb(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret = 0;
    int i;
    uint8_t reg;

    switch (pixformat) {
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        for (i = 0; yuv422_ov7670[i][0] != 0xff; i++) {
            ret |= cambus_writeb(yuv422_ov7670[i][0], yuv422_ov7670[i][1]);
        }

        break;

    case PIXFORMAT_RGB565:
        for (i = 0; rgb565_ov7670[i][0] != 0xff; i++) {
            ret |= cambus_writeb(rgb565_ov7670[i][0], rgb565_ov7670[i][1]);
        }

        reg = read_reg(REG_CLKRC);
        utils_delay_ms(1);
        ret |= cambus_writeb(
            REG_CLKRC, reg); //according to the Linux kernel driver rgb565 PCLK needs rewriting
        break;

    case PIXFORMAT_BAYER:
        for (i = 0; bayerRGB_ov7670[i][0] != 0xff; i++) {
            ret |= cambus_writeb(bayerRGB_ov7670[i][0], bayerRGB_ov7670[i][1]);
        }

        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static int set_framesize(framesize_t format)
{
    int ret = 0;
    int i;

    switch (format) {
    case FRAMESIZE_VGA:
        cambus_writeb(REG_COM3, 0);

        for (i = 0; vga_ov7670[i][0] != 0xff; i++) {
            ret |= cambus_writeb(vga_ov7670[i][0], vga_ov7670[i][1]);
        }

        break;

    case FRAMESIZE_QVGA:
        cambus_writeb(REG_COM3, 4); // enable scaling

        for (i = 0; qvga_ov7670[i][0] != 0xff; i++) {
            ret |= cambus_writeb(qvga_ov7670[i][0], qvga_ov7670[i][1]);
        }

        break;

    case FRAMESIZE_QQVGA:
        cambus_writeb(REG_COM3, 4); // enable scaling

        for (i = 0; qqvga_ov7670[i][0] != 0xff; i++) {
            ret |= cambus_writeb(qqvga_ov7670[i][0], qqvga_ov7670[i][1]);
        }

        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;

    switch (level) {
    case -4:
        level = 0;
        break;

    case -3:
        level = 30;
        break;

    case -2:
        level = 60;
        break;

    case -1:
        level = 90;
        break;

    case 0:
        level = 120;
        break;

    case 1:
        level = 150;
        break;

    case 2:
        level = 180;
        break;

    case 3:
        level = 210;
        break;

    case 4:
        level = 255;
        break;

    default:
        return -1;
    }

    ret = cambus_writeb(REG_CONTRAS, level);

    return ret;
}

static int set_brightness(int level)
{
    int ret;
    unsigned char reg = 0;

    switch (level) {
    case -4:
        level = 0;
        break;

    case -3:
        level = 30;
        break;

    case -2:
        level = 60;
        break;

    case -1:
        level = 90;
        break;

    case 0:
        level = 120;
        break;

    case 1:
        level = 150;
        break;

    case 2:
        level = 180;
        break;

    case 3:
        level = 210;
        break;

    case 4:
        level = 255;
        break;

    default:
        return -1;
    }

    // update REG_COM8
    ret = cambus_readb(REG_COM8, &reg);
    reg &= ~COM8_AEC;
    ret = cambus_writeb(REG_COM8, reg);

    // update REG_BRIGHT
    level &= 0xFF;
    reg = (level > 127) ? (level & 0x7f) : ((128 - level) | 0x80);
    ret = cambus_writeb(REG_BRIGHT, reg);

    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;
    (void)level;

    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t reg = 0;

    ret = cambus_readb(REG_COM9, &reg);

    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);
    ret |= cambus_writeb(REG_COM9, reg);

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;
    (void)enable;

    return ret;
}

static int set_hmirror(int enable)
{
    unsigned char reg = 0;
    int ret;

    ret = cambus_readb(REG_MVFP, &reg);

    if (enable) {
        reg |= MVFP_MIRROR;
    } else {
        reg &= ~MVFP_MIRROR;
    }

    ret |= cambus_writeb(REG_MVFP, reg);

    return ret;
}

static int set_vflip(int enable)
{
    unsigned char reg = 0;
    int ret;

    ret = cambus_readb(REG_MVFP, &reg);

    if (enable) {
        reg |= MVFP_FLIP;
    } else {
        reg &= ~MVFP_FLIP;
    }

    ret |= cambus_writeb(REG_MVFP, reg);

    return ret;
}

/******************************** Public Functions ***************************/
int sensor_register(camera_t *camera)
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

#endif // (ACTIVE_CAMERA == CAM_OV7670)
