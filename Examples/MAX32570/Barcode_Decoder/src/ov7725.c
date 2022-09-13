/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7725 driver.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "sccb.h"
#include "ov7725_regs.h"
#include "mxc_delay.h"
#include "max32570.h"
#include "utils.h"

#if (ACTIVE_CAMERA == CAM_OV7725)

#define cambus_writeb(addr, x) sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x) sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;
static const uint8_t default_regs[][2] = {

    // From App Note.

    { COM12, 0x03 },
    { HSTART, 0x22 },
    { HSIZE, 0xa4 },
    { VSTART, 0x07 },
    { VSIZE, 0xf0 },
    { HREF, 0x00 },
    { HOUTSIZE, 0xa0 },
    { VOUTSIZE, 0xf0 },
    { EXHCH, 0x00 },
    { CLKRC, 0xC0 }, // {CLKRC, 0x01},

    { TGT_B, 0x7f },
    { FIXGAIN, 0x09 },
    { AWB_CTRL0, 0xe0 },
    { DSP_CTRL1, 0xff },
    { DSP_CTRL2, 0x20 | DSP_CTRL2_VDCW_EN | DSP_CTRL2_HDCW_EN | DSP_CTRL2_VZOOM_EN |
                     DSP_CTRL2_HZOOM_EN }, // {DSP_CTRL2, 0x20},
    { DSP_CTRL3, 0x00 },
    { DSP_CTRL4, 0x48 },

    { COM8, 0xf0 },
    { COM4, 0x41 }, // {COM4, OMV_OV7725_PLL_CONFIG},
    { COM6, 0xc5 },
    { COM9, 0x11 },
    { BDBASE, 0x7f },
    { BDSTEP, 0x03 },
    { AEW, 0x40 },
    { AEB, 0x30 },
    { VPT, 0xa1 },
    { EXHCL, 0x00 },
    { AWB_CTRL3, 0xaa },
    { COM8, 0xff },

    { EDGE1, 0x05 },
    { DNSOFF, 0x01 },
    { EDGE2, 0x03 },
    { EDGE3, 0x00 },
    { MTX1, 0xb0 },
    { MTX2, 0x9d },
    { MTX3, 0x13 },
    { MTX4, 0x16 },
    { MTX5, 0x7b },
    { MTX6, 0x91 },
    { MTX_CTRL, 0x1e },
    { BRIGHTNESS, 0x08 },
    { CONTRAST, 0x20 },
    { UVADJ0, 0x81 },
    { SDE, SDE_CONT_BRIGHT_EN | SDE_SATURATION_EN },

    { GAM1, 0x0c },
    { GAM2, 0x16 },
    { GAM3, 0x2a },
    { GAM4, 0x4e },
    { GAM5, 0x61 },
    { GAM6, 0x6f },
    { GAM7, 0x7b },
    { GAM8, 0x86 },
    { GAM9, 0x8e },
    { GAM10, 0x97 },
    { GAM11, 0xa4 },
    { GAM12, 0xaf },
    { GAM13, 0xc5 },
    { GAM14, 0xd7 },
    { GAM15, 0xe8 },
    { SLOP, 0x20 },

    { DM_LNL, 0x00 },
    { BDBASE, 0x7f }, // {BDBASE, OMV_OV7725_BANDING}
    { BDSTEP, 0x03 },

    { LC_RADI, 0x10 },
    { LC_COEF, 0x10 },
    { LC_COEFB, 0x14 },
    { LC_COEFR, 0x17 },
    { LC_CTR, 0x01 }, // {LC_CTR, 0x05},

    { COM5, 0xf5 }, // {COM5, 0x65},

    // OpenMV Custom.

    { COM7, COM7_FMT_RGB565 },

    // End.

    { 0x00, 0x00 },
};

#define NUM_BRIGHTNESS_LEVELS (9)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS][2] = {
    { 0x38, 0x0e }, /* -4 */
    { 0x28, 0x0e }, /* -3 */
    { 0x18, 0x0e }, /* -2 */
    { 0x08, 0x0e }, /* -1 */
    { 0x08, 0x06 }, /*  0 */
    { 0x18, 0x06 }, /* +1 */
    { 0x28, 0x06 }, /* +2 */
    { 0x38, 0x06 }, /* +3 */
    { 0x48, 0x06 }, /* +4 */
};

#define NUM_CONTRAST_LEVELS (9)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS][1] = {
    { 0x10 }, /* -4 */
    { 0x14 }, /* -3 */
    { 0x18 }, /* -2 */
    { 0x1C }, /* -1 */
    { 0x20 }, /*  0 */
    { 0x24 }, /* +1 */
    { 0x28 }, /* +2 */
    { 0x2C }, /* +3 */
    { 0x30 }, /* +4 */
};

#define NUM_SATURATION_LEVELS (9)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS][2] = {
    { 0x00, 0x00 }, /* -4 */
    { 0x10, 0x10 }, /* -3 */
    { 0x20, 0x20 }, /* -2 */
    { 0x30, 0x30 }, /* -1 */
    { 0x40, 0x40 }, /*  0 */
    { 0x50, 0x50 }, /* +1 */
    { 0x60, 0x60 }, /* +2 */
    { 0x70, 0x70 }, /* +3 */
    { 0x80, 0x80 }, /* +4 */
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
            ret = sprintf((char *)ptr, " %02X", byt);

            if (ret < 0) {
                return ret;
            }

            ptr += 3; // XX + space
        } else {
            //printf("\nREAD FAILED: reg:%X\n", i);
            *ptr++ = '!';
            *ptr++ = '!';
            *ptr++ = ' ';
        }
    }

    return ret;
}

static int reset(void)
{
    // Reset all registers
    int ret = cambus_writeb(COM7, COM7_RESET);

    // Delay 2 ms
    utils_delay_ms(2);

    // Write default regsiters
    for (int i = 0; default_regs[i][0]; i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }

    ret |= cambus_writeb(CLKRC, 2); //InternalClk = InputClk / (2*(2+1))

    // Delay 300 ms
    utils_delay_ms(300);

    return ret;
}

static int sleep(int enable)
{
    uint8_t reg = 0;
    int ret = cambus_readb(COM2, &reg);

    if (enable) {
        reg |= COM2_SOFT_SLEEP;
    } else {
        reg &= ~COM2_SOFT_SLEEP;
    }

    // Write back register
    return cambus_writeb(COM2, reg) | ret;
}

static int read_reg(uint16_t reg_addr)
{
    uint8_t reg_data;

    if (cambus_readb(reg_addr, &reg_data) != 0) {
        return -1;
    }

    return reg_data;
}

static int write_reg(uint16_t reg_addr, uint16_t reg_data)
{
    return cambus_writeb(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    uint8_t reg = 0;
    int ret = cambus_readb(COM7, &reg);

    switch (pixformat) {
    case PIXFORMAT_RGB565:
        reg = COM7_SET_FMT(reg, COM7_FMT_RGB);
        ret |= cambus_writeb(DSP_CTRL4, DSP_CTRL4_YUV_RGB);
        break;

    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        reg = COM7_SET_FMT(reg, COM7_FMT_YUV);
        ret |= cambus_writeb(DSP_CTRL4, DSP_CTRL4_YUV_RGB);
        break;

    case PIXFORMAT_BAYER:
        reg = COM7_SET_FMT(reg, COM7_FMT_P_BAYER);
        ret |= cambus_writeb(DSP_CTRL4, DSP_CTRL4_RAW8);
        break;

    default:
        return -1;
    }

    // Write back register
    return cambus_writeb(COM7, reg) | ret;
}

extern const int resolution[][2];

static int set_framesize(framesize_t framesize)
{
    int ret = 0;
    uint8_t reg = 0;
    uint16_t w = resolution[(unsigned int)framesize][0];
    uint16_t h = resolution[(unsigned int)framesize][1];

    // Write MSBs
    ret |= cambus_writeb(HOUTSIZE, w >> 2);
    ret |= cambus_writeb(VOUTSIZE, h >> 1);

    // Write LSBs
    ret |= cambus_writeb(EXHCH, ((w & 0x3) | ((h & 0x1) << 2)));

    if ((w <= 320) && (h <= 240)) {
        // Set QVGA Resolution
        ret = cambus_readb(COM7, &reg);
        reg = COM7_SET_RES(reg, COM7_RES_QVGA);
        ret |= cambus_writeb(COM7, reg);

        // Set QVGA Window Size
        ret |= cambus_writeb(HSTART, 0x3F);
        ret |= cambus_writeb(HSIZE, 0x50);
        ret |= cambus_writeb(VSTART, 0x03);
        ret |= cambus_writeb(VSIZE, 0x78);

        // Enable auto-scaling/zooming factors
        ret |= cambus_writeb(DSPAUTO, 0xFF);
    } else {
        // Set VGA Resolution
        ret = cambus_readb(COM7, &reg);
        reg = COM7_SET_RES(reg, COM7_RES_VGA);
        ret |= cambus_writeb(COM7, reg);

        // Set VGA Window Size
        ret |= cambus_writeb(HSTART, 0x23);
        ret |= cambus_writeb(HSIZE, 0xA0);
        ret |= cambus_writeb(VSTART, 0x07);
        ret |= cambus_writeb(VSIZE, 0xF0);

        // Disable auto-scaling/zooming factors
        ret |= cambus_writeb(DSPAUTO, 0xF3);

        // Clear auto-scaling/zooming factors
        ret |= cambus_writeb(SCAL0, 0x00);
        ret |= cambus_writeb(SCAL1, 0x40);
        ret |= cambus_writeb(SCAL2, 0x40);
    }

    return ret;
}
static int set_contrast(int level)
{
    int ret = 0;

    level += (NUM_CONTRAST_LEVELS / 2);

    if (level < 0 || level >= NUM_CONTRAST_LEVELS) {
        return -1;
    }

    ret = cambus_writeb(CONTRAST, contrast_regs[level][0]);
    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;

    level += (NUM_BRIGHTNESS_LEVELS / 2);

    if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    ret |= cambus_writeb(BRIGHTNESS, brightness_regs[level][0]);
    ret |= cambus_writeb(SIGN_BIT, brightness_regs[level][1]);
    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;

    level += (NUM_SATURATION_LEVELS / 2);

    if (level < 0 || level >= NUM_SATURATION_LEVELS) {
        return -1;
    }

    ret |= cambus_writeb(USAT, saturation_regs[level][0]);
    ret |= cambus_writeb(VSAT, saturation_regs[level][1]);
    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    uint8_t reg = 0;
    int ret = cambus_readb(COM9, &reg);

    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);
    return cambus_writeb(COM9, reg) | ret;
}

static int set_colorbar(int enable)
{
    uint8_t reg = 0;
    int ret = cambus_readb(COM3, &reg);

    // Enable colorbar test pattern output
    reg = COM3_SET_CBAR(reg, enable);
    ret |= cambus_writeb(COM3, reg);

    // Enable DSP colorbar output
    ret |= cambus_readb(DSP_CTRL3, &reg);
    reg = DSP_CTRL3_SET_CBAR(reg, enable);
    return cambus_writeb(DSP_CTRL3, reg) | ret;
}

static int set_hmirror(int enable)
{
    uint8_t reg = 0;
    int ret = cambus_readb(COM3, &reg);
    ret |= cambus_writeb(COM3, COM3_SET_MIRROR(reg, enable));

    return ret;
}

static int set_vflip(int enable)
{
    uint8_t reg = 0;
    int ret = cambus_readb(COM3, &reg);
    ret |= cambus_writeb(COM3, COM3_SET_FLIP(reg, enable));

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

#endif // (ACTIVE_CAMERA == CAM_OV2640)
