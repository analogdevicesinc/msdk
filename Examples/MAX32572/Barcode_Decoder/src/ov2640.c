/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "max32572.h"
#include "mxc_delay.h"
#include "ov2640_regs.h"
#include "sccb.h"
#include "utils.h"

#if (ACTIVE_CAMERA == CAM_OV2640)

#define cambus_writeb(addr, x) sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x) sccb_read_byt(g_slv_addr, addr, x)

#define IM_LOG2_2(x) (((x)&0x2ULL) ? (2) : 1) // NO ({ ... }) !
#define IM_LOG2_4(x) (((x)&0xCULL) ? (2 + IM_LOG2_2((x) >> 2)) : IM_LOG2_2(x)) // NO ({ ... }) !
#define IM_LOG2_8(x) (((x)&0xF0ULL) ? (4 + IM_LOG2_4((x) >> 4)) : IM_LOG2_4(x)) // NO ({ ... }) !
#define IM_LOG2_16(x) (((x)&0xFF00ULL) ? (8 + IM_LOG2_8((x) >> 8)) : IM_LOG2_8(x)) // NO ({ ... }) !
#define IM_LOG2_32(x)                                                                              \
    (((x)&0xFFFF0000ULL) ? (16 + IM_LOG2_16((x) >> 16)) : IM_LOG2_16(x)) // NO ({ ... }) !
#define IM_LOG2(x)                                                                                 \
    (((x)&0xFFFFFFFF00000000ULL) ? (32 + IM_LOG2_32((x) >> 32)) : IM_LOG2_32(x)) // NO ({ ... }) !

#define IM_MAX(a, b)                                                                               \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _a > _b ? _a : _b;                                                                         \
    })
#define IM_MIN(a, b)                                                                               \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _a < _b ? _a : _b;                                                                         \
    })
#define IM_DIV(a, b)                                                                               \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _b ? (_a / _b) : 0;                                                                        \
    })
#define IM_MOD(a, b)                                                                               \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _b ? (_a % _b) : 0;                                                                        \
    })

#define CIF_WIDTH (400)
#define CIF_HEIGHT (296)

#define SVGA_WIDTH (800)
#define SVGA_HEIGHT (600)

#define UXGA_WIDTH (1600)
#define UXGA_HEIGHT (1200)

static int g_slv_addr;

static const uint8_t default_regs[][2] = {

    // From Linux Driver.

    { BANK_SEL, BANK_SEL_DSP }, { 0x2c, 0xff }, { 0x2e, 0xdf }, { BANK_SEL, BANK_SEL_SENSOR },
    { 0x3c, 0x32 }, { CLKRC, CLKRC_DOUBLE }, { COM2, COM2_OUT_DRIVE_3x },
    { REG04, REG04_SET(REG04_HFLIP_IMG | REG04_VFLIP_IMG | REG04_VREF_EN | REG04_HREF_EN) },
    { COM8, COM8_SET(COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN) },
    { COM9, COM9_AGC_SET(COM9_AGC_GAIN_8x) }, { 0x2c, 0x0c }, { 0x33, 0x78 }, { 0x3a, 0x33 },
    { 0x3b, 0xfb }, { 0x3e, 0x00 }, { 0x43, 0x11 }, { 0x16, 0x10 }, { 0x39, 0x02 }, { 0x35, 0x88 },
    { 0x22, 0x0a }, { 0x37, 0x40 }, { 0x23, 0x00 }, { ARCOM2, 0xa0 }, { 0x06, 0x02 },
    { 0x06, 0x88 }, { 0x07, 0xc0 }, { 0x0d, 0xb7 }, { 0x0e, 0x01 }, { 0x4c, 0x00 }, { 0x4a, 0x81 },
    { 0x21, 0x99 }, { AEW, 0x40 }, { AEB, 0x38 }, { VV, VV_AGC_TH_SET(0x08, 0x02) }, { 0x5c, 0x00 },
    { 0x63, 0x00 }, { FLL, 0x22 }, { COM3, COM3_BAND_SET(COM3_BAND_AUTO) }, { REG5D, 0x55 },
    { REG5E, 0x7d }, { REG5F, 0x7d }, { REG60, 0x55 }, { HISTO_LOW, 0x70 }, { HISTO_HIGH, 0x80 },
    { 0x7c, 0x05 }, { 0x20, 0x80 }, { 0x28, 0x30 }, { 0x6c, 0x00 }, { 0x6d, 0x80 }, { 0x6e, 0x00 },
    { 0x70, 0x02 }, { 0x71, 0x94 }, { 0x73, 0xc1 }, { 0x3d, 0x34 },
    { COM7, COM7_RES_UXGA | COM7_ZOOM_EN }, { 0x5a, 0x57 }, { COM25, 0x00 }, { BD50, 0xbb },
    { BD60, 0x9c }, { BANK_SEL, BANK_SEL_DSP }, { 0xe5, 0x7f },
    { MC_BIST, MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL }, { 0x41, 0x24 },
    { RESET, RESET_JPEG | RESET_DVP }, { 0x76, 0xff }, { 0x33, 0xa0 }, { 0x42, 0x20 },
    { 0x43, 0x18 }, { 0x4c, 0x00 }, { CTRL3, CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 }, { 0x88, 0x3f },
    { 0xd7, 0x03 }, { 0xd9, 0x10 }, { R_DVP_SP, R_DVP_SP_AUTO_MODE | 0x2 }, { 0xc8, 0x08 },
    { 0xc9, 0x80 }, { BPADDR, 0x00 }, { BPDATA, 0x00 }, { BPADDR, 0x03 }, { BPDATA, 0x48 },
    { BPDATA, 0x48 }, { BPADDR, 0x08 }, { BPDATA, 0x20 }, { BPDATA, 0x10 }, { BPDATA, 0x0e },
    { 0x90, 0x00 }, { 0x91, 0x0e }, { 0x91, 0x1a }, { 0x91, 0x31 }, { 0x91, 0x5a }, { 0x91, 0x69 },
    { 0x91, 0x75 }, { 0x91, 0x7e }, { 0x91, 0x88 }, { 0x91, 0x8f }, { 0x91, 0x96 }, { 0x91, 0xa3 },
    { 0x91, 0xaf }, { 0x91, 0xc4 }, { 0x91, 0xd7 }, { 0x91, 0xe8 }, { 0x91, 0x20 }, { 0x92, 0x00 },
    { 0x93, 0x06 }, { 0x93, 0xe3 }, { 0x93, 0x03 }, { 0x93, 0x03 }, { 0x93, 0x00 }, { 0x93, 0x02 },
    { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 },
    { 0x93, 0x00 }, { 0x96, 0x00 }, { 0x97, 0x08 }, { 0x97, 0x19 }, { 0x97, 0x02 }, { 0x97, 0x0c },
    { 0x97, 0x24 }, { 0x97, 0x30 }, { 0x97, 0x28 }, { 0x97, 0x26 }, { 0x97, 0x02 }, { 0x97, 0x98 },
    { 0x97, 0x80 }, { 0x97, 0x00 }, { 0x97, 0x00 }, { 0xa4, 0x00 }, { 0xa8, 0x00 }, { 0xc5, 0x11 },
    { 0xc6, 0x51 }, { 0xbf, 0x80 }, { 0xc7, 0x10 }, /* simple AWB */
    { 0xb6, 0x66 }, { 0xb8, 0xA5 }, { 0xb7, 0x64 }, { 0xb9, 0x7C }, { 0xb3, 0xaf }, { 0xb4, 0x97 },
    { 0xb5, 0xFF }, { 0xb0, 0xC5 }, { 0xb1, 0x94 }, { 0xb2, 0x0f }, { 0xc4, 0x5c }, { 0xa6, 0x00 },
    { 0xa7, 0x20 }, { 0xa7, 0xd8 }, { 0xa7, 0x1b }, { 0xa7, 0x31 }, { 0xa7, 0x00 }, { 0xa7, 0x18 },
    { 0xa7, 0x20 }, { 0xa7, 0xd8 }, { 0xa7, 0x19 }, { 0xa7, 0x31 }, { 0xa7, 0x00 }, { 0xa7, 0x18 },
    { 0xa7, 0x20 }, { 0xa7, 0xd8 }, { 0xa7, 0x19 }, { 0xa7, 0x31 }, { 0xa7, 0x00 }, { 0xa7, 0x18 },
    { 0x7f, 0x00 }, { 0xe5, 0x1f }, { 0xe1, 0x77 }, { 0xdd, 0x7f },
    { CTRL0, CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },

    // OpenMV Custom.

    { BANK_SEL, BANK_SEL_SENSOR }, { 0x0f, 0x4b }, { COM1, 0x8f },

    // End.

    { 0x00, 0x00 }, /* END MARKER */
};

static const uint8_t cif_regs[][2] = {
    { BANK_SEL, BANK_SEL_SENSOR }, { COM7, COM7_RES_CIF }, { COM1, 0x06 | 0x80 }, { HSTART, 0x11 },
    { HSTOP, 0x43 }, { VSTART, 0x01 }, // 0x01 fixes issue with garbage pixels in the image...
    { VSTOP, 0x97 }, { REG32, 0x09 }, { BANK_SEL, BANK_SEL_DSP }, { RESET, RESET_DVP },
    { SIZEL,
        SIZEL_HSIZE8_11_SET(CIF_WIDTH) | SIZEL_HSIZE8_SET(CIF_WIDTH)
            | SIZEL_VSIZE8_SET(CIF_HEIGHT) },
    { HSIZE8, HSIZE8_SET(CIF_WIDTH) }, { VSIZE8, VSIZE8_SET(CIF_HEIGHT) },
    { CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
    { 0x00, 0x00 }, /* END MARKER */
};

static const uint8_t svga_regs[][2] = {
    { BANK_SEL, BANK_SEL_SENSOR }, { COM7, COM7_RES_SVGA }, { COM1, 0x0A | 0x80 }, { HSTART, 0x11 },
    { HSTOP, 0x43 }, { VSTART, 0x01 }, // 0x01 fixes issue with garbage pixels in the image...
    { VSTOP, 0x97 }, { REG32, 0x09 }, { BANK_SEL, BANK_SEL_DSP }, { RESET, RESET_DVP },
    { SIZEL,
        SIZEL_HSIZE8_11_SET(SVGA_WIDTH) | SIZEL_HSIZE8_SET(SVGA_WIDTH)
            | SIZEL_VSIZE8_SET(SVGA_HEIGHT) },
    { HSIZE8, HSIZE8_SET(SVGA_WIDTH) }, { VSIZE8, VSIZE8_SET(SVGA_HEIGHT) },
    { CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
    { 0x00, 0x00 }, /* END MARKER */
};

static const uint8_t uxga_regs[][2] = {
    { BANK_SEL, BANK_SEL_SENSOR },
    { COM7, COM7_RES_UXGA },
    { COM1, 0x0F | 0x80 },
    { HSTART, 0x11 },
    { HSTOP, 0x75 },
    { VSTART, 0x01 },
    { VSTOP, 0x97 },
    { REG32, 0x36 },
    { BANK_SEL, BANK_SEL_DSP },
    { RESET, RESET_DVP },
    { SIZEL,
        SIZEL_HSIZE8_11_SET(UXGA_WIDTH) | SIZEL_HSIZE8_SET(UXGA_WIDTH)
            | SIZEL_VSIZE8_SET(UXGA_HEIGHT) },
    { HSIZE8, HSIZE8_SET(UXGA_WIDTH) },
    { VSIZE8, VSIZE8_SET(UXGA_HEIGHT) },
    { CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
    { 0, 0 },
};

static const uint8_t yuv422_regs[][2] = {
    { BANK_SEL, BANK_SEL_DSP }, { R_BYPASS, R_BYPASS_DSP_EN }, { IMAGE_MODE, IMAGE_MODE_YUV422 },
    { 0xd7, 0x03 }, { 0x33, 0xa0 }, { 0xe5, 0x1f }, { 0xe1, 0x67 }, { RESET, 0x00 },
    { R_BYPASS, R_BYPASS_DSP_EN }, { 0x00, 0x00 }, /* END MARKER */
};

static const uint8_t rgb565_regs[][2] = {
    { BANK_SEL, BANK_SEL_DSP }, { R_BYPASS, R_BYPASS_DSP_EN }, { IMAGE_MODE, IMAGE_MODE_RGB565 },
    { 0xd7, 0x03 }, { RESET, 0x00 }, { R_BYPASS, R_BYPASS_DSP_EN }, { 0x00, 0x00 }, /* END MARKER */
};

static const uint8_t bayer_regs[][2] = {
    { BANK_SEL, BANK_SEL_DSP }, { R_BYPASS, R_BYPASS_DSP_EN }, { IMAGE_MODE, IMAGE_MODE_RAW10 },
    { 0xd7, 0x03 }, { RESET, 0x00 }, { R_BYPASS, R_BYPASS_DSP_EN }, { 0x00, 0x00 }, /* END MARKER */
};

#define NUM_BRIGHTNESS_LEVELS (5)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA }, { 0x00, 0x04, 0x09, 0x00, 0x00 }, /* -2 */
    { 0x00, 0x04, 0x09, 0x10, 0x00 }, /* -1 */
    { 0x00, 0x04, 0x09, 0x20, 0x00 }, /*  0 */
    { 0x00, 0x04, 0x09, 0x30, 0x00 }, /* +1 */
    { 0x00, 0x04, 0x09, 0x40, 0x00 }, /* +2 */
};

#define NUM_CONTRAST_LEVELS (5)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS + 1][7] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA, BPDATA, BPDATA },
    { 0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06 }, /* -2 */
    { 0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06 }, /* -1 */
    { 0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06 }, /*  0 */
    { 0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06 }, /* +1 */
    { 0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06 }, /* +2 */
};

#define NUM_SATURATION_LEVELS (5)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS + 1][5] = {
    { BPADDR, BPDATA, BPADDR, BPDATA, BPDATA }, { 0x00, 0x02, 0x03, 0x28, 0x28 }, /* -2 */
    { 0x00, 0x02, 0x03, 0x38, 0x38 }, /* -1 */
    { 0x00, 0x02, 0x03, 0x48, 0x48 }, /*  0 */
    { 0x00, 0x02, 0x03, 0x58, 0x58 }, /* +1 */
    { 0x00, 0x02, 0x03, 0x68, 0x68 }, /* +2 */
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
    unsigned int i, k;
    unsigned char buf[64] = { 0 };
    unsigned char* ptr = buf;
    const char* banks[2] = { "DSP", "SENSOR" };
    unsigned char banks_select[2] = { BANK_SEL_DSP, BANK_SEL_SENSOR };

    for (k = 0; k < 2; k++) {
        printf("SECTION: %s\n", banks[k]);
        cambus_writeb(BANK_SEL, banks_select[k]);

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
    int ret = 0;

    ret |= cambus_writeb(BANK_SEL, BANK_SEL_SENSOR);
    ret |= cambus_writeb(COM7, COM7_SRST);

    utils_delay_ms(10);

    // Write default registers
    for (int i = 0; default_regs[i][0]; i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }

    ret |= cambus_writeb(CLKRC, 8); // InternalClk = InputClk / (2*(2+1))

    return ret;
}

static int sleep(int enable)
{
    int ret = 0;

    return ret;
}

static int read_reg(uint16_t reg_addr)
{
    uint8_t reg = 0;

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
    const uint8_t(*regs)[2];

    switch (pixformat) {
    case PIXFORMAT_RGB565:
        regs = rgb565_regs;
        break;

    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        regs = yuv422_regs;
        break;

    case PIXFORMAT_BAYER:
        regs = bayer_regs;
        break;

    default:
        return -1;
    }

    // Write registers
    for (int i = 0; regs[i][0]; i++) { ret |= cambus_writeb(regs[i][0], regs[i][1]); }

    utils_delay_ms(50);

    return ret;
}

extern const int resolution[][2];

static int set_framesize(framesize_t framesize)
{
    const uint8_t(*regs)[2];
    uint16_t sensor_w = 0;
    uint16_t sensor_h = 0;
    int ret = 0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];

    if ((w % 4) || (h % 4)) { // w/h must be divisble by 4
        return 1;
    }

    // Looks really bad.
    if ((w <= CIF_WIDTH) && (h <= CIF_HEIGHT)) {
        regs = cif_regs;
        sensor_w = CIF_WIDTH;
        sensor_h = CIF_HEIGHT;
    } else if ((w <= SVGA_WIDTH) && (h <= SVGA_HEIGHT)) {
        regs = svga_regs;
        sensor_w = SVGA_WIDTH;
        sensor_h = SVGA_HEIGHT;
    } else {
        regs = uxga_regs;
        sensor_w = UXGA_WIDTH;
        sensor_h = UXGA_HEIGHT;
    }

    // Write setup regsiters
    for (int i = 0; regs[i][0]; i++) { ret |= cambus_writeb(regs[i][0], regs[i][1]); }

    uint64_t tmp_div = IM_MIN(sensor_w / w, sensor_h / h);
    uint16_t log_div = IM_MIN(IM_LOG2(tmp_div) - 1, 3);
    uint16_t div = 1 << log_div;
    uint16_t w_mul = w * div;
    uint16_t h_mul = h * div;
    uint16_t x_off = (sensor_w - w_mul) / 2;
    uint16_t y_off = (sensor_h - h_mul) / 2;

    ret |= cambus_writeb(CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(log_div) | CTRLI_H_DIV_SET(log_div));
    ret |= cambus_writeb(HSIZE, HSIZE_SET(w_mul));
    ret |= cambus_writeb(VSIZE, VSIZE_SET(h_mul));
    ret |= cambus_writeb(XOFFL, XOFFL_SET(x_off));
    ret |= cambus_writeb(YOFFL, YOFFL_SET(y_off));
    ret |= cambus_writeb(VHYX,
        VHYX_HSIZE_SET(w_mul) | VHYX_VSIZE_SET(h_mul) | VHYX_XOFF_SET(x_off)
            | VHYX_YOFF_SET(y_off));
    ret |= cambus_writeb(TEST, TEST_HSIZE_SET(w_mul));
    ret |= cambus_writeb(ZMOW, ZMOW_OUTW_SET(w));
    ret |= cambus_writeb(ZMOH, ZMOH_OUTH_SET(h));
    ret |= cambus_writeb(ZMHH, ZMHH_OUTW_SET(w) | ZMHH_OUTH_SET(h));
    ret |= cambus_writeb(R_DVP_SP, div);
    ret |= cambus_writeb(RESET, 0x00);

    // Delay 50 ms
    utils_delay_ms(50);

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;

    level += (NUM_CONTRAST_LEVELS / 2) + 1;

    if (level < 0 || level > NUM_CONTRAST_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cambus_writeb(BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (int i = 0; i < sizeof(contrast_regs[0]) / sizeof(contrast_regs[0][0]); i++) {
        ret |= cambus_writeb(contrast_regs[0][i], contrast_regs[level][i]);
    }

    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;

    level += (NUM_BRIGHTNESS_LEVELS / 2) + 1;

    if (level < 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cambus_writeb(BANK_SEL, BANK_SEL_DSP);

    /* Write brightness registers */
    for (int i = 0; i < sizeof(brightness_regs[0]) / sizeof(brightness_regs[0][0]); i++) {
        ret |= cambus_writeb(brightness_regs[0][i], brightness_regs[level][i]);
    }

    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;

    level += (NUM_SATURATION_LEVELS / 2) + 1;

    if (level < 0 || level > NUM_SATURATION_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cambus_writeb(BANK_SEL, BANK_SEL_DSP);

    /* Write saturation registers */
    for (int i = 0; i < sizeof(saturation_regs[0]) / sizeof(saturation_regs[0][0]); i++) {
        ret |= cambus_writeb(saturation_regs[0][i], saturation_regs[level][i]);
    }

    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;

    /* Switch to SENSOR register bank */
    ret |= cambus_writeb(BANK_SEL, BANK_SEL_SENSOR);

    /* Write gain ceiling register */
    ret |= cambus_writeb(COM9, COM9_AGC_SET(gainceiling));

    return ret;
}

static int set_colorbar(int enable)
{
    uint8_t reg = 0;
    int ret = 0;

    ret |= cambus_writeb(BANK_SEL, BANK_SEL_SENSOR);
    ret |= cambus_readb(COM7, &reg);

    if (enable) {
        reg |= COM7_COLOR_BAR;
    } else {
        reg &= ~COM7_COLOR_BAR;
    }

    return cambus_writeb(COM7, reg) | ret;
}

static int set_hmirror(int enable)
{
    uint8_t reg = 0;
    int ret = 0;

    ret |= cambus_writeb(BANK_SEL, BANK_SEL_SENSOR);
    ret |= cambus_readb(REG04, &reg);

    if (!enable) { // Already mirrored.
        reg |= REG04_HFLIP_IMG;
    } else {
        reg &= ~REG04_HFLIP_IMG;
    }

    return cambus_writeb(REG04, reg) | ret;
}

static int set_vflip(int enable)
{
    uint8_t reg = 0;
    int ret = 0;

    ret |= cambus_writeb(BANK_SEL, BANK_SEL_SENSOR);
    ret |= cambus_readb(REG04, &reg);

    if (!enable) { // Already flipped.
        reg |= REG04_VFLIP_IMG | REG04_VREF_EN;
    } else {
        reg &= ~(REG04_VFLIP_IMG | REG04_VREF_EN);
    }

    return cambus_writeb(REG04, reg) | ret;
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

#endif // (ACTIVE_CAMERA == CAM_OV2640)
