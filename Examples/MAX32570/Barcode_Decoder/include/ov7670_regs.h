/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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

#if (ACTIVE_CAMERA == CAM_OV7670)


/* OV7670 Registers definition */
#define REG_GAIN                    0x00 /* AGC - Gain control gain setting  */
#define REG_BLUE                    0x01 /* AWB - Blue channel gain setting  */
#define REG_RED                     0x02 /* AWB - Red channel gain setting   */
#define REG_VREF                    0x03 /* Vertical frame control */
#define REG_COM1                    0x04 /* Common Control 1 */
#define REG_BAVE                    0x05 /* U/B Average Level  */
#define REG_GbAVE                   0x06 /* Y/Gb Average Level */
#define REG_AECHH                   0x07 /* Exposure Value - AEC MSB 5 bits */
#define REG_RAVE                    0x08 /* V/R Average Level */
#define REG_COM2                    0x09 /* Common Control 2 */
#define REG_PID                     0x0A /* Product ID Number MSB, (Read Only) */
#define REG_VER                     0x0B /* Product ID Number LSB, (Read Only) */
#define REG_COM3                    0x0C /* Common Control 3 */
#define REG_COM4                    0x0D /* Common Control 4 */
#define REG_COM5                    0x0E /* Common Control 5 */
#define REG_COM6                    0x0F /* Common Control 6 */
#define REG_AECH                    0x10 /* Exposure Value */
#define REG_CLKRC                   0x11 /* Internal Clock */
#define REG_COM7                    0x12 /* Common Control 7 */
#define REG_COM8                    0x13 /* Common Control 8 */
#define REG_COM9                    0x14 /* Common Control 9 */
#define REG_COM10                   0x15 /* Common Control 10 */
//#define REG_RSVD       0x16
#define REG_HSTART                  0x17
#define REG_HSTOP                   0x18
#define REG_VSTRT                   0x19
#define REG_VSTOP                   0x1A
#define REG_PSHFT                   0x1B
#define REG_MIDH                    0x1C
#define REG_MIDL                    0x1D
#define REG_MVFP                    0x1E
#define REG_LAEC                    0x1F
#define REG_ADCCTR0                 0x20
#define REG_ADCCTR1                 0x21
#define REG_ADCCTR2                 0x22
#define REG_ADCCTR3                 0x23
#define REG_AEW                     0x24
#define REG_AEB                     0x25
#define REG_VPT                     0x26
#define REG_BBIAS                   0x27
#define REG_GbBIAS                  0x28
//#define REG_RSVD       0x29
#define REG_EXHCH                   0x2A
#define REG_EXHCL                   0x2B
#define REG_RBIAS                   0x2C
#define REG_ADVFL                   0x2D
#define REG_ADVFH                   0x2E
#define REG_YAVE                    0x2F
#define REG_HSYST                   0x30
#define REG_HSYEN                   0x31
#define REG_HREF                    0x32
#define REG_CHLF                    0x33
#define REG_ARBLM                   0x34
//#define REG_RSVD       0x35
//#define REG_RSVD       0x36
#define REG_ADC                     0x37
#define REG_ACOM                    0x38
#define REG_OFON                    0x39
#define REG_TSLB                    0x3A
#define REG_COM11                   0x3B
#define REG_COM12                   0x3C
#define REG_COM13                   0x3D
#define REG_COM14                   0x3E
#define REG_EDGE                    0x3F
#define REG_COM15                   0x40
#define REG_COM16                   0x41
#define REG_COM17                   0x42
#define REG_AWBC1                   0x43
#define REG_AWBC2                   0x44
#define REG_AWBC3                   0x45
#define REG_AWBC4                   0x46
#define REG_AWBC5                   0x47
#define REG_AWBC6                   0x48
//#define REG_RSVD       0x49
//#define REG_RSVD                  0x4A
#define REG_REG4B                   0x4B
#define REG_DNSTH                   0x4C
//#define REG_RSVD       0x4D
//#define REG_RSVD       0x4E
#define REG_MTX1                    0x4F
#define REG_MTX2                    0x50
#define REG_MTX3                    0x51
#define REG_MTX4                    0x52
#define REG_MTX5                    0x53
#define REG_MTX6                    0x54
#define REG_BRIGHT                  0x55
#define REG_CONTRAS                 0x56
#define REG_CONTRASCENTER           0x57
#define REG_MTXS                    0x58
//#define REG_RSVD                  0x59
//#define REG_RSVD                  0x5A
//#define REG_RSVD                  0x5B
//#define REG_RSVD                  0x5C
//#define REG_RSVD                  0x5D
//#define REG_RSVD                  0x5E
//#define REG_RSVD                  0x5F
//#define REG_RSVD                  0x60
//#define REG_RSVD                  0x61
#define REG_LCC1                    0x62
#define REG_LCC2                    0x63
#define REG_LCC3                    0x64
#define REG_LCC4                    0x65
#define REG_LCC5                    0x66
#define REG_MANU                    0x67
#define REG_MANV                    0x68
#define REG_GFIX                    0x69
#define REG_GGAIN                   0x6A
#define REG_DBLV                    0x6B
#define REG_AWBCTR3                 0x6C
#define REG_AWBCTR2                 0x6D
#define REG_AWBCTR1                 0x6E
#define REG_AWBCTR0                 0x6F
#define REG_SCALING_XSC             0x70
#define REG_SCALING_YSC             0x71
#define REG_SCALING_DCWCTR          0x72
#define REG_SCALING_PCLK_DIV        0x73
#define REG_REG74                   0x74
#define REG_REG75                   0x75
#define REG_REG76                   0x76
#define REG_REG77                   0x77
//#define REG_RSVD       0x78
//#define REG_RSVD       0x79
#define REG_SLOP                    0x7A
#define REG_GAM1                    0x7B
#define REG_GAM2                    0x7C
#define REG_GAM3                    0x7D
#define REG_GAM4                    0x7E
#define REG_GAM5                    0x7F
#define REG_GAM6                    0x80
#define REG_GAM7                    0x81
#define REG_GAM8                    0x82
#define REG_GAM9                    0x83
#define REG_GAM10                   0x84
#define REG_GAM11                   0x85
#define REG_GAM12                   0x86
#define REG_GAM13                   0x87
#define REG_GAM14                   0x88
#define REG_GAM15                   0x89
//#define REG_RSVD       0x8A
//#define REG_RSVD       0x8B
#define REG_RGB444                  0x8C
//#define REG_RSVD       0x8D
//#define REG_RSVD       0x8E
//#define REG_RSVD       0x8F
//#define REG_RSVD       0x90
//#define REG_RSVD       0x91
#define REG_DM_LNL                  0x92
#define REG_DM_LNH                  0x93
#define REG_LCC6                    0x94
#define REG_LCC7                    0x95
//#define REG_RSVD       0x96
//#define REG_RSVD       0x97
//#define REG_RSVD       0x98
//#define REG_RSVD       0x99
//#define REG_RSVD       0x9A
//#define REG_RSVD       0x9B
//#define REG_RSVD       0x9C
#define REG_BD50ST                  0x9D
#define REG_BD60ST                  0x9E
#define REG_HAECC1                  0x9F
#define REG_HAECC2                  0xA0
//#define REG_RSVD       0xA1
#define REG_SCALING_PCLK_DELAY      0xA2
//#define REG_RSVD       0xA3
#define REG_NT_CTRL                 0xA4
#define REG_BD50MAX                 0xA5

#define REG_HAECC3                  0xA6
#define REG_HAECC4                  0xA7
#define REG_HAECC5                  0xA8
#define REG_HAECC6                  0xA9
#define REG_HAECC7                  0xAA
#define REG_BD60MAX                 0xAB
#define REG_STR_OPT                 0xAC
#define REG_STR_R                   0xAD
#define REG_STR_G                   0xAE
#define REG_STR_B                   0xAF
//#define REG_RSVD       0xB0
#define REG_ABLC1                   0xB1
//#define REG_RSVD       0xB2
#define REG_THL_ST                  0xB3
//#define REG_RSVD       0xB4
#define REG_THL_DLT                 0xB5
//#define REG_RSVD       0xB6
//#define REG_RSVD       0xB7
//#define REG_RSVD       0xB8
//#define REG_RSVD       0xBC
//#define REG_RSVD       0xBD
#define REG_AD_CHB                  0xBE
#define REG_AD_CHR                  0xBF
#define REG_AD_CHGb                 0xC0
#define REG_AD_CHGr                 0xC1
//#define REG_RSVD       0xC2
//#define REG_RSVD       0xC3
//#define REG_RSVD       0xC4
//#define REG_RSVD       0xC5
//#define REG_RSVD       0xC6
//#define REG_RSVD       0xC7
//#define REG_RSVD       0xC8
#define REG_SATCTR                  0xC9


/* Registers bit definition */
/* COM1 Register */
#define CCIR656_FORMAT  0x40        /* CCIR656 enable */
#define HREF_SKIP_0     0x00
#define HREF_SKIP_1     0x04
#define HREF_SKIP_3     0x08

/* COM2 Register */
#define COM2_SOFT_SLEEP_MODE  0x10
#define COM2_ODCAP_1x         0x00
#define COM2_ODCAP_2x         0x01
#define COM2_ODCAP_3x         0x02
#define COM2_ODCAP_4x         0x03

/* COM3 Register */
#define COM3_COLOR_BAR_OUTPUT         0x80
#define COM3_OUTPUT_MSB_LAS_SWAP      0x40
#define COM3_PIN_REMAP_RESETB_EXPST   0x08
#define COM3_RGB565_FORMAT            0x00
#define COM3_RGB_OUTPUT_AVERAGE       0x04
#define COM3_SINGLE_FRAME             0x01
#define COM3_SET_MIRROR(r, x)         ((r&0xBF)|((x&1)<<6))
#define COM3_GET_MIRROR(r)            ((r>>6)&1)

/* COM5 Register */
#define SLAM_MODE_ENABLE      0x40
#define EXPOSURE_NORMAL_MODE  0x01

/* COM7 Register */
#define SCCB_REG_RESET                       0x80
#define FORMAT_CTRL_15fpsVGA                 0x00
#define FORMAT_CTRL_30fpsVGA_NoVArioPixel    0x50
#define FORMAT_CTRL_30fpsVGA_VArioPixel      0x60
#define OUTPUT_FORMAT_RAWRGB                 0x00
#define OUTPUT_FORMAT_RAWRGB_DATA            0x00
#define OUTPUT_FORMAT_RAWRGB_INTERP          0x01
#define OUTPUT_FORMAT_YUV                    0x02
#define OUTPUT_FORMAT_RGB                    0x03

/* COM9 Register */
#define COM9_GAIN_2x         0x00
#define COM9_GAIN_4x         0x10
#define COM9_GAIN_8x         0x20
#define COM9_GAIN_16x        0x30
#define COM9_GAIN_32x        0x40
#define COM9_GAIN_64x        0x50
#define COM9_GAIN_128x       0x60
#define COM9_DROP_VSYNC      0x04
#define COM9_DROP_HREF       0x02
#define COM9_SET_AGC(r, x)   ((r&0x8F)|((x&0x07)<<4))

/* COM10 Register */
#define RESETb_REMAP_SLHS    0x80
#define HREF_CHANGE_HSYNC    0x40
#define PCLK_ON              0x00
#define PCLK_OFF             0x20
#define PCLK_POLARITY_REV    0x10
#define HREF_POLARITY_REV    0x08
#define RESET_ENDPOINT       0x04
#define VSYNC_NEG            0x02
#define HSYNC_NEG            0x01

/* TSLB Register */
#define PCLK_DELAY_0         0x00
#define PCLK_DELAY_2         0x40
#define PCLK_DELAY_4         0x80
#define PCLK_DELAY_6         0xC0
#define OUTPUT_BITWISE_REV   0x20
#define UV_NORMAL            0x00
#define UV_FIXED             0x10
#define YUV_SEQ_YUYV         0x00
#define YUV_SEQ_YVYU         0x02
#define YUV_SEQ_VYUY         0x04
#define YUV_SEQ_UYVY         0x06
#define BANDING_FREQ_50      0x02

#define RGB_NORMAL   0x00
#define RGB_565      0x10
#define RGB_555      0x30


#define COM1_CCIR656        0x40    /* CCIR656 enable */
#define COM2_SSLEEP         0x10    /* Soft sleep mode */
#define COM3_SWAP           0x40    /* Byte swap */
#define COM3_SCALEEN        0x08    /* Enable scaling */
#define COM3_DCWEN          0x04    /* Enable downsamp/crop/window */
#define CLK_EXT             0x40    /* Use external clock directly */
#define CLK_SCALE           0x3f    /* Mask for internal clock scale */
#define COM7_RESET          0x80    /* Register reset */
#define COM7_FMT_MASK       0x38
#define COM7_FMT_VGA        0x00
#define COM7_FMT_CIF        0x20    /* CIF format */
#define COM7_FMT_QVGA       0x10    /* QVGA format */
#define COM7_FMT_QCIF       0x08    /* QCIF format */
#define COM7_RGB            0x04    /* bits 0 and 2 - RGB format */
#define COM7_YUV            0x00    /* YUV */
#define COM7_BAYER          0x01    /* Bayer format */
#define COM7_PBAYER         0x05    /* "Processed bayer" */
#define COM8_FASTAEC        0x80    /* Enable fast AGC/AEC */
#define COM8_AECSTEP        0x40    /* Unlimited AEC step size */
#define COM8_BFILT          0x20    /* Band filter enable */
#define COM8_AGC            0x04    /* Auto gain enable */
#define COM8_AWB            0x02    /* White balance enable */
#define COM8_AEC            0x01    /* Auto exposure enable */
#define COM10_HSYNC         0x40    /* HSYNC instead of HREF */
#define COM10_PCLK_HB       0x20    /* Suppress PCLK on horiz blank */
#define COM10_HREF_REV      0x08    /* Reverse HREF */
#define COM10_VS_LEAD       0x04    /* VSYNC on clock leading edge */
#define COM10_VS_NEG        0x02    /* VSYNC negative */
#define COM10_HS_NEG        0x01    /* HSYNC negative */
#define MVFP_MIRROR         0x20    /* Mirror image */
#define MVFP_FLIP           0x10    /* Vertical flip */

#define TSLB_YLAST          0x04    /* UYVY or VYUY - see com13 */
#define COM11_NIGHT         0x80    /* NIght mode enable */
#define COM11_NMFR          0x60    /* Two bit NM frame rate */
#define COM11_HZAUTO        0x10    /* Auto detect 50/60 Hz */
#define COM11_50HZ          0x08    /* Manual 50Hz select */
#define COM11_EXP           0x02
#define COM12_HREF          0x80    /* HREF always */
#define COM13_GAMMA         0x80    /* Gamma enable */
#define COM13_UVSAT         0x40    /* UV saturation auto adjustment */
#define COM13_UVSWAP        0x01    /* V before U - w/TSLB */
#define COM14_DCWEN         0x10    /* DCW/PCLK-scale enable */
#define COM15_R10F0         0x00    /* Data range 10 to F0 */
#define COM15_R01FE         0x80    /*          01 to FE */
#define COM15_R00FF         0xc0    /*          00 to FF */
#define COM15_RGB565        0x10    /* RGB565 output */
#define COM15_RGB555        0x30    /* RGB555 output */
#define COM16_AWBGAIN       0x08    /* AWB gain enable */
#define COM17_AECWIN        0xc0    /* AEC window - must match COM4 */
#define COM17_CBAR          0x08    /* DSP Color bar */
/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 * They are nine-bit signed quantities, with the sign bit
 * stored in0x58.Sign for v-red is bit 0, and up from there.
 */
#define CMATRIX_LEN         6
#define R76_BLKPCOR         0x80    /* Black pixel correction enable */
#define R76_WHTPCOR         0x40    /* White pixel correction enable */
#define R444_ENABLE         0x02    /* Turn on RGB444, overrides 5x5 */
#define R444_RGBX           0x01    /* Empty nibble at end */
#define COM7_FMT_CIF        0x20    /* CIF format */
#define COM7_RGB            0x04    /* bits 0 and 2 - RGB format */
#define COM7_YUV            0x00    /* YUV */
#define COM7_BAYER          0x01    /* Bayer format */
#define COM7_PBAYER         0x05    /* "Processed bayer" */
#define COM10_VS_LEAD       0x04    /* VSYNC on clock leading edge */
#define COM11_50HZ          0x08    /* Manual 50Hz select */
#define COM13_UVSAT         0x40    /* UV saturation auto adjustment */
#define COM15_R01FE         0x80    /*          01 to FE */
#define MTX1                0x4f    /* Matrix Coefficient 1 */
#define MTX2                0x50    /* Matrix Coefficient 2 */
#define MTX3                0x51    /* Matrix Coefficient 3 */
#define MTX4                0x52    /* Matrix Coefficient 4 */
#define MTX5                0x53    /* Matrix Coefficient 5 */
#define MTX6                0x54    /* Matrix Coefficient 6 */
#define MTXS                0x58    /* Matrix Coefficient Sign */
#define AWBC7               0x59    /* AWB Control 7 */
#define AWBC8               0x5a    /* AWB Control 8 */
#define AWBC9               0x5b    /* AWB Control 9 */
#define AWBC10              0x5c    /* AWB Control 10 */
#define AWBC11              0x5d    /* AWB Control 11 */
#define AWBC12              0x5e    /* AWB Control 12 */
#define GGAIN               0x6a    /* G Channel AWB Gain */
#define DBLV                0x6b
#define AWBCTR3             0x6c    /* AWB Control 3 */
#define AWBCTR2             0x6d    /* AWB Control 2 */
#define AWBCTR1             0x6e    /* AWB Control 1 */
#define AWBCTR0             0x6f    /* AWB Control 0 */


#endif //__REG_REGS_H__
