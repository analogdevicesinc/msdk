/*******************************************************************************
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
******************************************************************************/

#ifndef LIBRARIES_MISCDRIVERS_CAMERA_OV5640_REGS_H_
#define LIBRARIES_MISCDRIVERS_CAMERA_OV5640_REGS_H_

// clang-format off
#define OV5640_I2C_SLAVE_ADDR   0x3C

#define SENSOR_IMAGER_X     (2592)
#define SENSOR_IMAGER_Y     (1944)

#define SENSOR_IMAGER_MID_X (SENSOR_IMAGER_X / 2)
#define SENSOR_IMAGER_MID_Y (SENSOR_IMAGER_Y / 2)

#define ISP_WIDTH           (1900)  // 1000
#define ISP_HEIGHT          (1900)  // 1000

#define X_ADDR_START (SENSOR_IMAGER_MID_X - (ISP_WIDTH / 2))
#define Y_ADDR_START (SENSOR_IMAGER_MID_Y - (ISP_HEIGHT / 2))
#define X_ADDR_END   (SENSOR_IMAGER_MID_X + (ISP_WIDTH / 2))
#define Y_ADDR_END   (SENSOR_IMAGER_MID_Y + (ISP_HEIGHT / 2))

/***** Register address definitions. *****/

/*** System Control Registers ***/
#define SYS_RST0                (0x3000)
#define SYS_RST1                (0x3001)
#define SYS_RST2                (0x3002)
#define SYS_RST3                (0x3003)

#define CLK_ENABLE_0            (0x3004)
#define CLK_ENABLE_1            (0x3005)
#define CLK_ENABLE_2            (0x3006)
#define CLK_ENABLE_3            (0x3007)

#define SYS_CTRL0               (0x3008)
#define CHIPID_H                (0x300a)
#define CHIPID_L                (0x300b)

#define MIPI_CTRL0              (0x300e)

#define PAD_OUT_ENABLE_0        (0x3016)
#define PAD_OUT_ENABLE_1        (0x3017)
#define PAD_OUT_ENABLE_2        (0x3018)
#define PAD_OUT_VALUE_0         (0x3019)
#define PAD_OUT_VALUE_1         (0x301a)
#define PAD_OUT_VALUE_2         (0x301b)

#define SC_PLL_CTRL0            (0x3034)
#define SC_PLL_CTRL1            (0x3035)
#define SC_PLL_CTRL2            (0x3036)
#define SC_PLL_CTRL3            (0x3037)

#define SCCB_CTRL0              (0x3102)
#define SCCB_CTRL1              (0x3103)
#define SCCB_PAD_CLK_DIV        (0x3108)

/*** VCM Control Registers ***/
#define VCM_0                   (0x3802)
#define VCM_1                   (0x3603)
#define VCM_SLEW_0              (0x3604)
#define VCM_SLEW_1              (0x3605)
#define VCM_CURRENT             (0x3606)

/*** Image Windowing Registers ***/
#define TIMING_HS_0             (0x3800)
#define TIMING_HS_1             (0x3801)
#define TIMING_VS_0             (0x3802)
#define TIMING_VS_1             (0x3803)
#define TIMING_HW_0             (0x3804)
#define TIMING_HW_1             (0x3805)
#define TIMING_VH_0             (0x3806)
#define TIMING_VH_1             (0x3807)
#define TIMING_DVPHO_0          (0x3808)
#define TIMING_DVPHO_1          (0x3809)
#define TIMING_DVPVO_0          (0x380a)
#define TIMING_DVPVO_1          (0x380b)
#define TIMING_HTS_0            (0x380c)
#define TIMING_HTS_1            (0x380d)
#define TIMING_VTS_0            (0x380e)
#define TIMING_VTS_1            (0x380f)
#define TIMING_HOFFSET_0        (0x3810)
#define TIMING_HOFFSET_1        (0x3811)
#define TIMING_VOFFSET_0        (0x3812)
#define TIMING_VOFFSET_1        (0x3813)
#define TIMING_TC_REG20         (0x3820)
#define TIMING_TC_REG21         (0x3821)

/*** AEC/AGC Power Down Domain Control Registers ***/
#define AEC_MAX_EXPO_60HZ_0     (0x3a02)
#define AEC_MAX_EXPO_60HZ_1     (0x3a03)
#define AEC_B50_STEP_0          (0x3a08)
#define AEC_B50_STEP_1          (0x3a09)
#define AEC_B60_STEP_0          (0x3a0a)
#define AEC_B60_STEP_1          (0x3a0b)
#define AEC_CTRL0Ch             (0x3a0c)
#define AEC_CTRL0Dh             (0x3a0d)
#define AEC_CTRL0Eh             (0x3a0e)
#define AEC_CTRL0Fh             (0x3a0f)
#define AEC_CTRL10h             (0x3a10)
#define AEC_CTRL11h             (0x3a11)
#define AEC_CTRL12h             (0x3a12)
#define AEC_CTRL13h             (0x3a13)
#define AEC_MAX_EXPO_50HZ_0     (0x3a14)
#define AEC_MAX_EXPO_50HZ_1     (0x3a15)
#define AEC_GAIN_CEILING_0      (0x3a18)
#define AEC_GAIN_CEILING_1      (0x3a19)

/*** 50/60Hz Detector Control Registers ***/
#define CTRL0_5060HZ            (0x3c00)
#define CTRL1_5060HZ            (0x3c01)
#define CTRL2_5060HZ            (0x3c02)
#define CTRL3_5060HZ            (0x3c03)
#define CTRL4_5060HZ            (0x3c04)
#define CTRL5_5060HZ            (0x3c05)
#define LIGHT_METER1_THD_0      (0x3c06)
#define LIGHT_METER1_THD_1      (0x3c07)
#define LIGHT_METER2_THD_0      (0x3c08)
#define LIGHT_METER2_THD_1      (0x3c09)
#define SAMPLE_NUM_0            (0x3c0a)
#define SAMPLE_NUM_1            (0x3c0b)

/*** BLC Control Registers ***/
#define BLC_CTRL00h             (0x4000)
#define BLC_CTRL01h             (0x4001)
#define BLC_CTRL02h             (0x4002)
#define BLC_CTRL03h             (0x4003)
#define BLC_CTRL04h             (0x4004)

/*** Format Control Registers ***/
#define FORMAT_CTRL00h          (0x4300)
#define FORMAT_CTRL01h          (0x4301)
#define FORMAT_YMAX_0           (0x4302)
#define FORMAT_YMAX_1           (0x4303)
#define FORMAT_YMIN_0           (0x4304)
#define FORMAT_YMIN_1           (0x4305)
#define FORMAT_UMAX_0           (0x4306)
#define FORMAT_UMAX_1           (0x4307)
#define FORMAT_UMIN_0           (0x4308)
#define FORMAT_UMIN_1           (0x4309)
#define FORMAT_VMAX_0           (0x430a)
#define FORMAT_VMAX_1           (0x430b)
#define FORMAT_VMIN_0           (0x430c)
#define FORMAT_VMIN_1           (0x430d)

/*** JPEG Registers ***/
#define JPEG_CTRL00h            (0x4400)
#define JPEG_CTRL01h            (0x4401)
#define JPEG_CTRL02h            (0x4402)
#define JPEG_CTRL03h            (0x4403)
#define JPEG_CTRL04h            (0x4404)
#define JPEG_CTRL05h            (0x4405)
#define JPEG_CTRL06h            (0x4406)
#define JPEG_CTRL07h            (0x4407)
#define JPEG_ISI_CTRL           (0x4408)
#define JPEG_CTRL09h            (0x4409)
#define JPEG_CTRL0Ah            (0x440a)
#define JPEG_CTRL0Bh            (0x440b)
#define JPEG_CTRL0Ch            (0x440c)
#define JPEG_QT_DATA            (0x4410)
#define JPEG_QT_ADDR            (0x4411)
#define JPEG_ISI_DATA           (0x4412)
#define JPEG_LENGTH_0           (0x4414)
#define JPEG_LENGTH_1           (0x4415)
#define JPEG_LENGTH_2           (0x4416)
#define JPEG_OVERFLOW           (0x4417)
#define JPEG_COMMENT            (0x4420)  // 0x4420 to 0x442F
#define JPEG_COMMENT_LENGTH     (0x4430)
#define JPEG_COMMENT_MARKER     (0x4431)

/*** VFIFO Registers ***/
#define VFIFO_CTRL00h           (0x4600)
#define VFIFO_HSIZE_0           (0x4602)
#define VFIFO_HSIZE_1           (0x4603)
#define VFIFO_VSIZE_0           (0x4604)
#define VFIFO_VSIZE_1           (0x4605)
#define VFIFO_CTRL0Ch           (0x460c)
#define VFIFO_CTRL0Dh           (0x460d)

/*** DVP Control Registers ***/
#define DVP_VSYNC_WIDTH0        (0x4709)
#define DVP_VSYNC_WIDTH1        (0x470a)
#define DVP_VSYNC_WITDH2        (0x470b)
#define DVP_PAD_LEFT_CTRL       (0x4711)
#define DVP_PAD_RIGHT_CTRL      (0x4712)
#define DVP_JPG_MODE_SEL        (0x4713)
#define DVP_656_DUMMY_LINE      (0x4715)
#define DVP_CCIR656_CTRL        (0x4719)
#define DVP_HSYNC_CTRL          (0x471b)
#define DVP_VSYNC_CTRL          (0x471d)
#define DVP_HREF_CTRL           (0x471f)
#define DVP_VERT_START_OFFSET   (0x4721)
#define DVP_VERT_END_OFFSET     (0x4722)
#define DVP_CTRL23h             (0x4723)
#define DVP_CCIR656_CTRL00h     (0x4730)
#define DVP_CCIR656_CTRL01h     (0x4731)
#define DVP_CCIR656_FS          (0x4732)
#define DVP_CCIR656_FE          (0x4733)
#define DVP_CCIR656_LS          (0x4734)
#define DVP_CCIR656_LE          (0x4735)
#define DVP_CCIR656_CTRL06h     (0x4736)
#define DVP_CCIR656_CTRL07h     (0x4737)
#define DVP_CCIR656_CTRL08h     (0x4738)
#define DVP_POLARITY_CTRL00h    (0x4740)
#define DVP_TEST_PATTERN        (0x4741)
#define DVP_DATA_ORDER          (0x4745)

/*** MIPI Transmitter Registers ***/
#define MIPI_CTRL00h            (0x4800)
#define MIPI_CTRL01h            (0x4801)
#define MIPI_CTRL05h            (0x4805)
#define MIPI_DATA_ORDER         (0x480a)
#define MIPI_MIN_HS_ZERO_H      (0x4818)
#define MIPI_MIN_HS_ZERO_L      (0x4819)
#define MIPI_MIN_HS_TRAIL_H     (0x481a)
#define MIPI_MIN_HS_TRAIL_L     (0x481b)
#define MIPI_MIN_CLK_ZERO_H     (0x481c)
#define MIPI_MIN_CLK_ZERO_L     (0x481d)
#define MIPI_MIN_CLK_PREP_H     (0x481e)
#define MIPI_MIN_CLK_PREP_L     (0x481f)
#define MIPI_MIN_CLK_POST_H     (0x4820)
#define MIPI_MIN_CLK_POST_L     (0x4821)
#define MIPI_MIN_CLK_TRAIL_H    (0x4822)
#define MIPI_MIN_CLK_TRAIL_L    (0x4823)
#define MIPI_MIN_LPX_PCLK_H     (0x4824)
#define MIPI_MIN_LPX_PCLK_L     (0x4825)
#define MIPI_MIN_HS_PREP_H      (0x4826)
#define MIPI_MIN_HS_PREP_L      (0x4827)
#define MIPI_MIN_HS_EXIT_H      (0x4828)
#define MIPI_MIN_HS_EXIT_L      (0x4829)
#define MIPI_MIN_HS_ZERO_UI     (0x482a)
#define MIPI_MIN_HS_TRAIL_UI    (0x482b)
#define MIPI_MIN_CLK_ZERO_UI    (0x482c)
#define MIPI_MIN_CLK_PREP_UI    (0x482d)
#define MIPI_MIN_CLK_POST_UI    (0x482e)
#define MIPI_MIN_CLK_TRAIL_UI   (0x482f)
#define MIPI_MIN_LPX_PCLK_UI    (0x4830)
#define MIPI_MIN_HS_PREP_UI     (0x4831)
#define MIPI_MIN_HS_EXIT_UI     (0x4832)
#define MIPI_PCLK_PERIOD        (0x4833)

/*** ISP Frame Control Registers ***/
#define FRAME_CTRL01h           (0x4901)
#define FRAME_CTRL02h           (0x4902)

/*** ISP Top Control Registers ***/
#define ISP_CTRL00h             (0x5000)
#define ISP_CTRL01h             (0x5001)
#define ISP_CTRL03h             (0x5003)
#define ISP_CTRL05h             (0x5005)
#define ISP_MISC_0              (0x501d)
#define ISP_MISC_1              (0x501e)
#define FORMAT_MUX_CTRL         (0x501f)
#define DITHER_CTRL_0           (0x5020)
#define DRAW_WIND_CTRL00h       (0x5027)
#define DRAW_WIND_L_POS_CTRL_0  (0x5028)
#define DRAW_WIND_L_POS_CTRL_1  (0x5029)
#define DRAW_WIND_R_POS_CTRL_0  (0x502a)
#define DRAW_WIND_R_POS_CTRL_1  (0x502b)
#define DRAW_WIND_T_POS_CTRL_0  (0x502c)
#define DRAW_WIND_T_POS_CTRL_1  (0x502d)
#define DRAW_WIND_B_POS_CTRL_0  (0x502e)
#define DRAW_WIND_B_POS_CTRL_1  (0x502f)
#define DRAW_WIND_HBW_CTRL_0    (0x5030)
#define DRAW_WIND_HBW_CTRL_1    (0x5031)
#define DRAW_WIND_VBW_CTRL_0    (0x5032)
#define DRAW_WIND_VBW_CTRL_1    (0x5033)
#define DRAW_WIND_Y_CTRL        (0x5034)
#define DRAW_WIND_U_CTRL        (0x5035)
#define DRAW_WIND_V_CTRL        (0x5036)
#define PRE_ISP_TEST_SETTING    (0x503d)
#define ISP_SENSOR_BIAS_I       (0x5061)
#define ISP_SENSOR_GAIN_I_0     (0x5062)
#define ISP_SENSOR_GAIN_I_1     (0x5063)

/*** AWB Control Registers ***/
#define AWB_CTRL00              (0x5180)
#define AWB_CTRL01              (0x5181)
#define AWB_CTRL02              (0x5182)
#define AWB_CTRL03              (0x5183)
#define AWB_CTRL04              (0x5184)
#define AWB_CTRL05              (0x5185)
#define AWB_CTRL17              (0x5191)
#define AWB_CTRL18              (0x5192)
#define AWB_CTRL19              (0x5193)
#define AWB_CTRL20              (0x5194)
#define AWB_CTRL21              (0x5195)
#define AWB_CTRL22              (0x5196)
#define AWB_CTRL23              (0x5197)
#define AWB_CTRL30              (0x519e)
#define AWB_CURRENT_R_GAIN_0    (0x519f)
#define AWB_CURRENT_R_GAIN_1    (0x51a0)
#define AWB_CURRENT_G_GAIN_0    (0x51a1)
#define AWB_CURRENT_G_GAIN_1    (0x51a2)
#define AWB_CURRENT_B_GAIN_0    (0x51a3)
#define AWB_CURRENT_B_GAIN_1    (0x51a4)
#define AWB_AVG_B_0             (0x51a5)
#define AWB_AVG_B_1             (0x51a6)
#define AWB_AVG_B_2             (0x51a7)
#define AWB_CTRL74              (0x51d0)

/*** CIP Control Registers ***/
#define CIP_SHARPENMT_THD1      (0x5300)
#define CIP_SHARPENMT_THD2      (0x5301)
#define CIP_SHARPENMT_OFFSET1   (0x5302)
#define CIP_SHARPENMT_OFFSET2   (0x5303)
#define CIP_DNS_THD1            (0x5304)
#define CIP_DNS_THD2            (0x5305)
#define CIP_DNS_OFFSET1         (0x5306)
#define CIP_DNS_OFFSET2         (0x5307)
#define CIP_CTRL                (0x5308)
#define CIP_SHARPENTH_THD1      (0x5309)
#define CIP_SHARPENTH_THD2      (0x530a)
#define CIP_SHARPENTH_OFFSET1   (0x530b)
#define CIP_SHARPENTH_OFFSET2   (0x530c)
#define CIP_EDGE_MT_AUTO        (0x530d)
#define CIP_DNS_THD_AUTO        (0x530e)
#define CIP_SHARPEN_THD_AUTO    (0x530f)

/*** CMX Control ***/
#define CMX_CTRL                (0x5380)
#define CMX1                    (0x5381)
#define CMX2                    (0x5382)
#define CMX3                    (0x5383)
#define CMX4                    (0x5384)
#define CMX5                    (0x5385)
#define CMX6                    (0x5386)
#define CMX7                    (0x5387)
#define CMX8                    (0x5388)
#define CMX9                    (0x5389)
#define CMXSIGN_0               (0x538a)
#define CMXSIGN_1               (0x538b)

/*** Gamma Control Registers ***/
#define GAMMA_CTRL              (0x5480)
#define GAMMA_YST00h            (0x5481)
#define GAMMA_YST01h            (0x5482)
#define GAMMA_YST02h            (0x5483)
#define GAMMA_YST03h            (0x5484)
#define GAMMA_YST04h            (0x5485)
#define GAMMA_YST05h            (0x5486)
#define GAMMA_YST06h            (0x5487)
#define GAMMA_YST07h            (0x5488)
#define GAMMA_YST08h            (0x5489)
#define GAMMA_YST09h            (0x548a)
#define GAMMA_YST0Ah            (0x548b)
#define GAMMA_YST0Bh            (0x548c)
#define GAMMA_YST0Ch            (0x548d)
#define GAMMA_YST0Dh            (0x548e)
#define GAMMA_YST0Eh            (0x548f)
#define GAMMA_YST0Fh            (0x5490)

/*** SDE Control Registers ***/
#define SDE_CTRL00              (0x5580)
#define SDE_CTRL01              (0x5581)
#define SDE_CTRL02              (0x5582)
#define SDE_CTRL03              (0x5583)
#define SDE_CTRL04              (0x5584)
#define SDE_CTRL05              (0x5585)
#define SDE_CTRL06              (0x5586)
#define SDE_CTRL07              (0x5587)
#define SDE_CTRL08              (0x5588)
#define SDE_CTRL09              (0x5589)
#define SDE_CTRL10              (0x558a)
#define SDE_CTRL11              (0x558b)
#define SDE_CTRL12              (0x558c)

/*** Scale Registers ***/
#define SCALE_CTRL0             (0x5600)
#define SCALE_CTRL1             (0x5601)
#define SCALE_CTRL2             (0x5602)
#define SCALE_CTRL3             (0x5603)
#define SCALE_CTRL4             (0x5604)
#define SCALE_CTRL5             (0x5605)
#define SCALE_CTRL6             (0x5606)

/*** AVG Registers ***/
#define X_START_0               (0x5680)
#define X_START_1               (0x5681)
#define Y_START_0               (0x5682)
#define Y_START_1               (0x5683)
#define X_WINDOW_0              (0x5684)
#define X_WINDOW_1              (0x5685)
#define Y_WINDOW_0              (0x5686)
#define Y_WINDOW_1              (0x5687)
#define WEIGHT00                (0x5688)
#define WEIGHT01                (0x5689)
#define WEIGHT02                (0x568a)
#define WEIGHT03                (0x568b)
#define WEIGHT04                (0x568c)
#define WEIGHT05                (0x568d)
#define WEIGHT06                (0x568e)
#define WEIGHT07                (0x568f)
#define AVG_CTRL                (0x5690)
#define AVG_WIN00               (0x5691)
#define AVG_WIN01               (0x5692)
#define AVG_WIN02               (0x5693)
#define AVG_WIN03               (0x5694)
#define AVG_WIN10               (0x5695)
#define AVG_WIN11               (0x5696)
#define AVG_WIN12               (0x5697)
#define AVG_WIN13               (0x5698)
#define AVG_WIN20               (0x5699)
#define AVG_WIN21               (0x569a)
#define AVG_WIN22               (0x569b)
#define AVG_WIN23               (0x569c)
#define AVG_WIN30               (0x569d)
#define AVG_WIN31               (0x569e)
#define AVG_WIN32               (0x569f)
#define AVG_WIN33               (0x56a0)
#define AVG_READOUT             (0x56a1)
#define AVG_WEIGHT_SUM          (0x56a2)

/*** LENC Control Registers ***/
#define GMTRX00                 (0x5800)
#define GMTRX01                 (0x5801)
#define GMTRX02                 (0x5802)
#define GMTRX03                 (0x5803)
#define GMTRX04                 (0x5804)
#define GMTRX05                 (0x5805)
#define GMTRX10                 (0x5806)
#define GMTRX11                 (0x5807)
#define GMTRX12                 (0x5808)
#define GMTRX13                 (0x5809)
#define GMTRX14                 (0x580a)
#define GMTRX15                 (0x580b)
#define GMTRX20                 (0x580c)
#define GMTRX21                 (0x580d)
#define GMTRX22                 (0x580e)
#define GMTRX23                 (0x580f)
#define GMTRX24                 (0x5810)
#define GMTRX25                 (0x5811)
#define GMTRX30                 (0x5812)
#define GMTRX31                 (0x5813)
#define GMTRX32                 (0x5814)
#define GMTRX33                 (0x5815)
#define GMTRX34                 (0x5816)
#define GMTRX35                 (0x5817)
#define GMTRX40                 (0x5818)
#define GMTRX41                 (0x5819)
#define GMTRX42                 (0x581a)
#define GMTRX43                 (0x581b)
#define GMTRX44                 (0x581c)
#define GMTRX45                 (0x581d)
#define GMTRX50                 (0x581e)
#define GMTRX51                 (0x581f)
#define GMTRX52                 (0x5820)
#define GMTRX53                 (0x5821)
#define GMTRX54                 (0x5822)
#define GMTRX55                 (0x5823)
#define BRMATRX00               (0x5824)
#define BRMATRX01               (0x5825)
#define BRMATRX02               (0x5826)
#define BRMATRX03               (0x5827)
#define BRMATRX04               (0x5828)
#define BRMATRX05               (0x5829)
#define BRMATRX06               (0x582a)
#define BRMATRX07               (0x582b)
#define BRMATRX08               (0x582c)
#define BRMATRX09               (0x582d)
#define BRMATRX20               (0x582e)
#define BRMATRX21               (0x582f)
#define BRMATRX22               (0x5830)
#define BRMATRX23               (0x5831)
#define BRMATRX24               (0x5832)
#define BRMATRX30               (0x5833)
#define BRMATRX31               (0x5834)
#define BRMATRX32               (0x5835)
#define BRMATRX33               (0x5836)
#define BRMATRX34               (0x5837)
#define BRMATRX40               (0x5838)
#define BRMATRX41               (0x5839)
#define BRMATRX42               (0x583a)
#define BRMATRX43               (0x583b)
#define BRMATRX44               (0x583c)
#define LENC_BR_OFFSET          (0x583d)
#define LENC_MAX_GAIN           (0x583e)
#define LENC_MIN_GAIN           (0x583f)
#define LENC_MIN_Q              (0x5840)
#define LENC_CTRL               (0x5841)
#define BR_HSCALE_0             (0x5842)
#define BR_HSCALE_1             (0x5843)
#define BR_VSCALE_0             (0x5844)
#define BR_VSCALE_1             (0x5845)
#define G_HSCALE_0              (0x5846)
#define G_HSCALE_1              (0x5847)
#define G_VSCALE_0              (0x5848)
#define G_VSCALE_1              (0x5849)

/*** AFC Control Registers ***/
#define AFC_CTRL00              (0x6000)
#define AFC_CTRL01              (0x6001)
#define AFC_CTRL02              (0x6002)
#define AFC_CTRL03              (0x6003)
#define AFC_CTRL04              (0x6004)
#define AFC_CTRL05              (0x6005)
#define AFC_CTRL06              (0x6006)
#define AFC_CTRL07              (0x6007)
#define AFC_CTRL08              (0x6008)
#define AFC_CTRL09              (0x6009)
#define AFC_CTRL10              (0x600a)
#define AFC_CTRL11              (0x600b)
#define AFC_CTRL12              (0x600c)
#define AFC_CTRL13              (0x600d)
#define AFC_CTRL14              (0x600e)
#define AFC_CTRL15              (0x600f)
#define AFC_CTRL16              (0x6010)
#define AFC_CTRL17              (0x6011)
#define AFC_CTRL18              (0x6012)
#define AFC_CTRL19              (0x6013)
#define AFC_CTRL20              (0x6014)
#define AFC_CTRL21              (0x6015)
#define AFC_CTRL22              (0x6016)
#define AFC_CTRL23              (0x6017)
#define AFC_CTRL24              (0x6018)
#define AFC_CTRL25              (0x6019)
#define AFC_CTRL26              (0x601a)
#define AFC_CTRL27              (0x601b)
#define AFC_CTRL28              (0x601c)
#define AFC_CTRL29              (0x601d)
#define AFC_CTRL30              (0x601e)
#define AFC_CTRL31              (0x601f)
#define AFC_CTRL32              (0x6020)
#define AFC_CTRL33              (0x6021)
#define AFC_CTRL34              (0x6022)
#define AFC_CTRL35              (0x6023)
#define AFC_CTRL36              (0x6024)
#define AFC_CTRL37              (0x6025)
#define AFC_CTRL38              (0x6026)
#define AFC_CTRL39              (0x6027)
#define AFC_CTRL40              (0x6028)
#define AFC_CTRL41              (0x6029)
#define AFC_CTRL42              (0x602a)
#define AFC_CTRL43              (0x602b)
#define AFC_CTRL44              (0x602c)
#define AFC_CTRL45              (0x602d)
#define AFC_CTRL46              (0x602e)
#define AFC_CTRL47              (0x602f)
#define AFC_CTRL48              (0x6030)
#define AFC_CTRL49              (0x6031)
#define AFC_CTRL50              (0x6032)
#define AFC_CTRL51              (0x6033)
#define AFC_CTRL52              (0x6034)
#define AFC_CTRL53              (0x6035)
#define AFC_CTRL54              (0x6036)
#define AFC_CTRL55              (0x6037)
#define AFC_CTRL56              (0x6038)
#define AFC_CTRL57              (0x6039)
#define AFC_CTRL58              (0x603a)
#define AFC_CTRL59              (0x603b)
#define AFC_CTRL60              (0x603c)
#define AFC_CTRL61              (0x603d)
#define AFC_CTRL62              (0x603e)
#define AFC_CTRL63              (0x603f)



#define OV5640_SYSREM_RESET00                      0x3000U
#define OV5640_SYSREM_RESET01                      0x3001U
#define OV5640_SYSREM_RESET02                      0x3002U
#define OV5640_SYSREM_RESET03                      0x3003U
#define OV5640_CLOCK_ENABLE00                      0x3004U
#define OV5640_CLOCK_ENABLE01                      0x3005U
#define OV5640_CLOCK_ENABLE02                      0x3006U
#define OV5640_CLOCK_ENABLE03                      0x3007U
#define OV5640_SYSTEM_CTROL0                       0x3008U
#define OV5640_CHIP_ID_HIGH_BYTE                   0x300AU
#define OV5640_CHIP_ID_LOW_BYTE                    0x300BU
#define OV5640_MIPI_CONTROL00                      0x300EU
#define OV5640_PAD_OUTPUT_ENABLE00                 0x3016U
#define OV5640_PAD_OUTPUT_ENABLE01                 0x3017U
#define OV5640_PAD_OUTPUT_ENABLE02                 0x3018U
#define OV5640_PAD_OUTPUT_VALUE00                  0x3019U
#define OV5640_PAD_OUTPUT_VALUE01                  0x301AU
#define OV5640_PAD_OUTPUT_VALUE02                  0x301BU
#define OV5640_PAD_SELECT00                        0x301CU
#define OV5640_PAD_SELECT01                        0x301DU
#define OV5640_PAD_SELECT02                        0x301EU
#define OV5640_CHIP_REVISION                       0x302AU
#define OV5640_PAD_CONTROL00                       0x301CU
#define OV5640_SC_PWC                              0x3031U
#define OV5640_SC_PLL_CONTRL0                      0x3034U
#define OV5640_SC_PLL_CONTRL1                      0x3035U
#define OV5640_SC_PLL_CONTRL2                      0x3036U
#define OV5640_SC_PLL_CONTRL3                      0x3037U
#define OV5640_SC_PLL_CONTRL4                      0x3038U
#define OV5640_SC_PLL_CONTRL5                      0x3039U
#define OV5640_SC_PLLS_CTRL0                       0x303AU
#define OV5640_SC_PLLS_CTRL1                       0x303BU
#define OV5640_SC_PLLS_CTRL2                       0x303CU
#define OV5640_SC_PLLS_CTRL3                       0x303DU
#define OV5640_IO_PAD_VALUE00                      0x3050U
#define OV5640_IO_PAD_VALUE01                      0x3051U
#define OV5640_IO_PAD_VALUE02                      0x3052U

/* SCCB control [0x3100 ~ 0x3108]                       */
#define OV5640_SCCB_ID                             0x3100U
#define OV5640_SCCB_SYSTEM_CTRL0                   0x3102U
#define OV5640_SCCB_SYSTEM_CTRL1                   0x3103U
#define OV5640_SYSTEM_ROOT_DIVIDER                 0x3108U

/* SRB control [0x3200 ~ 0x3213]                        */
#define OV5640_GROUP_ADDR0                         0x3200U
#define OV5640_GROUP_ADDR1                         0x3201U
#define OV5640_GROUP_ADDR2                         0x3202U
#define OV5640_GROUP_ADDR3                         0x3203U
#define OV5640_SRM_GROUP_ACCESS                    0x3212U
#define OV5640_SRM_GROUP_STATUS                    0x3213U

/* AWB gain control [0x3400 ~ 0x3406]                   */
#define OV5640_AWB_R_GAIN_MSB                      0x3400U
#define OV5640_AWB_R_GAIN_LSB                      0x3401U
#define OV5640_AWB_G_GAIN_MSB                      0x3402U
#define OV5640_AWB_G_GAIN_LSB                      0x3403U
#define OV5640_AWB_B_GAIN_MSB                      0x3404U
#define OV5640_AWB_B_GAIN_LSB                      0x3405U
#define OV5640_AWB_MANUAL_CONTROL                  0x3406U

/* AEC/AGC control [0x3500 ~ 0x350D]                    */
#define OV5640_AEC_PK_EXPOSURE_19_16               0x3500U
#define OV5640_AEC_PK_EXPOSURE_HIGH                0x3501U
#define OV5640_AEC_PK_EXPOSURE_LOW                 0x3502U
#define OV5640_AEC_PK_MANUAL                       0x3503U
#define OV5640_AEC_PK_REAL_GAIN_9_8                0x350AU
#define OV5640_AEC_PK_REAL_GAIN_LOW                0x350BU
#define OV5640_AEC_PK_VTS_HIGH                     0x350CU
#define OV5640_AEC_PK_VTS_LOW                      0x350DU

/* VCM control [0x3600 ~ 0x3606]                        */
#define OV5640_VCM_CONTROL_0                       0x3602U
#define OV5640_VCM_CONTROL_1                       0x3603U
#define OV5640_VCM_CONTROL_2                       0x3604U
#define OV5640_VCM_CONTROL_3                       0x3605U
#define OV5640_VCM_CONTROL_4                       0x3606U

/* timing control [0x3800 ~ 0x3821]                    */
#define OV5640_TIMING_HS_HIGH                      0x3800U
#define OV5640_TIMING_HS_LOW                       0x3801U
#define OV5640_TIMING_VS_HIGH                      0x3802U
#define OV5640_TIMING_VS_LOW                       0x3803U
#define OV5640_TIMING_HW_HIGH                      0x3804U
#define OV5640_TIMING_HW_LOW                       0x3805U
#define OV5640_TIMING_VH_HIGH                      0x3806U
#define OV5640_TIMING_VH_LOW                       0x3807U
#define OV5640_TIMING_DVPHO_HIGH                   0x3808U
#define OV5640_TIMING_DVPHO_LOW                    0x3809U
#define OV5640_TIMING_DVPVO_HIGH                   0x380AU
#define OV5640_TIMING_DVPVO_LOW                    0x380BU
#define OV5640_TIMING_HTS_HIGH                     0x380CU
#define OV5640_TIMING_HTS_LOW                      0x380DU
#define OV5640_TIMING_VTS_HIGH                     0x380EU
#define OV5640_TIMING_VTS_LOW                      0x380FU
#define OV5640_TIMING_HOFFSET_HIGH                 0x3810U
#define OV5640_TIMING_HOFFSET_LOW                  0x3811U
#define OV5640_TIMING_VOFFSET_HIGH                 0x3812U
#define OV5640_TIMING_VOFFSET_LOW                  0x3813U
#define OV5640_TIMING_X_INC                        0x3814U
#define OV5640_TIMING_Y_INC                        0x3815U
#define OV5640_HSYNC_START_HIGH                    0x3816U
#define OV5640_HSYNC_START_LOW                     0x3817U
#define OV5640_HSYNC_WIDTH_HIGH                    0x3818U
#define OV5640_HSYNC_WIDTH_LOW                     0x3819U
#define OV5640_TIMING_TC_REG20                     0x3820U
#define OV5640_TIMING_TC_REG21                     0x3821U

/* AEC/AGC power down domain control [0x3A00 ~ 0x3A25] */
#define OV5640_AEC_CTRL00                          0x3A00U
#define OV5640_AEC_CTRL01                          0x3A01U
#define OV5640_AEC_CTRL02                          0x3A02U
#define OV5640_AEC_CTRL03                          0x3A03U
#define OV5640_AEC_CTRL04                          0x3A04U
#define OV5640_AEC_CTRL05                          0x3A05U
#define OV5640_AEC_CTRL06                          0x3A06U
#define OV5640_AEC_CTRL07                          0x3A07U
#define OV5640_AEC_B50_STEP_HIGH                   0x3A08U
#define OV5640_AEC_B50_STEP_LOW                    0x3A09U
#define OV5640_AEC_B60_STEP_HIGH                   0x3A0AU
#define OV5640_AEC_B60_STEP_LOW                    0x3A0BU
#define OV5640_AEC_AEC_CTRL0C                      0x3A0CU
#define OV5640_AEC_CTRL0D                          0x3A0DU
#define OV5640_AEC_CTRL0E                          0x3A0EU
#define OV5640_AEC_CTRL0F                          0x3A0FU
#define OV5640_AEC_CTRL10                          0x3A10U
#define OV5640_AEC_CTRL11                          0x3A11U
#define OV5640_AEC_CTRL13                          0x3A13U
#define OV5640_AEC_MAX_EXPO_HIGH                   0x3A14U
#define OV5640_AEC_MAX_EXPO_LOW                    0x3A15U
#define OV5640_AEC_CTRL17                          0x3A17U
#define OV5640_AEC_GAIN_CEILING_HIGH               0x3A18U
#define OV5640_AEC_GAIN_CEILING_LOW                0x3A19U
#define OV5640_AEC_DIFF_MIN                        0x3A1AU
#define OV5640_AEC_CTRL1B                          0x3A1BU
#define OV5640_LED_ADD_ROW_HIGH                    0x3A1CU
#define OV5640_LED_ADD_ROW_LOW                     0x3A1DU
#define OV5640_AEC_CTRL1E                          0x3A1EU
#define OV5640_AEC_CTRL1F                          0x3A1FU
#define OV5640_AEC_CTRL20                          0x3A20U
#define OV5640_AEC_CTRL21                          0x3A21U
#define OV5640_AEC_CTRL25                          0x3A25U

/* strobe control [0x3B00 ~ 0x3B0C]                      */
#define OV5640_STROBE_CTRL                         0x3B00U
#define OV5640_FREX_EXPOSURE02                     0x3B01U
#define OV5640_FREX_SHUTTER_DLY01                  0x3B02U
#define OV5640_FREX_SHUTTER_DLY00                  0x3B03U
#define OV5640_FREX_EXPOSURE01                     0x3B04U
#define OV5640_FREX_EXPOSURE00                     0x3B05U
#define OV5640_FREX_CTRL07                         0x3B06U
#define OV5640_FREX_MODE                           0x3B07U
#define OV5640_FREX_RQST                           0x3B08U
#define OV5640_FREX_HREF_DLY                       0x3B09U
#define OV5640_FREX_RST_LENGTH                     0x3B0AU
#define OV5640_STROBE_WIDTH_HIGH                   0x3B0BU
#define OV5640_STROBE_WIDTH_LOW                    0x3B0CU

/* 50/60Hz detector control [0x3C00 ~ 0x3C1E]            */
#define OV5640_5060HZ_CTRL00                       0x3C00U
#define OV5640_5060HZ_CTRL01                       0x3C01U
#define OV5640_5060HZ_CTRL02                       0x3C02U
#define OV5640_5060HZ_CTRL03                       0x3C03U
#define OV5640_5060HZ_CTRL04                       0x3C04U
#define OV5640_5060HZ_CTRL05                       0x3C05U
#define OV5640_LIGHTMETER1_TH_HIGH                 0x3C06U
#define OV5640_LIGHTMETER1_TH_LOW                  0x3C07U
#define OV5640_LIGHTMETER2_TH_HIGH                 0x3C08U
#define OV5640_LIGHTMETER2_TH_LOW                  0x3C09U
#define OV5640_SAMPLE_NUMBER_HIGH                  0x3C0AU
#define OV5640_SAMPLE_NUMBER_LOW                   0x3C0BU
#define OV5640_SIGMA_DELTA_CTRL0C                  0x3C0CU
#define OV5640_SUM50_BYTE4                         0x3C0DU
#define OV5640_SUM50_BYTE3                         0x3C0EU
#define OV5640_SUM50_BYTE2                         0x3C0FU
#define OV5640_SUM50_BYTE1                         0x3C10U
#define OV5640_SUM60_BYTE4                         0x3C11U
#define OV5640_SUM60_BYTE3                         0x3C12U
#define OV5640_SUM60_BYTE2                         0x3C13U
#define OV5640_SUM60_BYTE1                         0x3C14U
#define OV5640_SUM5060_HIGH                        0x3C15U
#define OV5640_SUM5060_LOW                         0x3C16U
#define OV5640_BLOCK_CNTR_HIGH                     0x3C17U
#define OV5640_BLOCK_CNTR_LOW                      0x3C18U
#define OV5640_B6_HIGH                             0x3C19U
#define OV5640_B6_LOW                              0x3C1AU
#define OV5640_LIGHTMETER_OUTPUT_BYTE3             0x3C1BU
#define OV5640_LIGHTMETER_OUTPUT_BYTE2             0x3C1CU
#define OV5640_LIGHTMETER_OUTPUT_BYTE1             0x3C1DU
#define OV5640_SUM_THRESHOLD                       0x3C1EU

/* OTP control [0x3D00 ~ 0x3D21]                         */
/* MC control [0x3F00 ~ 0x3F0D]                          */
/* BLC control [0x4000 ~ 0x4033]                         */
#define OV5640_BLC_CTRL00                          0x4000U
#define OV5640_BLC_CTRL01                          0x4001U
#define OV5640_BLC_CTRL02                          0x4002U
#define OV5640_BLC_CTRL03                          0x4003U
#define OV5640_BLC_CTRL04                          0x4004U
#define OV5640_BLC_CTRL05                          0x4005U

/* frame control [0x4201 ~ 0x4202]                       */
#define OV5640_FRAME_CTRL01                        0x4201U
#define OV5640_FRAME_CTRL02                        0x4202U

/* format control [0x4300 ~ 0x430D]                      */
#define OV5640_FORMAT_CTRL00                       0x4300U
#define OV5640_FORMAT_CTRL01                       0x4301U
#define OV5640_YMAX_VAL_HIGH                       0x4302U
#define OV5640_YMAX_VAL_LOW                        0x4303U
#define OV5640_YMIN_VAL_HIGH                       0x4304U
#define OV5640_YMIN_VAL_LOW                        0x4305U
#define OV5640_UMAX_VAL_HIGH                       0x4306U
#define OV5640_UMAX_VAL_LOW                        0x4307U
#define OV5640_UMIN_VAL_HIGH                       0x4308U
#define OV5640_UMIN_VAL_LOW                        0x4309U
#define OV5640_VMAX_VAL_HIGH                       0x430AU
#define OV5640_VMAX_VAL_LOW                        0x430BU
#define OV5640_VMIN_VAL_HIGH                       0x430CU
#define OV5640_VMIN_VAL_LOW                        0x430DU

/* JPEG control [0x4400 ~ 0x4431]                        */
#define OV5640_JPEG_CTRL00                         0x4400U
#define OV5640_JPEG_CTRL01                         0x4401U
#define OV5640_JPEG_CTRL02                         0x4402U
#define OV5640_JPEG_CTRL03                         0x4403U
#define OV5640_JPEG_CTRL04                         0x4404U
#define OV5640_JPEG_CTRL05                         0x4405U
#define OV5640_JPEG_CTRL06                         0x4406U
#define OV5640_JPEG_CTRL07                         0x4407U
#define OV5640_JPEG_ISI_CTRL1                      0x4408U
#define OV5640_JPEG_CTRL09                         0x4409U
#define OV5640_JPEG_CTRL0A                         0x440AU
#define OV5640_JPEG_CTRL0B                         0x440BU
#define OV5640_JPEG_CTRL0C                         0x440CU
#define OV5640_JPEG_QT_DATA                        0x4410U
#define OV5640_JPEG_QT_ADDR                        0x4411U
#define OV5640_JPEG_ISI_DATA                       0x4412U
#define OV5640_JPEG_ISI_CTRL2                      0x4413U
#define OV5640_JPEG_LENGTH_BYTE3                   0x4414U
#define OV5640_JPEG_LENGTH_BYTE2                   0x4415U
#define OV5640_JPEG_LENGTH_BYTE1                   0x4416U
#define OV5640_JFIFO_OVERFLOW                      0x4417U

/* VFIFO control [0x4600 ~ 0x460D]                       */
#define OV5640_VFIFO_CTRL00                        0x4600U
#define OV5640_VFIFO_HSIZE_HIGH                    0x4602U
#define OV5640_VFIFO_HSIZE_LOW                     0x4603U
#define OV5640_VFIFO_VSIZE_HIGH                    0x4604U
#define OV5640_VFIFO_VSIZE_LOW                     0x4605U
#define OV5640_VFIFO_CTRL0C                        0x460CU
#define OV5640_VFIFO_CTRL0D                        0x460DU

/* DVP control [0x4709 ~ 0x4745]                         */
#define OV5640_DVP_VSYNC_WIDTH0                    0x4709U
#define OV5640_DVP_VSYNC_WIDTH1                    0x470AU
#define OV5640_DVP_VSYNC_WIDTH2                    0x470BU
#define OV5640_PAD_LEFT_CTRL                       0x4711U
#define OV5640_PAD_RIGHT_CTRL                      0x4712U
#define OV5640_JPG_MODE_SELECT                     0x4713U
#define OV5640_656_DUMMY_LINE                      0x4715U
#define OV5640_CCIR656_CTRL                        0x4719U
#define OV5640_HSYNC_CTRL00                        0x471BU
#define OV5640_DVP_VSYN_CTRL                       0x471DU
#define OV5640_DVP_HREF_CTRL                       0x471FU
#define OV5640_VSTART_OFFSET                       0x4721U
#define OV5640_VEND_OFFSET                         0x4722U
#define OV5640_DVP_CTRL23                          0x4723U
#define OV5640_CCIR656_CTRL00                      0x4730U
#define OV5640_CCIR656_CTRL01                      0x4731U
#define OV5640_CCIR656_FS                          0x4732U
#define OV5640_CCIR656_FE                          0x4733U
#define OV5640_CCIR656_LS                          0x4734U
#define OV5640_CCIR656_LE                          0x4735U
#define OV5640_CCIR656_CTRL06                      0x4736U
#define OV5640_CCIR656_CTRL07                      0x4737U
#define OV5640_CCIR656_CTRL08                      0x4738U
#define OV5640_POLARITY_CTRL                       0x4740U
#define OV5640_TEST_PATTERN                        0x4741U
#define OV5640_DATA_ORDER                          0x4745U

/* MIPI control [0x4800 ~ 0x4837]                        */
#define OV5640_MIPI_CTRL00                         0x4800U
#define OV5640_MIPI_CTRL01                         0x4801U
#define OV5640_MIPI_CTRL05                         0x4805U
#define OV5640_MIPI_DATA_ORDER                     0x480AU
#define OV5640_MIN_HS_ZERO_HIGH                    0x4818U
#define OV5640_MIN_HS_ZERO_LOW                     0x4819U
#define OV5640_MIN_MIPI_HS_TRAIL_HIGH              0x481AU
#define OV5640_MIN_MIPI_HS_TRAIL_LOW               0x481BU
#define OV5640_MIN_MIPI_CLK_ZERO_HIGH              0x481CU
#define OV5640_MIN_MIPI_CLK_ZERO_LOW               0x481DU
#define OV5640_MIN_MIPI_CLK_PREPARE_HIGH           0x481EU
#define OV5640_MIN_MIPI_CLK_PREPARE_LOW            0x481FU
#define OV5640_MIN_CLK_POST_HIGH                   0x4820U
#define OV5640_MIN_CLK_POST_LOW                    0x4821U
#define OV5640_MIN_CLK_TRAIL_HIGH                  0x4822U
#define OV5640_MIN_CLK_TRAIL_LOW                   0x4823U
#define OV5640_MIN_LPX_PCLK_HIGH                   0x4824U
#define OV5640_MIN_LPX_PCLK_LOW                    0x4825U
#define OV5640_MIN_HS_PREPARE_HIGH                 0x4826U
#define OV5640_MIN_HS_PREPARE_LOW                  0x4827U
#define OV5640_MIN_HS_EXIT_HIGH                    0x4828U
#define OV5640_MIN_HS_EXIT_LOW                     0x4829U
#define OV5640_MIN_HS_ZERO_UI                      0x482AU
#define OV5640_MIN_HS_TRAIL_UI                     0x482BU
#define OV5640_MIN_CLK_ZERO_UI                     0x482CU
#define OV5640_MIN_CLK_PREPARE_UI                  0x482DU
#define OV5640_MIN_CLK_POST_UI                     0x482EU
#define OV5640_MIN_CLK_TRAIL_UI                    0x482FU
#define OV5640_MIN_LPX_PCLK_UI                     0x4830U
#define OV5640_MIN_HS_PREPARE_UI                   0x4831U
#define OV5640_MIN_HS_EXIT_UI                      0x4832U
#define OV5640_PCLK_PERIOD                         0x4837U

/* ISP frame control [0x4901 ~ 0x4902]                   */
#define OV5640_ISP_FRAME_CTRL01                    0x4901U
#define OV5640_ISP_FRAME_CTRL02                    0x4902U

/* ISP top control [0x5000 ~ 0x5063]                     */
#define OV5640_ISP_CONTROL00                       0x5000U
#define OV5640_ISP_CONTROL01                       0x5001U
#define OV5640_ISP_CONTROL03                       0x5003U
#define OV5640_ISP_CONTROL05                       0x5005U
#define OV5640_ISP_MISC0                           0x501DU
#define OV5640_ISP_MISC1                           0x501EU
#define OV5640_FORMAT_MUX_CTRL                     0x501FU
#define OV5640_DITHER_CTRL0                        0x5020U
#define OV5640_DRAW_WINDOW_CTRL00                  0x5027U
#define OV5640_DRAW_WINDOW_LEFT_CTRL_HIGH          0x5028U
#define OV5640_DRAW_WINDOW_LEFT_CTRL_LOW           0x5029U
#define OV5640_DRAW_WINDOW_RIGHT_CTRL_HIGH         0x502AU
#define OV5640_DRAW_WINDOW_RIGHT_CTRL_LOW          0x502BU
#define OV5640_DRAW_WINDOW_TOP_CTRL_HIGH           0x502CU
#define OV5640_DRAW_WINDOW_TOP_CTRL_LOW            0x502DU
#define OV5640_DRAW_WINDOW_BOTTOM_CTRL_HIGH        0x502EU
#define OV5640_DRAW_WINDOW_BOTTOM_CTRL_LOW         0x502FU
#define OV5640_DRAW_WINDOW_HBW_CTRL_HIGH           0x5030U          /* HBW: Horizontal Boundary Width */
#define OV5640_DRAW_WINDOW_HBW_CTRL_LOW            0x5031U
#define OV5640_DRAW_WINDOW_VBW_CTRL_HIGH           0x5032U          /* VBW: Vertical Boundary Width */
#define OV5640_DRAW_WINDOW_VBW_CTRL_LOW            0x5033U
#define OV5640_DRAW_WINDOW_Y_CTRL                  0x5034U
#define OV5640_DRAW_WINDOW_U_CTRL                  0x5035U
#define OV5640_DRAW_WINDOW_V_CTRL                  0x5036U
#define OV5640_PRE_ISP_TEST_SETTING1               0x503DU
#define OV5640_ISP_SENSOR_BIAS_I                   0x5061U
#define OV5640_ISP_SENSOR_GAIN1_I                  0x5062U
#define OV5640_ISP_SENSOR_GAIN2_I                  0x5063U

/* AWB control [0x5180 ~ 0x51D0]                         */
#define OV5640_AWB_CTRL00                          0x5180U
#define OV5640_AWB_CTRL01                          0x5181U
#define OV5640_AWB_CTRL02                          0x5182U
#define OV5640_AWB_CTRL03                          0x5183U
#define OV5640_AWB_CTRL04                          0x5184U
#define OV5640_AWB_CTRL05                          0x5185U
#define OV5640_AWB_CTRL06                          0x5186U     /* Advanced AWB control registers: 0x5186 ~ 0x5190 */
#define OV5640_AWB_CTRL07                          0x5187U
#define OV5640_AWB_CTRL08                          0x5188U
#define OV5640_AWB_CTRL09                          0x5189U
#define OV5640_AWB_CTRL10                          0x518AU
#define OV5640_AWB_CTRL11                          0x518BU
#define OV5640_AWB_CTRL12                          0x518CU
#define OV5640_AWB_CTRL13                          0x518DU
#define OV5640_AWB_CTRL14                          0x518EU
#define OV5640_AWB_CTRL15                          0x518FU
#define OV5640_AWB_CTRL16                          0x5190U
#define OV5640_AWB_CTRL17                          0x5191U
#define OV5640_AWB_CTRL18                          0x5192U
#define OV5640_AWB_CTRL19                          0x5193U
#define OV5640_AWB_CTRL20                          0x5194U
#define OV5640_AWB_CTRL21                          0x5195U
#define OV5640_AWB_CTRL22                          0x5196U
#define OV5640_AWB_CTRL23                          0x5197U
#define OV5640_AWB_CTRL24                          0x5198U
#define OV5640_AWB_CTRL25                          0x5199U
#define OV5640_AWB_CTRL26                          0x519AU
#define OV5640_AWB_CTRL27                          0x519BU
#define OV5640_AWB_CTRL28                          0x519CU
#define OV5640_AWB_CTRL29                          0x519DU
#define OV5640_AWB_CTRL30                          0x519EU
#define OV5640_AWB_CURRENT_R_GAIN_HIGH             0x519FU
#define OV5640_AWB_CURRENT_R_GAIN_LOW              0x51A0U
#define OV5640_AWB_CURRENT_G_GAIN_HIGH             0x51A1U
#define OV5640_AWB_CURRENT_G_GAIN_LOW              0x51A2U
#define OV5640_AWB_CURRENT_B_GAIN_HIGH             0x51A3U
#define OV5640_AWB_CURRENT_B_GAIN_LOW              0x51A4U
#define OV5640_AWB_AVERAGE_R                       0x51A5U
#define OV5640_AWB_AVERAGE_G                       0x51A6U
#define OV5640_AWB_AVERAGE_B                       0x51A7U
#define OV5640_AWB_CTRL74                          0x5180U

/* CIP control [0x5300 ~ 0x530F]                         */
#define OV5640_CIP_SHARPENMT_TH1                   0x5300U
#define OV5640_CIP_SHARPENMT_TH2                   0x5301U
#define OV5640_CIP_SHARPENMT_OFFSET1               0x5302U
#define OV5640_CIP_SHARPENMT_OFFSET2               0x5303U
#define OV5640_CIP_DNS_TH1                         0x5304U
#define OV5640_CIP_DNS_TH2                         0x5305U
#define OV5640_CIP_DNS_OFFSET1                     0x5306U
#define OV5640_CIP_DNS_OFFSET2                     0x5307U
#define OV5640_CIP_CTRL                            0x5308U
#define OV5640_CIP_SHARPENTH_TH1                   0x5309U
#define OV5640_CIP_SHARPENTH_TH2                   0x530AU
#define OV5640_CIP_SHARPENTH_OFFSET1               0x530BU
#define OV5640_CIP_SHARPENTH_OFFSET2               0x530CU
#define OV5640_CIP_EDGE_MT_AUTO                    0x530DU
#define OV5640_CIP_DNS_TH_AUTO                     0x530EU
#define OV5640_CIP_SHARPEN_TH_AUTO                 0x530FU

/* CMX control [0x5380 ~ 0x538B]                         */
#define OV5640_CMX_CTRL                            0x5380U
#define OV5640_CMX1                                0x5381U
#define OV5640_CMX2                                0x5382U
#define OV5640_CMX3                                0x5383U
#define OV5640_CMX4                                0x5384U
#define OV5640_CMX5                                0x5385U
#define OV5640_CMX6                                0x5386U
#define OV5640_CMX7                                0x5387U
#define OV5640_CMX8                                0x5388U
#define OV5640_CMX9                                0x5389U
#define OV5640_CMXSIGN_HIGH                        0x538AU
#define OV5640_CMXSIGN_LOW                         0x538BU

/* gamma control [0x5480 ~ 0x5490]                       */
#define OV5640_GAMMA_CTRL00                        0x5480U
#define OV5640_GAMMA_YST00                         0x5481U
#define OV5640_GAMMA_YST01                         0x5482U
#define OV5640_GAMMA_YST02                         0x5483U
#define OV5640_GAMMA_YST03                         0x5484U
#define OV5640_GAMMA_YST04                         0x5485U
#define OV5640_GAMMA_YST05                         0x5486U
#define OV5640_GAMMA_YST06                         0x5487U
#define OV5640_GAMMA_YST07                         0x5488U
#define OV5640_GAMMA_YST08                         0x5489U
#define OV5640_GAMMA_YST09                         0x548AU
#define OV5640_GAMMA_YST0A                         0x548BU
#define OV5640_GAMMA_YST0B                         0x548CU
#define OV5640_GAMMA_YST0C                         0x548DU
#define OV5640_GAMMA_YST0D                         0x548EU
#define OV5640_GAMMA_YST0E                         0x548FU
#define OV5640_GAMMA_YST0F                         0x5490U

/* SDE control [0x5580 ~ 0x558C]                         */
#define OV5640_SDE_CTRL0                           0x5580U
#define OV5640_SDE_CTRL1                           0x5581U
#define OV5640_SDE_CTRL2                           0x5582U
#define OV5640_SDE_CTRL3                           0x5583U
#define OV5640_SDE_CTRL4                           0x5584U
#define OV5640_SDE_CTRL5                           0x5585U
#define OV5640_SDE_CTRL6                           0x5586U
#define OV5640_SDE_CTRL7                           0x5587U
#define OV5640_SDE_CTRL8                           0x5588U
#define OV5640_SDE_CTRL9                           0x5589U
#define OV5640_SDE_CTRL10                          0x558AU
#define OV5640_SDE_CTRL11                          0x558BU
#define OV5640_SDE_CTRL12                          0x558CU

/* scale control [0x5600 ~ 0x5606]                       */
#define OV5640_SCALE_CTRL0                         0x5600U
#define OV5640_SCALE_CTRL1                         0x5601U
#define OV5640_SCALE_CTRL2                         0x5602U
#define OV5640_SCALE_CTRL3                         0x5603U
#define OV5640_SCALE_CTRL4                         0x5604U
#define OV5640_SCALE_CTRL5                         0x5605U
#define OV5640_SCALE_CTRL6                         0x5606U


/* AVG control [0x5680 ~ 0x56A2]                         */
#define OV5640_X_START_HIGH                        0x5680U
#define OV5640_X_START_LOW                         0x5681U
#define OV5640_Y_START_HIGH                        0x5682U
#define OV5640_Y_START_LOW                         0x5683U
#define OV5640_X_WINDOW_HIGH                       0x5684U
#define OV5640_X_WINDOW_LOW                        0x5685U
#define OV5640_Y_WINDOW_HIGH                       0x5686U
#define OV5640_Y_WINDOW_LOW                        0x5687U
#define OV5640_WEIGHT00                            0x5688U
#define OV5640_WEIGHT01                            0x5689U
#define OV5640_WEIGHT02                            0x568AU
#define OV5640_WEIGHT03                            0x568BU
#define OV5640_WEIGHT04                            0x568CU
#define OV5640_WEIGHT05                            0x568DU
#define OV5640_WEIGHT06                            0x568EU
#define OV5640_WEIGHT07                            0x568FU
#define OV5640_AVG_CTRL10                          0x5690U
#define OV5640_AVG_WIN_00                          0x5691U
#define OV5640_AVG_WIN_01                          0x5692U
#define OV5640_AVG_WIN_02                          0x5693U
#define OV5640_AVG_WIN_03                          0x5694U
#define OV5640_AVG_WIN_10                          0x5695U
#define OV5640_AVG_WIN_11                          0x5696U
#define OV5640_AVG_WIN_12                          0x5697U
#define OV5640_AVG_WIN_13                          0x5698U
#define OV5640_AVG_WIN_20                          0x5699U
#define OV5640_AVG_WIN_21                          0x569AU
#define OV5640_AVG_WIN_22                          0x569BU
#define OV5640_AVG_WIN_23                          0x569CU
#define OV5640_AVG_WIN_30                          0x569DU
#define OV5640_AVG_WIN_31                          0x569EU
#define OV5640_AVG_WIN_32                          0x569FU
#define OV5640_AVG_WIN_33                          0x56A0U
#define OV5640_AVG_READOUT                         0x56A1U
#define OV5640_AVG_WEIGHT_SUM                      0x56A2U

/* LENC control [0x5800 ~ 0x5849]                        */
#define OV5640_GMTRX00                             0x5800U
#define OV5640_GMTRX01                             0x5801U
#define OV5640_GMTRX02                             0x5802U
#define OV5640_GMTRX03                             0x5803U
#define OV5640_GMTRX04                             0x5804U
#define OV5640_GMTRX05                             0x5805U
#define OV5640_GMTRX10                             0x5806U
#define OV5640_GMTRX11                             0x5807U
#define OV5640_GMTRX12                             0x5808U
#define OV5640_GMTRX13                             0x5809U
#define OV5640_GMTRX14                             0x580AU
#define OV5640_GMTRX15                             0x580BU
#define OV5640_GMTRX20                             0x580CU
#define OV5640_GMTRX21                             0x580DU
#define OV5640_GMTRX22                             0x580EU
#define OV5640_GMTRX23                             0x580FU
#define OV5640_GMTRX24                             0x5810U
#define OV5640_GMTRX25                             0x5811U
#define OV5640_GMTRX30                             0x5812U
#define OV5640_GMTRX31                             0x5813U
#define OV5640_GMTRX32                             0x5814U
#define OV5640_GMTRX33                             0x5815U
#define OV5640_GMTRX34                             0x5816U
#define OV5640_GMTRX35                             0x5817U
#define OV5640_GMTRX40                             0x5818U
#define OV5640_GMTRX41                             0x5819U
#define OV5640_GMTRX42                             0x581AU
#define OV5640_GMTRX43                             0x581BU
#define OV5640_GMTRX44                             0x581CU
#define OV5640_GMTRX45                             0x581DU
#define OV5640_GMTRX50                             0x581EU
#define OV5640_GMTRX51                             0x581FU
#define OV5640_GMTRX52                             0x5820U
#define OV5640_GMTRX53                             0x5821U
#define OV5640_GMTRX54                             0x5822U
#define OV5640_GMTRX55                             0x5823U
#define OV5640_BRMATRX00                           0x5824U
#define OV5640_BRMATRX01                           0x5825U
#define OV5640_BRMATRX02                           0x5826U
#define OV5640_BRMATRX03                           0x5827U
#define OV5640_BRMATRX04                           0x5828U
#define OV5640_BRMATRX05                           0x5829U
#define OV5640_BRMATRX06                           0x582AU
#define OV5640_BRMATRX07                           0x582BU
#define OV5640_BRMATRX08                           0x582CU
#define OV5640_BRMATRX09                           0x582DU
#define OV5640_BRMATRX20                           0x582EU
#define OV5640_BRMATRX21                           0x582FU
#define OV5640_BRMATRX22                           0x5830U
#define OV5640_BRMATRX23                           0x5831U
#define OV5640_BRMATRX24                           0x5832U
#define OV5640_BRMATRX30                           0x5833U
#define OV5640_BRMATRX31                           0x5834U
#define OV5640_BRMATRX32                           0x5835U
#define OV5640_BRMATRX33                           0x5836U
#define OV5640_BRMATRX34                           0x5837U
#define OV5640_BRMATRX40                           0x5838U
#define OV5640_BRMATRX41                           0x5839U
#define OV5640_BRMATRX42                           0x583AU
#define OV5640_BRMATRX43                           0x583BU
#define OV5640_BRMATRX44                           0x583CU
#define OV5640_LENC_BR_OFFSET                      0x583DU
#define OV5640_MAX_GAIN                            0x583EU
#define OV5640_MIN_GAIN                            0x583FU
#define OV5640_MIN_Q                               0x5840U
#define OV5640_LENC_CTRL59                         0x5841U
#define OV5640_BR_HSCALE_HIGH                      0x5842U
#define OV5640_BR_HSCALE_LOW                       0x5843U
#define OV5640_BR_VSCALE_HIGH                      0x5844U
#define OV5640_BR_VSCALE_LOW                       0x5845U
#define OV5640_G_HSCALE_HIGH                       0x5846U
#define OV5640_G_HSCALE_LOW                        0x5847U
#define OV5640_G_VSCALE_HIGH                       0x5848U
#define OV5640_G_VSCALE_LOW                        0x5849U

/* AFC control [0x6000 ~ 0x603F]                         */
#define OV5640_AFC_CTRL00                          0x6000U
#define OV5640_AFC_CTRL01                          0x6001U
#define OV5640_AFC_CTRL02                          0x6002U
#define OV5640_AFC_CTRL03                          0x6003U
#define OV5640_AFC_CTRL04                          0x6004U
#define OV5640_AFC_CTRL05                          0x6005U
#define OV5640_AFC_CTRL06                          0x6006U
#define OV5640_AFC_CTRL07                          0x6007U
#define OV5640_AFC_CTRL08                          0x6008U
#define OV5640_AFC_CTRL09                          0x6009U
#define OV5640_AFC_CTRL10                          0x600AU
#define OV5640_AFC_CTRL11                          0x600BU
#define OV5640_AFC_CTRL12                          0x600CU
#define OV5640_AFC_CTRL13                          0x600DU
#define OV5640_AFC_CTRL14                          0x600EU
#define OV5640_AFC_CTRL15                          0x600FU
#define OV5640_AFC_CTRL16                          0x6010U
#define OV5640_AFC_CTRL17                          0x6011U
#define OV5640_AFC_CTRL18                          0x6012U
#define OV5640_AFC_CTRL19                          0x6013U
#define OV5640_AFC_CTRL20                          0x6014U
#define OV5640_AFC_CTRL21                          0x6015U
#define OV5640_AFC_CTRL22                          0x6016U
#define OV5640_AFC_CTRL23                          0x6017U
#define OV5640_AFC_CTRL24                          0x6018U
#define OV5640_AFC_CTRL25                          0x6019U
#define OV5640_AFC_CTRL26                          0x601AU
#define OV5640_AFC_CTRL27                          0x601BU
#define OV5640_AFC_CTRL28                          0x601CU
#define OV5640_AFC_CTRL29                          0x601DU
#define OV5640_AFC_CTRL30                          0x601EU
#define OV5640_AFC_CTRL31                          0x601FU
#define OV5640_AFC_CTRL32                          0x6020U
#define OV5640_AFC_CTRL33                          0x6021U
#define OV5640_AFC_CTRL34                          0x6022U
#define OV5640_AFC_CTRL35                          0x6023U
#define OV5640_AFC_CTRL36                          0x6024U
#define OV5640_AFC_CTRL37                          0x6025U
#define OV5640_AFC_CTRL38                          0x6026U
#define OV5640_AFC_CTRL39                          0x6027U
#define OV5640_AFC_CTRL40                          0x6028U
#define OV5640_AFC_CTRL41                          0x6029U
#define OV5640_AFC_CTRL42                          0x602AU
#define OV5640_AFC_CTRL43                          0x602BU
#define OV5640_AFC_CTRL44                          0x602CU
#define OV5640_AFC_CTRL45                          0x602DU
#define OV5640_AFC_CTRL46                          0x602EU
#define OV5640_AFC_CTRL47                          0x602FU
#define OV5640_AFC_CTRL48                          0x6030U
#define OV5640_AFC_CTRL49                          0x6031U
#define OV5640_AFC_CTRL50                          0x6032U
#define OV5640_AFC_CTRL51                          0x6033U
#define OV5640_AFC_CTRL52                          0x6034U
#define OV5640_AFC_CTRL53                          0x6035U
#define OV5640_AFC_CTRL54                          0x6036U
#define OV5640_AFC_CTRL55                          0x6037U
#define OV5640_AFC_CTRL56                          0x6038U
#define OV5640_AFC_CTRL57                          0x6039U
#define OV5640_AFC_CTRL58                          0x603AU
#define OV5640_AFC_CTRL59                          0x603BU
#define OV5640_AFC_CTRL60                          0x603CU
#define OV5640_AFC_READ58                          0x603DU
#define OV5640_AFC_READ59                          0x603EU
#define OV5640_AFC_READ60                          0x603FU
// clang-format on

#endif // LIBRARIES_MISCDRIVERS_CAMERA_OV5640_REGS_H_
