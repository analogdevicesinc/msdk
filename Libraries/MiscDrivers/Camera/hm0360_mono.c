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
#include "hm0360_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"

// clang-format off
#define cambus_write(addr, x)     sccb_write_reg16(g_slv_addr, addr, x)
#define cambus_read(addr, x)      sccb_read_reg16(g_slv_addr, addr, x)

#if 1
// Output Resolution: MONO,  CXTA = <320x240>, CXTB = <160x120>
static const uint16_t default_regs[][2] = {
        {   0x8A04  ,   0x01    },
        {   0x8A00  ,   0x22    },
        {   0x8A01  ,   0x00    },
        {   0x8A02  ,   0x01    },
        {   0x0035  ,   0x93    },
        {   0x0036  ,   0x00    },
        {   0x0011  ,   0x09    },
        {   0x0012  ,   0xB6    },
        {   0x0014  ,   0x08    },
        {   0x0015  ,   0x98    },
        {   0x0130  ,   0x03    },
        {   0x0100  ,   0x04    },
        {   0x0150  ,   0x00    },
        {   0x0150  ,   0x04    },
        {   0x0100  ,   0x05    },
        {   0x0C25  ,   0x00    },
        {   0x0C0D  ,   0x00    },
        {   0x0C0E  ,   0x00    },
        {   0x0C11  ,   0x17    },
        {   0x0C14  ,   0x00    },
        {   0x0C18  ,   0x03    },
        {   0x0C20  ,   0x02    },
        {   0x0C22  ,   0x01    },
        {   0x0C09  ,   0x01    },
        {   0x0C24  ,   0x0F    },
        {   0x0C25  ,   0x01    },
        {   0x0C24  ,   0x2F    },
        {   0x0002  ,   0x21    },
        {   0x0003  ,   0x00    },
        {   0x0004  ,   0x1E    },
        {   0x0005  ,   0x03    },
        {   0x0006  ,   0x0F    },
        {   0x0009  ,   0x00    },
        {   0x000D  ,   0x92    },
        {   0x000E  ,   0x20    },
        {   0x0014  ,   0x40    },
        {   0x0015  ,   0x08    },
        {   0x0016  ,   0x00    },
        {   0x0017  ,   0x2A    },
        {   0x0018  ,   0x2A    },
        {   0x001E  ,   0x08    },
        {   0x001F  ,   0x08    },
        {   0x000A  ,   0x0F    },
        {   0x0032  ,   0x00    },
        {   0x0033  ,   0x1E    },
        {   0x0035  ,   0x0F    },
        {   0x0036  ,   0x00    },
        {   0x0019  ,   0x0C    },
        {   0x0037  ,   0x30    },

        //======== Sensor Start ===========
        {   0x0103  ,   0x00    },  // SW_RESET             [0] Software Reset
        {   0x0350  ,   0xE0    },  // -*-*-*-*-*
        {   0x0103  ,   0x00    },  // SW_RESET             [0] Software Reset
        {   0x0350  ,   0xE0    },  // -*-*-*-*-*

        {   0x0370  ,   0x01    },  // MONO_MODE            [9]: Mono mode indicator                            //00
        {   0x0371  ,   0x01    },  // MONO_MODE_ISP        [0]: Mono mode for ISP block                        //00
        {   0x0372  ,   0x01    },  // MONO_MODE_SEL        [0]: Select mono mode indicator from OTP            //01

        {   0x1000  ,   0x43    },  // RESERVED             Set to 1
        {   0x1001  ,   0x80    },  // -*-*-*-*-*
        {   0x1003  ,   0x20    },  // RESERVED             Must be same with BLC_TGT                           //20
        {   0x1004  ,   0x20    },  // BLC_TGT
        {   0x1007  ,   0x01    },  // RESERVED             Set to 1
        {   0x1008  ,   0x20    },  // RESERVED             Must be same with BLC_TGT                           //20
        {   0x1009  ,   0x20    },  // BLC2_TGT
        {   0x100A  ,   0x05    },  // MONO_CTRL            [3:2]:Reserved [1]:Mono mode [0]: Reserved, set to 1    //05 = '101
        {   0x100B  ,   0x20    },  // -*-*-*-*-*
        {   0x100C  ,   0x20    },  // -*-*-*-*-*
        {   0x1013  ,   0x00    },  // -*-*-*-*-*
        {   0x1014  ,   0x00    },  // OPFM_CTRL            Output Format Control                               //00
        {   0x1018  ,   0x00    },
        {   0x101D  ,   0xCF    },  // RESERVED
        {   0x101E  ,   0x01    },  // RESERVED
        {   0x101F  ,   0x00    },  // RESERVED
        {   0x1020  ,   0x01    },  // RESERVED
        {   0x1021  ,   0x5D    },  // RESERVED
        {   0x102F  ,   0x08    },  // CMPRS_CTRL           Compression control                                 //08 = '1000
        {   0x1030  ,   0x09    },  // CMPRS_01
        {   0x1031  ,   0x12    },  // CMPRS_02
        {   0x1032  ,   0x23    },  // CMPRS_03
        {   0x1033  ,   0x31    },  // CMPRS_04
        {   0x1034  ,   0x3E    },  // CMPRS_05
        {   0x1035  ,   0x4B    },  // CMPRS_06
        {   0x1036  ,   0x56    },  // CMPRS_07
        {   0x1037  ,   0x5E    },  // CMPRS_08
        {   0x1038  ,   0x65    },  // CMPRS_99
        {   0x1039  ,   0x72    },  // CMPRS_10
        {   0x103A  ,   0x7F    },  // CMPRS_11
        {   0x103B  ,   0x8C    },  // CMPRS_12
        {   0x103C  ,   0x98    },  // CMPRS_13
        {   0x103D  ,   0xB2    },  // CMPRS_14
        {   0x103E  ,   0xCC    },  // CMPRS_15
        {   0x103F  ,   0xE6    },  // CMPRS_16
        {   0x1041  ,   0x00    },  // -*-*-*-*-*
        {   0x0202  ,   0x01    },  //*INTEGRATION_H
        {   0x0203  ,   0x1c    },  //*INTEGRATION_H
        {   0x0205  ,   0x40    },  //*ANALOG_GAIN          Analog Global Gain code
        {   0x020E  ,   0x01    },  //*DIGITAL_GAIN_H       Digital Global Gain code
        {   0x020F  ,   0x00    },  //*DIGITAL_GAIN_L

        {   0x2000  ,   0x00    },  // AE_CTRL              Auto exposure control
        {   0x202B  ,   0x04    },  // MAX_AGAIN            AE max AGAIN allowance
        {   0x202C  ,   0x03    },  // MAX_DGAIN_H          AE max DGAIN allowance H (must be >=2)
        {   0x202D  ,   0x00    },  // MAX_DGAIN_L
        {   0x2031  ,   0x60    },  // T_DAMPING
        {   0x2032  ,   0x08    },  // N_DAMPING
        {   0x2036  ,   0x19    },  // AE_TARGETZONE
        {   0x2037  ,   0x08    },  // CONVERGE_IN_TH
        {   0x2038  ,   0x10    },  // CONVERGE_OUT_TH
        {   0x203C  ,   0x01    },  // FS_60_HZ_H           AE flicker step H (60Hz)
        {   0x203D  ,   0x04    },  // FS_60_HZ_H           AE flicker step L (60Hz)
        {   0x203E  ,   0x01    },  // FS_50_HZ_H           AE flicker step H (50Hz)
        {   0x203F  ,   0x38    },  // FS_50_HZ_H           AE flicker step L (50Hz)
        {   0x2048  ,   0x00    },
        {   0x2049  ,   0x10    },
        {   0x204A  ,   0x40    },
        {   0x204B  ,   0x00    },
        {   0x204C  ,   0x08    },
        {   0x204D  ,   0x20    },
        {   0x204E  ,   0x00    },  // FR_EGPTH23_H
        {   0x204F  ,   0x38    },  // FR_EGPTH23_M
        {   0x2050  ,   0xE0    },  // FR_EGPTH23_L
        {   0x2051  ,   0x00    },  // FR_EGPTH32_H
        {   0x2052  ,   0x1C    },  // FR_EGPTH32_M
        {   0x2053  ,   0x70    },  // FR_EGPTH32_L
        {   0x2054  ,   0x00    },  // RESERVED
        {   0x2055  ,   0x1A    },  // RESERVED
        {   0x2056  ,   0xC0    },  // RESERVED
        {   0x2057  ,   0x00    },  // RESERVED
        {   0x2058  ,   0x06    },  // RESERVED
        {   0x2059  ,   0xB0    },  // RESERVED
        {   0x2061  ,   0x00    },  // PULSE_MODE           [0]:  0: INT level mode  1: INT pulse mode
        {   0x2062  ,   0x00    },  // PULSE_TH_H           INT pulse width
        {   0x2063  ,   0xC8    },  // PULSE_TH_L           INT pulse with
        {   0x2080  ,   0x40    },  // MD_CTRL              [7:1] Motion detect light coefficient [0] Motion detect enable
        {   0x2081  ,   0xE0    },  // ROI_START_END_V
        {   0x2082  ,   0xF0    },  // ROI_START_END_H
        {   0x2083  ,   0x01    },  // MD_TH_MIN
        {   0x2084  ,   0x10    },  // MD_TH_STR_L
        {   0x2085  ,   0x10    },  // MD_TH_STR_H
        {   0x2086  ,   0x01    },  // MD_TH_COEF_0         Motion detect threshold coefficient 0
        {   0x2087  ,   0x06    },  // MD_TH_COEF_1         Motion detect threshold coefficient 1
        {   0x2088  ,   0x0C    },  // MD_TH_COEF_2         Motion detect threshold coefficient 2
        {   0x2089  ,   0x12    },  // MD_TH_COEF_3         Motion detect threshold coefficient 3
        {   0x208A  ,   0x1C    },  // MD_TH_COEF_4         Motion detect threshold coefficient 4
        {   0x208B  ,   0x30    },  // MD_TH_COEF_5         Motion detect threshold coefficient 5
        {   0x208C  ,   0x10    },  // RESERVED
        {   0x208D  ,   0x02    },  // RESERVED
        {   0x208E  ,   0x08    },  // RESERVED
        {   0x208F  ,   0x0D    },  // RESERVED
        {   0x2090  ,   0x14    },  // RESERVED
        {   0x2091  ,   0x1D    },  // RESERVED
        {   0x2092  ,   0x30    },  // RESERVED
        {   0x2093  ,   0x08    },  // MD_TG_COEFF_1
        {   0x2094  ,   0x0A    },  // MD_TG_COEFF_2
        {   0x2095  ,   0x0F    },  // MD_TG_COEFF_3
        {   0x2096  ,   0x14    },  // MD_TG_COEFF_4
        {   0x2097  ,   0x18    },  // MD_TG_COEFF_5
        {   0x2098  ,   0x20    },  // MD_TG_COEFF_6
        {   0x2099  ,   0x10    },  // MD_LIGHT_COEF
        {   0x209A  ,   0x00    },  // MD_IIR_PARAMETER
        {   0x209B  ,   0x01    },  // MD_BLOCK_NUM_TH
        {   0x209C  ,   0x01    },  // MD_LATENCY
        {   0x209D  ,   0x11    },  // MD_LATENCY_TH
        {   0x209E  ,   0x06    },  // MD_CTRL1
        {   0x209F  ,   0x20    },  // -*-*-*-*-*
        {   0x20A0  ,   0x10    },  // -*-*-*-*-*
        {   0x2590  ,   0x01    },  // -*-*-*-*-*
        {   0x2800  ,   0x00    },  // MIPI_EN              [4]:RESERVED [3]:Line Gate [2]:Grame Gate [1] RESERVED [0]:MIPI Enable
        {   0x2804  ,   0x02    },  // RESERVED
        {   0x2805  ,   0x03    },  // RESERVED
        {   0x2806  ,   0x03    },  // RESERVED
        {   0x2807  ,   0x08    },  // RESERVED
        {   0x2808  ,   0x04    },  // RESERVED
        {   0x2809  ,   0x00C   },  // RESERVED
        {   0x280A  ,   0x03    },  // RESERVED
        {   0x280F  ,   0x03    },  // RESERVED
        {   0x2810  ,   0x03    },  // RESERVED
        {   0x2811  ,   0x00    },  // RESERVED
        {   0x2812  ,   0x09    },  // RESERVED
        {   0x2821  ,   0xDE    },  // RESERVED
        {   0x282A  ,   0x0F    },  // RESERVED
        {   0x282B  ,   0x08    },  // RESERVED
        {   0x282E  ,   0x2F    },  // RESERVED

        {   0x3010  ,   0x00    },  // EXP_SYNC_CFG
        {   0x3013  ,   0x01    },  // ERR_FLAG_CFG
        {   0x3019  ,   0x00    },  // OFFSET_RDSYNC_H      Adjust the delay between FVLD and FSIN input signal
        {   0x301A  ,   0x00    },  // OFFSET_RDSYNC_L
        {   0x301B  ,   0x20    },  // RDSYNC_DEC_TH_H      Threshold for Out-of-sync in read synchronization mode
        {   0x301C  ,   0xFF    },  // RDSYNC_DEC_TH_L
        {   0x3020  ,   0x00    },
        {   0x3021  ,   0x00    },
        {   0x3024  ,   0x00    },  // PMU_CFG_3    [3]:CTX disable [1]:Pad Sel enable. 0:SW    [0]: 0:Context A  1:Context B   //00
        {   0x3025  ,   0x12    },  // PMU_CFG4     {7:4] Conext B frame counter  [3:0] Context A frame counter
        {   0x3026  ,   0x03    },  // PMU_CFG5
        {   0x3027  ,   0x81    },  // PMU_CFG6
        {   0x3028  ,   0x01    },  // PMU_CFG7
        {   0x3029  ,   0x00    },  // PMU_CFG8
        {   0x302A  ,   0x30    },  // PMU_CFG9
        {   0x302B  ,   0x2A    },  // RESERVED
        {   0x302C  ,   0x00    },  // RESERVED
        {   0x302D  ,   0x03    },  // RESERVED
        {   0x302E  ,   0x00    },  // PMU_CFG_13
        {   0x302F  ,   0x00    },  // PMU_CFG_14
        {   0x3030  ,   0x01    },  //*WIN_MODE             [0]:Pixel Window                0: 656x496  1: 640x480
        {   0x3031  ,   0x01    },  // RESERVED
        {   0x3034  ,   0x00    },  // -*-*-*-*-*
        {   0x3035  ,   0x01    },  // -*-*-*-*-*
        {   0x3051  ,   0x00    },  // -*-*-*-*-*
        {   0x305C  ,   0x03    },  // -*-*-*-*-*
        {   0x3060  ,   0x00    },  // ROI_CFG              [0]:ROI Enable
        {   0x3061  ,   0xFA    },  // ROI_WIN_NUMBER
        {   0x3062  ,   0xFF    },  // ROI_WIN_ONE_H
        {   0x3063  ,   0xFF    },  // ROI_WIN_ONE_L
        {   0x3064  ,   0xFF    },  // ROI_WIN_TWO_H
        {   0x3065  ,   0xFF    },  // ROI_WIN_TWO_L
        {   0x3066  ,   0xFF    },
        {   0x3067  ,   0xFF    },
        {   0x3068  ,   0xFF    },
        {   0x3069  ,   0xFF    },
        {   0x306A  ,   0xFF    },
        {   0x306B  ,   0xFF    },
        {   0x306C  ,   0xFF    },
        {   0x306D  ,   0xFF    },
        {   0x306E  ,   0xFF    },
        {   0x306F  ,   0xFF    },
        {   0x3070  ,   0xFF    },
        {   0x3071  ,   0xFF    },
        {   0x3072  ,   0xFF    },
        {   0x3073  ,   0xFF    },
        {   0x3074  ,   0xFF    },
        {   0x3075  ,   0xFF    },
        {   0x3076  ,   0xFF    },
        {   0x3077  ,   0xFF    },
        {   0x3078  ,   0xFF    },
        {   0x3079  ,   0xFF    },
        {   0x307A  ,   0xFF    },
        {   0x307B  ,   0xFF    },
        {   0x307C  ,   0xFF    },  // ROI_WIN_FOURTEEN_H
        {   0x307D  ,   0xFF    },  // ROI_WIN_FOURTEEN_L
        {   0x307E  ,   0xFF    },  // ROI_WIN_FIFTEEN_H
        {   0x307F  ,   0xFF    },  // ROI_WIN_FIFTEEN_L
        {   0x3080  ,   0x00    },  // STROBE_CFG
        {   0x3081  ,   0x00    },  // STROBE_SEL
        {   0x3082  ,   0x00    },
        {   0x3083  ,   0x20    },
        {   0x3084  ,   0x00    },
        {   0x3085  ,   0x20    },
        {   0x3086  ,   0x00    },  // STROBE_LINE_H
        {   0x3087  ,   0x20    },  // STROBE_LINE_L
        {   0x3088  ,   0x00    },  // STROBE_FRAME_H
        {   0x3089  ,   0x04    },  // STROBE_FRAME_L
        {   0x3094  ,   0x02    },  // VSYNC_FRONT              Early VSYNC front porch register
        {   0x3095  ,   0x02    },  // VSYNC_END                Early VSYNC end porch register
        {   0x3096  ,   0x00    },  // HSYNC_FRONT_H            Early HSYNC front porch register
        {   0x3097  ,   0x02    },  // HSYNC_FRONT_L            Early HSYNC front porch register
        {   0x3098  ,   0x00    },  // HSYNC_END_H              Early HSYNC end porch register
        {   0x3099  ,   0x02    },  // HSYNC_END_L              Early HSYNC end porch register
        {   0x309E  ,   0x05    },  // PCLKO_GATED_EN           [1}:Gated by line [0]:Gated by frame
        {   0x309F  ,   0x02    },  // PCLKO_FRAME_FRONT        PCLKO frame based front porch register (row adjustment)
        {   0x30A0  ,   0x02    },  // PCLKO_FRAME_END          PCLKO frame based end porch register (row adjustment)
        {   0x30A1  ,   0x00    },  // PCLKO_LINE_FRONT_H       PCLKO line based front porch register (clock adjustment)
        {   0x30A2  ,   0x08    },  // PCLKO_LINE_FRONT_L       PCLKO line based front porch register (clock adjustment)
        {   0x30A3  ,   0x00    },  // PCLKO_LINE_END_H         PCLKO line based end porch register (clock adjustment)
        {   0x30A4  ,   0x20    },  // PCLKO_LINE_END_L         PCLKO line based end porch register (clock adjustment)
        {   0x30A5  ,   0x04    },  // OUTPUT_EN                [2]:PCLKO continuous mode [1] Trigger on/off mode  [0]:VSYNC mode
        {   0x30A6  ,   0x02    },  // -*-*-*-*-*
        {   0x30A7  ,   0x02    },  // -*-*-*-*-*
        {   0x30A8  ,   0x01    },  // FRAME_OUTPUT_EN          [2]:Mask out enable of AE non-converged frame [1]Mask out for MIPI [0]:Mask out for Parallel
        {   0x30A9  ,   0x00    },  // MULTI_CAMERA_CONFIG      [1]:MODE2 [0]:MODE1
        {   0x30AA  ,   0x02    },  // MULTI_CAMERA_TUNE_H
        {   0x30AB  ,   0x34    },  // MULTI_CAMERA_TUNE_L
        {   0x30B0  ,   0x03    },  // PAD_OUTPUT_EN
        {   0x30C4  ,   0x10    },  // RESERVED
        {   0x30C5  ,   0x01    },  // RESERVED
        {   0x30C6  ,   0xBF    },  // RESERVED
        {   0x30C7  ,   0x00    },  // RESERVED
        {   0x30C8  ,   0x00    },  // RESERVED
        {   0x30CB  ,   0xFF    },  // RESERVED
        {   0x30CC  ,   0xFF    },  // RESERVED
        {   0x30CD  ,   0x7F    },  // RESERVED
        {   0x30CE  ,   0x7F    },  // RESERVED
        {   0x30D3  ,   0x01    },  // RESERVED
        {   0x30D4  ,   0xFF    },  // RESERVED
        {   0x30D5  ,   0x00    },  // RESERVED
        {   0x30D6  ,   0x40    },  // RESERVED
        {   0x30D7  ,   0x00    },  // RESERVED
        {   0x30D8  ,   0xA7    },  // RESERVED
        {   0x30D9  ,   0x05    },  // RESERVED
        {   0x30DA  ,   0x01    },  // RESERVED
        {   0x30DB  ,   0x40    },  // RESERVED
        {   0x30DC  ,   0x00    },  // RESERVED
        {   0x30DD  ,   0x27    },  // RESERVED
        {   0x30DE  ,   0x05    },  // RESERVED
        {   0x30DF  ,   0x07    },  // RESERVED
        {   0x30E0  ,   0x40    },  // RESERVED
        {   0x30E1  ,   0x00    },  // RESERVED
        {   0x30E2  ,   0x27    },  // RESERVED
        {   0x30E3  ,   0x05    },  // RESERVED
        {   0x30E4  ,   0x47    },  // RESERVED
        {   0x30E5  ,   0x30    },  // RESERVED
        {   0x30E6  ,   0x00    },  // RESERVED
        {   0x30E7  ,   0x27    },  // RESERVED
        {   0x30E8  ,   0x05    },  // RESERVED
        {   0x30E9  ,   0x87    },  // RESERVED
        {   0x30EA  ,   0x30    },  // RESERVED
        {   0x30EB  ,   0x00    },  // RESERVED
        {   0x30EC  ,   0x27    },  // RESERVED
        {   0x30ED  ,   0x05    },  // RESERVED
        {   0x30EE  ,   0x00    },  // RESERVED
        {   0x30EF  ,   0x40    },  // RESERVED
        {   0x30F0  ,   0x00    },  // RESERVED
        {   0x30F1  ,   0xA7    },  // RESERVED
        {   0x30F2  ,   0x05    },  // RESERVED
        {   0x30F3  ,   0x01    },  // RESERVED
        {   0x30F4  ,   0x40    },  // RESERVED
        {   0x30F5  ,   0x00    },  // RESERVED
        {   0x30F6  ,   0x27    },  // RESERVED
        {   0x30F7  ,   0x05    },  // RESERVED
        {   0x30F8  ,   0x07    },  // RESERVED
        {   0x30F9  ,   0x40    },  // RESERVED
        {   0x30FA  ,   0x00    },  // RESERVED
        {   0x30FB  ,   0x27    },  // RESERVED
        {   0x30FC  ,   0x05    },  // RESERVED
        {   0x30FD  ,   0x47    },  // RESERVED
        {   0x30FE  ,   0x30    },  // RESERVED
        {   0x30FF  ,   0x00    },  // RESERVED
        {   0x3100  ,   0x27    },  // RESERVED
        {   0x3101  ,   0x05    },  // RESERVED
        {   0x3102  ,   0x87    },  // RESERVED
        {   0x3103  ,   0x30    },  // RESERVED
        {   0x3104  ,   0x00    },  // RESERVED
        {   0x3105  ,   0x27    },  // RESERVED
        {   0x3106  ,   0x05    },  // RESERVED
        {   0x310B  ,   0x10    },  // RESERVED
        {   0x3112  ,   0x04    },  // PAD_REGISTER_07
        {   0x3113  ,   0xA0    },  // -*-*-*-*-*
        {   0x3114  ,   0x67    },  // -*-*-*-*-*
        {   0x3115  ,   0x42    },  // -*-*-*-*-*
        {   0x3116  ,   0x10    },  // -*-*-*-*-*
        {   0x3117  ,   0x0A    },  // -*-*-*-*-*
        {   0x3118  ,   0x3F    },  // -*-*-*-*-*
        //{ 0x311A  ,   0x31    },  // ANA_REGISTER15       [0]: Disable internal LDO
        {   0x311C  ,   0x10    },  // RESERVED
        {   0x311D  ,   0x06    },  // RESERVED
        {   0x311E  ,   0x0F    },  // RESERVED
        {   0x311F  ,   0x0E    },  // RESERVED
        {   0x3120  ,   0x0D    },  // RESERVED
        {   0x3121  ,   0x0F    },  // RESERVED
        {   0x3122  ,   0x00    },  // RESERVED
        {   0x3123  ,   0x1D    },  // RESERVED
        {   0x3126  ,   0x03    },  // RESERVED
        {   0x3128  ,   0x57    },  // PLL_POST_DIV_D
        {   0x312A  ,   0x11    },  // RESERVED
        {   0x312B  ,   0x41    },  // RESERVED
        {   0x312E  ,   0x00    },  // RESERVED
        {   0x312F  ,   0x00    },  // RESERVED
        {   0x3130  ,   0x0C    },  // RESERVED
        {   0x3141  ,   0x2A    },  // RESERVED
        {   0x3142  ,   0x9F    },  // RESERVED
        {   0x3147  ,   0x18    },  // RESERVED
        {   0x3149  ,   0x18    },  // RESERVED
        {   0x314B  ,   0x01    },  // RESERVED
        {   0x3150  ,   0x50    },  // RESERVED
        {   0x3152  ,   0x00    },  // RESERVED
        {   0x3156  ,   0x2C    },  // RESERVED
        {   0x315A  ,   0x0A    },  // RESERVED
        {   0x315B  ,   0x2F    },  // RESERVED
        {   0x315C  ,   0xE0    },  // RESERVED
        {   0x315F  ,   0x02    },  // RESERVED
        {   0x3160  ,   0x1F    },  // RESERVED
        {   0x3163  ,   0x1F    },  // RESERVED
        {   0x3164  ,   0x7F    },  // RESERVED
        {   0x3165  ,   0x7F    },  // RESERVED
        {   0x317B  ,   0x94    },  // RESERVED
        {   0x317C  ,   0x00    },  // RESERVED
        {   0x317D  ,   0x02    },  // RESERVED
        {   0x318C  ,   0x00    },  // RESERVED


    // CONTEXT A START
    //
    {   0x3500  ,   0x03    },  // PLL1CFG                  //74
    {   0x3501  ,   0x0A    },  // PLL2CFG
    {   0x3502  ,   0x77    },  // PLL3CFG

    {   0x3503  ,   0x01    },  // FRAME_LENGTH_LINES_H                                                         //01
    {   0x3504  ,   0x1c    },  // FRAME_LENGTH_LINES_L                                                         //E0

    {   0x3505  ,   0x03    },  // LINE_LENGTH_PCK_H                                                            //03
    {   0x3506  ,   0x00    },  // LINE_LENGTH_PCK_L                                                            //00

    {   0x3507  ,   0x01    },  // H_SUB                [1:0]: Horizontal                                       //1
    {   0x3508  ,   0x01    },  // V_SUB                [1:0]: Vertical                                         //1
    {   0x3509  ,   0x03    },  // BIN_MODE             [1]:H Binning [0]:V Binning                             //0
    {   0x350A  ,   0x55    },  // RESERVED                 //55
    {   0x350B  ,   0x01    },  // MONO_MODE_ISP        [0]:Mono Mode for ISP block
    {   0x350C  ,   0x00    },  // N_PLUS_MODE_EN       [0]:N+1 CMU Update
    {   0x350D  ,   0x01    },  // WINMODE              [0]:Pixel Window                0: 656x496  1: 640x480
    {   0x350F  ,   0x00    },  // EARLY_INT_EN         [0]:Early Interrupt Enable
    {   0x3510  ,   0x01    },  // FRAME_OUTPUT_EN
    {   0x3512  ,   0x7F    },
    {   0x3513  ,   0x00    },
    {   0x3514  ,   0x00    },
    {   0x3515  ,   0x01    },
    {   0x3516  ,   0x00    },
    {   0x3517  ,   0x02    },
    {   0x3518  ,   0x00    },
    {   0x3519  ,   0x3F    },
    {   0x351A  ,   0x00    },
    {   0x351B  ,   0x2F    },
    {   0x351C  ,   0x00    },
    {   0x351D  ,   0x04    },
    {   0x351E  ,   0x24    },
    {   0x351F  ,   0x04    },
    {   0x3520  ,   0x03    },
    {   0x3521  ,   0x00    },
    {   0x3523  ,   0x60    },
    {   0x3524  ,   0x08    },
    {   0x3525  ,   0x19    },
    {   0x3526  ,   0x08    },
    {   0x3527  ,   0x10    },
    {   0x352A  ,   0x01    },
    {   0x352B  ,   0x04    },
    {   0x352C  ,   0x01    },
    {   0x352D  ,   0x38    },
    {   0x3535  ,   0x02    },
    {   0x3536  ,   0x03    },
    {   0x3537  ,   0x03    },
    {   0x3538  ,   0x08    },
    {   0x3539  ,   0x04    },
    {   0x353A  ,   0x0C    },
    {   0x353B  ,   0x03    },
    {   0x3540  ,   0x03    },
    {   0x3541  ,   0x03    },
    {   0x3542  ,   0x00    },
    {   0x3543  ,   0x09    },
    {   0x3549  ,   0x02    },
    {   0x354A  ,   0x03    },
    {   0x354B  ,   0x21    },
    {   0x354C  ,   0x01    },
    {   0x354D  ,   0xE0    },
    {   0x354E  ,   0xF0    },
    {   0x354F  ,   0x10    },
    {   0x3550  ,   0x10    },
    {   0x3551  ,   0x10    },
    {   0x3552  ,   0x20    },
    {   0x3553  ,   0x10    },
    {   0x3554  ,   0x01    },
    {   0x3555  ,   0x06    },
    {   0x3556  ,   0x0C    },
    {   0x3557  ,   0x12    },
    {   0x3558  ,   0x1C    },
    {   0x3559  ,   0x30    },

// Start of Context B
//{0x355A, 0x74},  // clk_tb_div[1:0]=/1, plcko_div[3:2]=/1, clk_i2c_div[5:4]=/8, pll_pre_div=1
    {0x355A, 0x7c},  // clk_tb_div[1:0]=/1, plcko_div[3:2]=/4, clk_i2c_div[5:4]=/8, pll_pre_div=1,  pclk=6MHz
//{0x355A, 0x33},  // clk_tb_div[1:0]=/8, plcko_div[3:2]=/1, clk_i2c_div[5:4]=/8, pll_pre_div=0, pclk=3MHz
//{0x355A, 0x03},  // clk_tb_div[1:0]=/8, plcko_div[3:2]=/1, clk_i2c_div[5:4]=/2, pll_pre_div=0, pclk=3MHz

    {0x355B, 0x0A},
    {0x355C, 0x77},
    {0x355D, 0x00},
    {0x355E, 0x8E},
    {0x355F, 0x03},
    {0x3560, 0x00},
    {0x3561, 0x02},
    {0x3562, 0x02},
    {0x3563, 0x03},
    {0x3564, 0x11},
    {0x3565, 0x01},
    {0x3566, 0x00},
    {0x3567, 0x01},
    {0x3569, 0x00},
    {0x356A, 0x01},
    {0x356C, 0x7F},
    {0x356D, 0x00},
    {0x356E, 0x00},
    {0x356F, 0x01},
    {0x3570, 0x00},
    {0x3571, 0x02},
    {0x3572, 0x00},
    {0x3573, 0x1F},
    {0x3574, 0x00},
    {0x3575, 0x17},
    {0x3576, 0x00},
    {0x3577, 0x04},
    {0x3578, 0x24},
    {0x3579, 0x04},
    {0x357A, 0x03},
    {0x357B, 0x00},
    {0x357D, 0x60},
    {0x357E, 0x08},
    {0x357F, 0x19},
    {0x3580, 0x08},
    {0x3581, 0x10},
    {0x3584, 0x01},
    {0x3585, 0x04},
    {0x3586, 0x01},
    {0x3587, 0x38},
    {0x3588, 0x02},
    {0x3589, 0x12},
    {0x358A, 0x04},
    {0x358B, 0x24},
    {0x358C, 0x06},
    {0x358D, 0x36},
    {0x358F, 0x02},
    {0x3590, 0x03},
    {0x3591, 0x03},
    {0x3592, 0x08},
    {0x3593, 0x04},
    {0x3594, 0x0C},
    {0x3595, 0x03},
    {0x359A, 0x03},
    {0x359B, 0x03},
    {0x359C, 0x00},
    {0x359D, 0x09},
    {0x35A3, 0x00},
    {0x35A4, 0xEA},
    {0x35A5, 0x21},
    {0x35A6, 0x01},
    {0x35A7, 0xD0},
    {0x35A8, 0xF0},
    {0x35A9, 0x10},
    {0x35AA, 0x10},
    {0x35AB, 0x10},
    {0x35AC, 0x20},
    {0x35AD, 0x10},
    {0x35AE, 0x01},
    {0x35AF, 0x06},
    {0x35B0, 0x0C},
    {0x35B1, 0x12},
    {0x35B2, 0x1C},
    {0x35B3, 0x30},
    {0x35B4, 0x74},
    {0x35B5, 0x0A},
    {0x35B6, 0x77},
    {0x35B7, 0x00},
    {0x35B8, 0x94},
    {0x35B9, 0x03},
    {0x35BA, 0x00},
    {0x35BB, 0x03},
    {0x35BC, 0x03},
    {0x35BD, 0x03},
    {0x35BE, 0x01},
    {0x35BF, 0x01},
    {0x35C0, 0x01},
    {0x35C1, 0x01},
    {0x35C3, 0x00},
    {0x35C4, 0x00},
    {0x35C6, 0x7F},
    {0x35C7, 0x00},
    {0x35C8, 0x00},
    {0x35C9, 0x01},
    {0x35CA, 0x00},
    {0x35CB, 0x02},
    {0x35CC, 0x00},
    {0x35CD, 0x0F},
    {0x35CE, 0x00},
    {0x35CF, 0x0B},
    {0x35D0, 0x00},
    {0x35D3, 0x04},
    {0x35D7, 0x18},
    {0x35D8, 0x01},
    {0x35D9, 0x20},
    {0x35DA, 0x08},
    {0x35DB, 0x14},
    {0x35DC, 0x70},
    {0x35DE, 0x00},
    {0x35DF, 0x01},
    {0x35E9, 0x02},
    {0x35EA, 0x03},
    {0x35EB, 0x03},
    {0x35EC, 0x08},
    {0x35ED, 0x04},
    {0x35EE, 0x0C},
    {0x35EF, 0x03},
    {0x35F4, 0x03},
    {0x35F5, 0x03},
    {0x35F6, 0x00},
    {0x35F7, 0x09},
    {0x35FD, 0x00},
    {0x35FE, 0x5E},  // End of Context B

//{0x0601, 0x01},  // test pattern: color bar
//{0x0601, 0x21},  // test pattern: walking ones

    {0x0104, 0x01},
    {0x0100, 0x01},



    {0xFFFF, 0xFF} // end of settings
};
#else
// Output Resolution: MONO, CXTA = <640x480>, CXTB = <320x240>
static const uint16_t default_regs[][2] = {
//{0x0103, 0x00},
    {0x0350, 0xE0},
    {0x0370, 0x01},
    {0x0371, 0x01},
    {0x0372, 0x00},
    {0x1000, 0x43},
    {0x1001, 0x80},
    {0x1003, 0x20},
    {0x1004, 0x20},
    {0x1007, 0x01},
    {0x1008, 0x20},
    {0x1009, 0x20},
    {0x100A, 0x07},
    {0x100B, 0x20},
    {0x100C, 0x20},
    {0x1013, 0x00},
    {0x1014, 0x00},
    {0x1018, 0x00},
    {0x101D, 0xCF},
    {0x101E, 0x01},
    {0x101F, 0x00},
    {0x1020, 0x01},
    {0x1021, 0x5D},
    {0x102F, 0x08},
    {0x1030, 0x09},
    {0x1031, 0x12},
    {0x1032, 0x23},
    {0x1033, 0x31},
    {0x1034, 0x3E},
    {0x1035, 0x4B},
    {0x1036, 0x56},
    {0x1037, 0x5E},
    {0x1038, 0x65},
    {0x1039, 0x72},
    {0x103A, 0x7F},
    {0x103B, 0x8C},
    {0x103C, 0x98},
    {0x103D, 0xB2},
    {0x103E, 0xCC},
    {0x103F, 0xE6},
    {0x1041, 0x00},
    {0x2000, 0x7F},
    {0x202B, 0x04},
    {0x202C, 0x03},
    {0x202D, 0x00},
    {0x2031, 0x60},
    {0x2032, 0x08},
    {0x2036, 0x19},
    {0x2037, 0x08},
    {0x2038, 0x10},
    {0x203C, 0x01},
    {0x203D, 0x04},
    {0x203E, 0x01},
    {0x203F, 0x38},
    {0x2048, 0x00},
    {0x2049, 0x10},
    {0x204A, 0x40},
    {0x204B, 0x00},
    {0x204C, 0x08},
    {0x204D, 0x20},
    {0x204E, 0x00},
    {0x204F, 0x38},
    {0x2050, 0xE0},
    {0x2051, 0x00},
    {0x2052, 0x1C},
    {0x2053, 0x70},
    {0x2054, 0x00},
    {0x2055, 0x1A},
    {0x2056, 0xC0},
    {0x2057, 0x00},
    {0x2058, 0x06},
    {0x2059, 0xB0},
    {0x2061, 0x00},
    {0x2062, 0x00},
    {0x2063, 0xC8},
    {0x2080, 0x41},
    {0x2081, 0xE0},
    {0x2082, 0xF0},
    {0x2083, 0x01},
    {0x2084, 0x10},
    {0x2085, 0x10},
    {0x2086, 0x01},
    {0x2087, 0x06},
    {0x2088, 0x0C},
    {0x2089, 0x12},
    {0x208A, 0x1C},
    {0x208B, 0x30},
    {0x208C, 0x10},
    {0x208D, 0x02},
    {0x208E, 0x08},
    {0x208F, 0x0D},
    {0x2090, 0x14},
    {0x2091, 0x1D},
    {0x2092, 0x30},
    {0x2093, 0x08},
    {0x2094, 0x0A},
    {0x2095, 0x0F},
    {0x2096, 0x14},
    {0x2097, 0x18},
    {0x2098, 0x20},
    {0x2099, 0x10},
    {0x209A, 0x00},
    {0x209B, 0x01},
    {0x209C, 0x01},
    {0x209D, 0x11},
    {0x209E, 0x06},
    {0x209F, 0x20},
    {0x20A0, 0x10},
    {0x2590, 0x01},
    {0x2800, 0x00},
    {0x2804, 0x02},
    {0x2805, 0x03},
    {0x2806, 0x03},
    {0x2807, 0x08},
    {0x2808, 0x04},
    {0x2809, 0x0C},
    {0x280A, 0x03},
    {0x280F, 0x03},
    {0x2810, 0x03},
    {0x2811, 0x00},
    {0x2812, 0x09},
    {0x2821, 0xDE},
    {0x282A, 0x0F},
    {0x282B, 0x08},
    {0x282E, 0x2F},
    {0x3010, 0x00},
    {0x3013, 0x01},
    {0x3019, 0x00},
    {0x301A, 0x00},
    {0x301B, 0x20},
    {0x301C, 0xFF},
    {0x3020, 0x00},
    {0x3021, 0x00},
    {0x3024, 0x00},
    {0x3025, 0x12},
    {0x3026, 0x03},
    {0x3027, 0x81},
    {0x3028, 0x01},
    {0x3029, 0x00},
    {0x302A, 0x30},
    {0x302B, 0x2A},
    {0x302C, 0x00},
    {0x302D, 0x03},
    {0x302E, 0x00},
    {0x302F, 0x00},
    {0x3031, 0x01},
    {0x3034, 0x00},
    {0x3035, 0x01},
    {0x3051, 0x00},
    {0x305C, 0x03},
    {0x3060, 0x00},
    {0x3061, 0xFA},
    {0x3062, 0xFF},
    {0x3063, 0xFF},
    {0x3064, 0xFF},
    {0x3065, 0xFF},
    {0x3066, 0xFF},
    {0x3067, 0xFF},
    {0x3068, 0xFF},
    {0x3069, 0xFF},
    {0x306A, 0xFF},
    {0x306B, 0xFF},
    {0x306C, 0xFF},
    {0x306D, 0xFF},
    {0x306E, 0xFF},
    {0x306F, 0xFF},
    {0x3070, 0xFF},
    {0x3071, 0xFF},
    {0x3072, 0xFF},
    {0x3073, 0xFF},
    {0x3074, 0xFF},
    {0x3075, 0xFF},
    {0x3076, 0xFF},
    {0x3077, 0xFF},
    {0x3078, 0xFF},
    {0x3079, 0xFF},
    {0x307A, 0xFF},
    {0x307B, 0xFF},
    {0x307C, 0xFF},
    {0x307D, 0xFF},
    {0x307E, 0xFF},
    {0x307F, 0xFF},
    {0x3080, 0x00},
    {0x3081, 0x00},
    {0x3082, 0x00},
    {0x3083, 0x20},
    {0x3084, 0x00},
    {0x3085, 0x20},
    {0x3086, 0x00},
    {0x3087, 0x20},
    {0x3088, 0x00},
    {0x3089, 0x04},
    {0x3094, 0x02},
    {0x3095, 0x02},
    {0x3096, 0x00},
    {0x3097, 0x02},
    {0x3098, 0x00},
    {0x3099, 0x02},
    {0x309E, 0x05},
    {0x309F, 0x02},
    {0x30A0, 0x02},
    {0x30A1, 0x00},
    {0x30A2, 0x08},
    {0x30A3, 0x00},
    {0x30A4, 0x20},
    {0x30A5, 0x04},
    {0x30A6, 0x02},
    {0x30A7, 0x02},
    {0x30A8, 0x01},
    {0x30A9, 0x00},
    {0x30AA, 0x02},
    {0x30AB, 0x34},
    {0x30B0, 0x03},
    {0x30C4, 0x10},
    {0x30C5, 0x01},
    {0x30C6, 0xBF},
    {0x30C7, 0x00},
    {0x30C8, 0x00},
    {0x30CB, 0xFF},
    {0x30CC, 0xFF},
    {0x30CD, 0x7F},
    {0x30CE, 0x7F},
    {0x30D3, 0x01},
    {0x30D4, 0xFF},
    {0x30D5, 0x00},
    {0x30D6, 0x40},
    {0x30D7, 0x00},
    {0x30D8, 0xA7},
    {0x30D9, 0x05},
    {0x30DA, 0x01},
    {0x30DB, 0x40},
    {0x30DC, 0x00},
    {0x30DD, 0x27},
    {0x30DE, 0x05},
    {0x30DF, 0x07},
    {0x30E0, 0x40},
    {0x30E1, 0x00},
    {0x30E2, 0x27},
    {0x30E3, 0x05},
    {0x30E4, 0x47},
    {0x30E5, 0x30},
    {0x30E6, 0x00},
    {0x30E7, 0x27},
    {0x30E8, 0x05},
    {0x30E9, 0x87},
    {0x30EA, 0x30},
    {0x30EB, 0x00},
    {0x30EC, 0x27},
    {0x30ED, 0x05},
    {0x30EE, 0x00},
    {0x30EF, 0x40},
    {0x30F0, 0x00},
    {0x30F1, 0xA7},
    {0x30F2, 0x05},
    {0x30F3, 0x01},
    {0x30F4, 0x40},
    {0x30F5, 0x00},
    {0x30F6, 0x27},
    {0x30F7, 0x05},
    {0x30F8, 0x07},
    {0x30F9, 0x40},
    {0x30FA, 0x00},
    {0x30FB, 0x27},
    {0x30FC, 0x05},
    {0x30FD, 0x47},
    {0x30FE, 0x30},
    {0x30FF, 0x00},
    {0x3100, 0x27},
    {0x3101, 0x05},
    {0x3102, 0x87},
    {0x3103, 0x30},
    {0x3104, 0x00},
    {0x3105, 0x27},
    {0x3106, 0x05},
    {0x310B, 0x10},
    {0x3112, 0x04},
    {0x3113, 0xA0},
    {0x3114, 0x67},
    {0x3115, 0x42},
    {0x3116, 0x10},
    {0x3117, 0x0A},
    {0x3118, 0x3F},
////{0x311A, 0x31}, // Disable internal LDO
    {0x311C, 0x10},
    {0x311D, 0x06},
    {0x311E, 0x0F},
    {0x311F, 0x0E},
    {0x3120, 0x0D},
    {0x3121, 0x0F},
    {0x3122, 0x00},
    {0x3123, 0x1D},
    {0x3126, 0x03},
    {0x3128, 0x57},
    {0x312A, 0x11},
    {0x312B, 0x41},
    {0x312E, 0x00},
    {0x312F, 0x00},
    {0x3130, 0x0C},
    {0x3141, 0x2A},
    {0x3142, 0x9F},
    {0x3147, 0x18},
    {0x3149, 0x18},
    {0x314B, 0x01},
    {0x3150, 0x50},
    {0x3152, 0x00},
    {0x3156, 0x2C},
    {0x315A, 0x0A},
    {0x315B, 0x2F},
    {0x315C, 0xE0},
    {0x315F, 0x02},
    {0x3160, 0x1F},
    {0x3163, 0x1F},
    {0x3164, 0x7F},
    {0x3165, 0x7F},
    {0x317B, 0x94},
    {0x317C, 0x00},
    {0x317D, 0x02},
    {0x318C, 0x00},
    {0x3500, 0x74},  // Start of Context A
    {0x3501, 0x0A},
    {0x3502, 0x77},
    {0x3503, 0x02},
    {0x3504, 0x14},
    {0x3505, 0x03},
    {0x3506, 0x00},
    {0x3507, 0x00},
    {0x3508, 0x00},
    {0x3509, 0x00},
    {0x350A, 0xFF},
    {0x350B, 0x01},
    {0x350C, 0x00},
    {0x350D, 0x01},
    {0x350F, 0x00},
    {0x3510, 0x01},
    {0x3512, 0x7F},
    {0x3513, 0x00},
    {0x3514, 0x00},
    {0x3515, 0x01},
    {0x3516, 0x00},
    {0x3517, 0x02},
    {0x3518, 0x00},
    {0x3519, 0x7F},
    {0x351A, 0x00},
    {0x351B, 0x5F},
    {0x351C, 0x00},
    {0x351D, 0x04},
    {0x351E, 0x24},
    {0x351F, 0x04},
    {0x3520, 0x03},
    {0x3521, 0x00},
    {0x3523, 0x60},
    {0x3524, 0x08},
    {0x3525, 0x19},
    {0x3526, 0x08},
    {0x3527, 0x10},
    {0x352A, 0x01},
    {0x352B, 0x04},
    {0x352C, 0x01},
    {0x352D, 0x38},
    {0x3535, 0x02},
    {0x3536, 0x03},
    {0x3537, 0x03},
    {0x3538, 0x08},
    {0x3539, 0x04},
    {0x353A, 0x0C},
    {0x353B, 0x03},
    {0x3540, 0x03},
    {0x3541, 0x03},
    {0x3542, 0x00},
    {0x3543, 0x09},
    {0x3549, 0x04},
    {0x354A, 0x35},
    {0x354B, 0x21},
    {0x354C, 0x01},
    {0x354D, 0xE0},
    {0x354E, 0xF0},
    {0x354F, 0x10},
    {0x3550, 0x10},
    {0x3551, 0x10},
    {0x3552, 0x20},
    {0x3553, 0x10},
    {0x3554, 0x01},
    {0x3555, 0x06},
    {0x3556, 0x0C},
    {0x3557, 0x12},
    {0x3558, 0x1C},
    {0x3559, 0x30},  // End of Context A
    {0x355A, 0x74},  // Start of Context B
    {0x355B, 0x0A},
    {0x355C, 0x77},
    {0x355D, 0x01},
    {0x355E, 0x1C},
    {0x355F, 0x03},
    {0x3560, 0x00},
    {0x3561, 0x01},
    {0x3562, 0x01},
    {0x3563, 0x03},
    {0x3564, 0x55},
    {0x3565, 0x01},
    {0x3566, 0x00},
    {0x3567, 0x01},
    {0x3569, 0x00},
    {0x356A, 0x01},
    {0x356C, 0x7F},
    {0x356D, 0x00},
    {0x356E, 0x00},
    {0x356F, 0x01},
    {0x3570, 0x00},
    {0x3571, 0x02},
    {0x3572, 0x00},
    {0x3573, 0x3F},
    {0x3574, 0x00},
    {0x3575, 0x2F},
    {0x3576, 0x00},
    {0x3577, 0x04},
    {0x3578, 0x24},
    {0x3579, 0x04},
    {0x357A, 0x03},
    {0x357B, 0x00},
    {0x357D, 0x60},
    {0x357E, 0x08},
    {0x357F, 0x19},
    {0x3580, 0x08},
    {0x3581, 0x10},
    {0x3584, 0x01},
    {0x3585, 0x04},
    {0x3586, 0x01},
    {0x3587, 0x38},
    {0x3588, 0x02},
    {0x3589, 0x12},
    {0x358A, 0x04},
    {0x358B, 0x24},
    {0x358C, 0x06},
    {0x358D, 0x36},
    {0x358F, 0x02},
    {0x3590, 0x03},
    {0x3591, 0x03},
    {0x3592, 0x08},
    {0x3593, 0x04},
    {0x3594, 0x0C},
    {0x3595, 0x03},
    {0x359A, 0x03},
    {0x359B, 0x03},
    {0x359C, 0x00},
    {0x359D, 0x09},
    {0x35A3, 0x02},
    {0x35A4, 0x03},
    {0x35A5, 0x21},
    {0x35A6, 0x01},
    {0x35A7, 0xE0},
    {0x35A8, 0xF0},
    {0x35A9, 0x10},
    {0x35AA, 0x10},
    {0x35AB, 0x10},
    {0x35AC, 0x20},
    {0x35AD, 0x10},
    {0x35AE, 0x01},
    {0x35AF, 0x06},
    {0x35B0, 0x0C},
    {0x35B1, 0x12},
    {0x35B2, 0x1C},
    {0x35B3, 0x30},
    {0x35B4, 0x74},
    {0x35B5, 0x0A},
    {0x35B6, 0x77},
    {0x35B7, 0x00},
    {0x35B8, 0x94},
    {0x35B9, 0x03},
    {0x35BA, 0x00},
    {0x35BB, 0x03},
    {0x35BC, 0x03},
    {0x35BD, 0x03},
    {0x35BE, 0x01},
    {0x35BF, 0x01},
    {0x35C0, 0x01},
    {0x35C1, 0x01},
    {0x35C3, 0x00},
    {0x35C4, 0x00},
    {0x35C6, 0x7F},
    {0x35C7, 0x00},
    {0x35C8, 0x00},
    {0x35C9, 0x01},
    {0x35CA, 0x00},
    {0x35CB, 0x02},
    {0x35CC, 0x00},
    {0x35CD, 0x0F},
    {0x35CE, 0x00},
    {0x35CF, 0x0B},
    {0x35D0, 0x00},
    {0x35D3, 0x04},
    {0x35D7, 0x18},
    {0x35D8, 0x01},
    {0x35D9, 0x20},
    {0x35DA, 0x08},
    {0x35DB, 0x14},
    {0x35DC, 0x70},
    {0x35DE, 0x00},
    {0x35DF, 0x01},
    {0x35E9, 0x02},
    {0x35EA, 0x03},
    {0x35EB, 0x03},
    {0x35EC, 0x08},
    {0x35ED, 0x04},
    {0x35EE, 0x0C},
    {0x35EF, 0x03},
    {0x35F4, 0x03},
    {0x35F5, 0x03},
    {0x35F6, 0x00},
    {0x35F7, 0x09},
    {0x35FD, 0x00},
    {0x35FE, 0x5E}, // End of Context B

    {0x0601, 0x01},  // test pattern: color bar
//{0x0601, 0x11},  // test pattern: walking ones

    {0x0104, 0x01},
    {0x0100, 0x01},
    {0xFFFF, 0xFF} // end of settings
};
#endif
// clang-format on

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_BAYER;

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;
#if 0
    // I2C slave address auto scan.
    // Looks for first 'ACK' on the bus,
    // so it doesn't work if there are
    // multiple slaves.
    g_slv_addr = sccb_scan();

    if (g_slv_addr == -1) {
        return -1;
    }

#else
    g_slv_addr = HM0360_I2C_SLAVE_ADDR;
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
    uint8_t rev;

    ret |= cambus_read(REVISION, &rev);
    *id = (int)rev;
    return ret;
}

static int get_manufacture_id(int *id)
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
    uint8_t buf[64] = { 0 };
    uint8_t *ptr = buf;

    for (i = 0;; i++) {
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }

        if (i == 0x35B3) {
            break;
        }

        ret = cambus_read(i, &byt);

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

    ret |= cambus_write(SW_RESET, 0);
    MXC_Delay(10000);
    ret |= cambus_write(SW_RESET, 0xFF);

#if 1

    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xFFFF); i++) {
        ret |= cambus_write(default_regs[i][0], (uint8_t)default_regs[i][1]);

        //printf("reg: 0x%04x , val: 0x%02x\r\n",default_regs[i][0], default_regs[i][1]);
        if (ret) {
            printf("fail");
        }
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
            reg = SW_STREAMING_MODE;
        }

        // Write back register
        ret |= cambus_write(MODE_SELECT, reg);
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
