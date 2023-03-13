/**
 * @file    afe_adc_zero_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AFE_ADC_ZERO Peripheral Module.
 * @note    This file is @generated.
 */

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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AFE_ADC_ZERO_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AFE_ADC_ZERO_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     afe_adc_zero
 * @defgroup    afe_adc_zero_registers AFE_ADC_ZERO_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AFE_ADC_ZERO Peripheral Module.
 * @details     Analog Front End ADC0 on Stacked Die via SPI
 */

/**
 * @ingroup afe_adc_zero_registers
 * Structure type to access the AFE_ADC_ZERO Registers.
 */
typedef struct {
    __R  uint8_t  rsv_0x0;
    __IO uint8_t  pd;                   /**< <tt>\b 0x00000001:</tt> AFE_ADC_ZERO PD Register */
    __R  uint8_t  rsv_0x2_0x10000[65535];
    __IO uint8_t  conv_start;           /**< <tt>\b 0x00010001:</tt> AFE_ADC_ZERO CONV_START Register */
    __R  uint8_t  rsv_0x10002_0x20000[65535];
    __IO uint8_t  seq_start;            /**< <tt>\b 0x00020001:</tt> AFE_ADC_ZERO SEQ_START Register */
    __R  uint8_t  rsv_0x20002_0x30000[65535];
    __IO uint8_t  cal_start;            /**< <tt>\b 0x00030001:</tt> AFE_ADC_ZERO CAL_START Register */
    __R  uint8_t  rsv_0x30002_0x40000[65535];
    __IO uint8_t  gp0_ctrl;             /**< <tt>\b 0x00040001:</tt> AFE_ADC_ZERO GP0_CTRL Register */
    __R  uint8_t  rsv_0x40002_0x50000[65535];
    __IO uint8_t  gp1_ctrl;             /**< <tt>\b 0x00050001:</tt> AFE_ADC_ZERO GP1_CTRL Register */
    __R  uint8_t  rsv_0x50002_0x60000[65535];
    __IO uint8_t  gp_conv;              /**< <tt>\b 0x00060001:</tt> AFE_ADC_ZERO GP_CONV Register */
    __R  uint8_t  rsv_0x60002_0x70000[65535];
    __IO uint8_t  gp_seq_addr;          /**< <tt>\b 0x00070001:</tt> AFE_ADC_ZERO GP_SEQ_ADDR Register */
    __R  uint8_t  rsv_0x70002_0x80000[65535];
    __IO uint8_t  filter;               /**< <tt>\b 0x00080001:</tt> AFE_ADC_ZERO FILTER Register */
    __R  uint8_t  rsv_0x80002_0x90000[65535];
    __IO uint8_t  ctrl;                 /**< <tt>\b 0x00090001:</tt> AFE_ADC_ZERO CTRL Register */
    __R  uint8_t  rsv_0x90002_0xa0000[65535];
    __IO uint8_t  source;               /**< <tt>\b 0x000A0001:</tt> AFE_ADC_ZERO SOURCE Register */
    __R  uint8_t  rsv_0xa0002_0xb0000[65535];
    __IO uint8_t  mux_ctrl0;            /**< <tt>\b 0x000B0001:</tt> AFE_ADC_ZERO MUX_CTRL0 Register */
    __R  uint8_t  rsv_0xb0002_0xc0000[65535];
    __IO uint8_t  mux_ctrl1;            /**< <tt>\b 0x000C0001:</tt> AFE_ADC_ZERO MUX_CTRL1 Register */
    __R  uint8_t  rsv_0xc0002_0xd0000[65535];
    __IO uint8_t  mux_ctrl2;            /**< <tt>\b 0x000D0001:</tt> AFE_ADC_ZERO MUX_CTRL2 Register */
    __R  uint8_t  rsv_0xd0002_0xe0000[65535];
    __IO uint8_t  pga;                  /**< <tt>\b 0x000E0001:</tt> AFE_ADC_ZERO PGA Register */
    __R  uint8_t  rsv_0xe0002_0xf0000[65535];
    __IO uint8_t  wait_ext;             /**< <tt>\b 0x000F0001:</tt> AFE_ADC_ZERO WAIT_EXT Register */
    __R  uint8_t  rsv_0xf0002_0x100000[65535];
    __IO uint8_t  wait_start;           /**< <tt>\b 0x00100001:</tt> AFE_ADC_ZERO WAIT_START Register */
    __R  uint8_t  rsv_0x100002_0x110002[65537];
    __IO uint32_t part_id;              /**< <tt>\b 0x00110003:</tt> AFE_ADC_ZERO PART_ID Register */
    __R  uint32_t rsv_0x110007_0x120002[16383];
    __IO uint32_t sysc_sel;             /**< <tt>\b 0x00120003:</tt> AFE_ADC_ZERO SYSC_SEL Register */
    __R  uint32_t rsv_0x120007_0x130002[16383];
    __IO uint32_t sys_off_a;            /**< <tt>\b 0x00130003:</tt> AFE_ADC_ZERO SYS_OFF_A Register */
    __R  uint32_t rsv_0x130007_0x140002[16383];
    __IO uint32_t sys_off_b;            /**< <tt>\b 0x00140003:</tt> AFE_ADC_ZERO SYS_OFF_B Register */
    __R  uint32_t rsv_0x140007_0x150002[16383];
    __IO uint32_t sys_gain_a;           /**< <tt>\b 0x00150003:</tt> AFE_ADC_ZERO SYS_GAIN_A Register */
    __R  uint32_t rsv_0x150007_0x160002[16383];
    __IO uint32_t sys_gain_b;           /**< <tt>\b 0x00160003:</tt> AFE_ADC_ZERO SYS_GAIN_B Register */
    __R  uint32_t rsv_0x160007_0x170002[16383];
    __IO uint32_t self_off;             /**< <tt>\b 0x00170003:</tt> AFE_ADC_ZERO SELF_OFF Register */
    __R  uint32_t rsv_0x170007_0x180002[16383];
    __IO uint32_t self_gain_1;          /**< <tt>\b 0x00180003:</tt> AFE_ADC_ZERO SELF_GAIN_1 Register */
    __R  uint32_t rsv_0x180007_0x190002[16383];
    __IO uint32_t self_gain_2;          /**< <tt>\b 0x00190003:</tt> AFE_ADC_ZERO SELF_GAIN_2 Register */
    __R  uint32_t rsv_0x190007_0x1a0002[16383];
    __IO uint32_t self_gain_4;          /**< <tt>\b 0x001A0003:</tt> AFE_ADC_ZERO SELF_GAIN_4 Register */
    __R  uint32_t rsv_0x1a0007_0x1b0002[16383];
    __IO uint32_t self_gain_8;          /**< <tt>\b 0x001B0003:</tt> AFE_ADC_ZERO SELF_GAIN_8 Register */
    __R  uint32_t rsv_0x1b0007_0x1c0002[16383];
    __IO uint32_t self_gain_16;         /**< <tt>\b 0x001C0003:</tt> AFE_ADC_ZERO SELF_GAIN_16 Register */
    __R  uint32_t rsv_0x1c0007_0x1d0002[16383];
    __IO uint32_t self_gain_32;         /**< <tt>\b 0x001D0003:</tt> AFE_ADC_ZERO SELF_GAIN_32 Register */
    __R  uint32_t rsv_0x1d0007_0x1e0002[16383];
    __IO uint32_t self_gain_64;         /**< <tt>\b 0x001E0003:</tt> AFE_ADC_ZERO SELF_GAIN_64 Register */
    __R  uint32_t rsv_0x1e0007_0x1f0002[16383];
    __IO uint32_t self_gain_128;        /**< <tt>\b 0x001F0003:</tt> AFE_ADC_ZERO SELF_GAIN_128 Register */
    __R  uint32_t rsv_0x1f0007_0x200002[16383];
    __IO uint32_t lthresh0;             /**< <tt>\b 0x00200003:</tt> AFE_ADC_ZERO LTHRESH0 Register */
    __R  uint32_t rsv_0x200007_0x210002[16383];
    __IO uint32_t lthresh1;             /**< <tt>\b 0x00210003:</tt> AFE_ADC_ZERO LTHRESH1 Register */
    __R  uint32_t rsv_0x210007_0x220002[16383];
    __IO uint32_t lthresh2;             /**< <tt>\b 0x00220003:</tt> AFE_ADC_ZERO LTHRESH2 Register */
    __R  uint32_t rsv_0x220007_0x230002[16383];
    __IO uint32_t lthresh3;             /**< <tt>\b 0x00230003:</tt> AFE_ADC_ZERO LTHRESH3 Register */
    __R  uint32_t rsv_0x230007_0x240002[16383];
    __IO uint32_t lthresh4;             /**< <tt>\b 0x00240003:</tt> AFE_ADC_ZERO LTHRESH4 Register */
    __R  uint32_t rsv_0x240007_0x250002[16383];
    __IO uint32_t lthresh5;             /**< <tt>\b 0x00250003:</tt> AFE_ADC_ZERO LTHRESH5 Register */
    __R  uint32_t rsv_0x250007_0x260002[16383];
    __IO uint32_t lthresh6;             /**< <tt>\b 0x00260003:</tt> AFE_ADC_ZERO LTHRESH6 Register */
    __R  uint32_t rsv_0x260007_0x270002[16383];
    __IO uint32_t lthresh7;             /**< <tt>\b 0x00270003:</tt> AFE_ADC_ZERO LTHRESH7 Register */
    __R  uint32_t rsv_0x270007_0x280002[16383];
    __IO uint32_t uthresh0;             /**< <tt>\b 0x00280003:</tt> AFE_ADC_ZERO UTHRESH0 Register */
    __R  uint32_t rsv_0x280007_0x290002[16383];
    __IO uint32_t uthresh1;             /**< <tt>\b 0x00290003:</tt> AFE_ADC_ZERO UTHRESH1 Register */
    __R  uint32_t rsv_0x290007_0x2a0002[16383];
    __IO uint32_t uthresh2;             /**< <tt>\b 0x002A0003:</tt> AFE_ADC_ZERO UTHRESH2 Register */
    __R  uint32_t rsv_0x2a0007_0x2b0002[16383];
    __IO uint32_t uthresh3;             /**< <tt>\b 0x002B0003:</tt> AFE_ADC_ZERO UTHRESH3 Register */
    __R  uint32_t rsv_0x2b0007_0x2c0002[16383];
    __IO uint32_t uthresh4;             /**< <tt>\b 0x002C0003:</tt> AFE_ADC_ZERO UTHRESH4 Register */
    __R  uint32_t rsv_0x2c0007_0x2d0002[16383];
    __IO uint32_t uthresh5;             /**< <tt>\b 0x002D0003:</tt> AFE_ADC_ZERO UTHRESH5 Register */
    __R  uint32_t rsv_0x2d0007_0x2e0002[16383];
    __IO uint32_t uthresh6;             /**< <tt>\b 0x002E0003:</tt> AFE_ADC_ZERO UTHRESH6 Register */
    __R  uint32_t rsv_0x2e0007_0x2f0002[16383];
    __IO uint32_t uthresh7;             /**< <tt>\b 0x002F0003:</tt> AFE_ADC_ZERO UTHRESH7 Register */
    __R  uint32_t rsv_0x2f0007_0x300002[16383];
    __IO uint32_t data0;                /**< <tt>\b 0x00300003:</tt> AFE_ADC_ZERO DATA0 Register */
    __R  uint32_t rsv_0x300007_0x310002[16383];
    __IO uint32_t data1;                /**< <tt>\b 0x00310003:</tt> AFE_ADC_ZERO DATA1 Register */
    __R  uint32_t rsv_0x310007_0x320002[16383];
    __IO uint32_t data2;                /**< <tt>\b 0x00320003:</tt> AFE_ADC_ZERO DATA2 Register */
    __R  uint32_t rsv_0x320007_0x330002[16383];
    __IO uint32_t data3;                /**< <tt>\b 0x00330003:</tt> AFE_ADC_ZERO DATA3 Register */
    __R  uint32_t rsv_0x330007_0x340002[16383];
    __IO uint32_t data4;                /**< <tt>\b 0x00340003:</tt> AFE_ADC_ZERO DATA4 Register */
    __R  uint32_t rsv_0x340007_0x350002[16383];
    __IO uint32_t data5;                /**< <tt>\b 0x00350003:</tt> AFE_ADC_ZERO DATA5 Register */
    __R  uint32_t rsv_0x350007_0x360002[16383];
    __IO uint32_t data6;                /**< <tt>\b 0x00360003:</tt> AFE_ADC_ZERO DATA6 Register */
    __R  uint32_t rsv_0x360007_0x370002[16383];
    __IO uint32_t data7;                /**< <tt>\b 0x00370003:</tt> AFE_ADC_ZERO DATA7 Register */
    __R  uint32_t rsv_0x370007_0x380002[16383];
    __IO uint32_t status;               /**< <tt>\b 0x00380003:</tt> AFE_ADC_ZERO STATUS Register */
    __R  uint32_t rsv_0x380007_0x390002[16383];
    __IO uint32_t status_ie;            /**< <tt>\b 0x00390003:</tt> AFE_ADC_ZERO STATUS_IE Register */
    __R  uint8_t  rsv_0x390007_0x3a0001[65531];
    __IO uint16_t uc_0;                 /**< <tt>\b 0x003A0002:</tt> AFE_ADC_ZERO UC_0 Register */
    __R  uint16_t rsv_0x3a0004_0x3b0001[32767];
    __IO uint16_t uc_1;                 /**< <tt>\b 0x003B0002:</tt> AFE_ADC_ZERO UC_1 Register */
    __R  uint16_t rsv_0x3b0004_0x3c0001[32767];
    __IO uint16_t uc_2;                 /**< <tt>\b 0x003C0002:</tt> AFE_ADC_ZERO UC_2 Register */
    __R  uint16_t rsv_0x3c0004_0x3d0001[32767];
    __IO uint16_t uc_3;                 /**< <tt>\b 0x003D0002:</tt> AFE_ADC_ZERO UC_3 Register */
    __R  uint16_t rsv_0x3d0004_0x3e0001[32767];
    __IO uint16_t uc_4;                 /**< <tt>\b 0x003E0002:</tt> AFE_ADC_ZERO UC_4 Register */
    __R  uint16_t rsv_0x3e0004_0x3f0001[32767];
    __IO uint16_t uc_5;                 /**< <tt>\b 0x003F0002:</tt> AFE_ADC_ZERO UC_5 Register */
    __R  uint16_t rsv_0x3f0004_0x400001[32767];
    __IO uint16_t uc_6;                 /**< <tt>\b 0x00400002:</tt> AFE_ADC_ZERO UC_6 Register */
    __R  uint16_t rsv_0x400004_0x410001[32767];
    __IO uint16_t uc_7;                 /**< <tt>\b 0x00410002:</tt> AFE_ADC_ZERO UC_7 Register */
    __R  uint16_t rsv_0x410004_0x420001[32767];
    __IO uint16_t uc_8;                 /**< <tt>\b 0x00420002:</tt> AFE_ADC_ZERO UC_8 Register */
    __R  uint16_t rsv_0x420004_0x430001[32767];
    __IO uint16_t uc_9;                 /**< <tt>\b 0x00430002:</tt> AFE_ADC_ZERO UC_9 Register */
    __R  uint16_t rsv_0x430004_0x440001[32767];
    __IO uint16_t uc_10;                /**< <tt>\b 0x00440002:</tt> AFE_ADC_ZERO UC_10 Register */
    __R  uint16_t rsv_0x440004_0x450001[32767];
    __IO uint16_t uc_11;                /**< <tt>\b 0x00450002:</tt> AFE_ADC_ZERO UC_11 Register */
    __R  uint16_t rsv_0x450004_0x460001[32767];
    __IO uint16_t uc_12;                /**< <tt>\b 0x00460002:</tt> AFE_ADC_ZERO UC_12 Register */
    __R  uint16_t rsv_0x460004_0x470001[32767];
    __IO uint16_t uc_13;                /**< <tt>\b 0x00470002:</tt> AFE_ADC_ZERO UC_13 Register */
    __R  uint16_t rsv_0x470004_0x480001[32767];
    __IO uint16_t uc_14;                /**< <tt>\b 0x00480002:</tt> AFE_ADC_ZERO UC_14 Register */
    __R  uint16_t rsv_0x480004_0x490001[32767];
    __IO uint16_t uc_15;                /**< <tt>\b 0x00490002:</tt> AFE_ADC_ZERO UC_15 Register */
    __R  uint16_t rsv_0x490004_0x4a0001[32767];
    __IO uint16_t uc_16;                /**< <tt>\b 0x004A0002:</tt> AFE_ADC_ZERO UC_16 Register */
    __R  uint16_t rsv_0x4a0004_0x4b0001[32767];
    __IO uint16_t uc_17;                /**< <tt>\b 0x004B0002:</tt> AFE_ADC_ZERO UC_17 Register */
    __R  uint16_t rsv_0x4b0004_0x4c0001[32767];
    __IO uint16_t uc_18;                /**< <tt>\b 0x004C0002:</tt> AFE_ADC_ZERO UC_18 Register */
    __R  uint16_t rsv_0x4c0004_0x4d0001[32767];
    __IO uint16_t uc_19;                /**< <tt>\b 0x004D0002:</tt> AFE_ADC_ZERO UC_19 Register */
    __R  uint16_t rsv_0x4d0004_0x4e0001[32767];
    __IO uint16_t uc_20;                /**< <tt>\b 0x004E0002:</tt> AFE_ADC_ZERO UC_20 Register */
    __R  uint16_t rsv_0x4e0004_0x4f0001[32767];
    __IO uint16_t uc_21;                /**< <tt>\b 0x004F0002:</tt> AFE_ADC_ZERO UC_21 Register */
    __R  uint16_t rsv_0x4f0004_0x500001[32767];
    __IO uint16_t uc_22;                /**< <tt>\b 0x00500002:</tt> AFE_ADC_ZERO UC_22 Register */
    __R  uint16_t rsv_0x500004_0x510001[32767];
    __IO uint16_t uc_23;                /**< <tt>\b 0x00510002:</tt> AFE_ADC_ZERO UC_23 Register */
    __R  uint16_t rsv_0x510004_0x520001[32767];
    __IO uint16_t uc_24;                /**< <tt>\b 0x00520002:</tt> AFE_ADC_ZERO UC_24 Register */
    __R  uint16_t rsv_0x520004_0x530001[32767];
    __IO uint16_t uc_25;                /**< <tt>\b 0x00530002:</tt> AFE_ADC_ZERO UC_25 Register */
    __R  uint16_t rsv_0x530004_0x540001[32767];
    __IO uint16_t uc_26;                /**< <tt>\b 0x00540002:</tt> AFE_ADC_ZERO UC_26 Register */
    __R  uint16_t rsv_0x540004_0x550001[32767];
    __IO uint16_t uc_27;                /**< <tt>\b 0x00550002:</tt> AFE_ADC_ZERO UC_27 Register */
    __R  uint16_t rsv_0x550004_0x560001[32767];
    __IO uint16_t uc_28;                /**< <tt>\b 0x00560002:</tt> AFE_ADC_ZERO UC_28 Register */
    __R  uint16_t rsv_0x560004_0x570001[32767];
    __IO uint16_t uc_29;                /**< <tt>\b 0x00570002:</tt> AFE_ADC_ZERO UC_29 Register */
    __R  uint16_t rsv_0x570004_0x580001[32767];
    __IO uint16_t uc_30;                /**< <tt>\b 0x00580002:</tt> AFE_ADC_ZERO UC_30 Register */
    __R  uint16_t rsv_0x580004_0x590001[32767];
    __IO uint16_t uc_31;                /**< <tt>\b 0x00590002:</tt> AFE_ADC_ZERO UC_31 Register */
    __R  uint16_t rsv_0x590004_0x5a0001[32767];
    __IO uint16_t uc_32;                /**< <tt>\b 0x005A0002:</tt> AFE_ADC_ZERO UC_32 Register */
    __R  uint16_t rsv_0x5a0004_0x5b0001[32767];
    __IO uint16_t uc_33;                /**< <tt>\b 0x005B0002:</tt> AFE_ADC_ZERO UC_33 Register */
    __R  uint16_t rsv_0x5b0004_0x5c0001[32767];
    __IO uint16_t uc_34;                /**< <tt>\b 0x005C0002:</tt> AFE_ADC_ZERO UC_34 Register */
    __R  uint16_t rsv_0x5c0004_0x5d0001[32767];
    __IO uint16_t uc_35;                /**< <tt>\b 0x005D0002:</tt> AFE_ADC_ZERO UC_35 Register */
    __R  uint16_t rsv_0x5d0004_0x5e0001[32767];
    __IO uint16_t uc_36;                /**< <tt>\b 0x005E0002:</tt> AFE_ADC_ZERO UC_36 Register */
    __R  uint16_t rsv_0x5e0004_0x5f0001[32767];
    __IO uint16_t uc_37;                /**< <tt>\b 0x005F0002:</tt> AFE_ADC_ZERO UC_37 Register */
    __R  uint16_t rsv_0x5f0004_0x600001[32767];
    __IO uint16_t uc_38;                /**< <tt>\b 0x00600002:</tt> AFE_ADC_ZERO UC_38 Register */
    __R  uint16_t rsv_0x600004_0x610001[32767];
    __IO uint16_t uc_39;                /**< <tt>\b 0x00610002:</tt> AFE_ADC_ZERO UC_39 Register */
    __R  uint16_t rsv_0x610004_0x620001[32767];
    __IO uint16_t uc_40;                /**< <tt>\b 0x00620002:</tt> AFE_ADC_ZERO UC_40 Register */
    __R  uint16_t rsv_0x620004_0x630001[32767];
    __IO uint16_t uc_41;                /**< <tt>\b 0x00630002:</tt> AFE_ADC_ZERO UC_41 Register */
    __R  uint16_t rsv_0x630004_0x640001[32767];
    __IO uint16_t uc_42;                /**< <tt>\b 0x00640002:</tt> AFE_ADC_ZERO UC_42 Register */
    __R  uint16_t rsv_0x640004_0x650001[32767];
    __IO uint16_t uc_43;                /**< <tt>\b 0x00650002:</tt> AFE_ADC_ZERO UC_43 Register */
    __R  uint16_t rsv_0x650004_0x660001[32767];
    __IO uint16_t uc_44;                /**< <tt>\b 0x00660002:</tt> AFE_ADC_ZERO UC_44 Register */
    __R  uint16_t rsv_0x660004_0x670001[32767];
    __IO uint16_t uc_45;                /**< <tt>\b 0x00670002:</tt> AFE_ADC_ZERO UC_45 Register */
    __R  uint16_t rsv_0x670004_0x680001[32767];
    __IO uint16_t uc_46;                /**< <tt>\b 0x00680002:</tt> AFE_ADC_ZERO UC_46 Register */
    __R  uint16_t rsv_0x680004_0x690001[32767];
    __IO uint16_t uc_47;                /**< <tt>\b 0x00690002:</tt> AFE_ADC_ZERO UC_47 Register */
    __R  uint16_t rsv_0x690004_0x6a0001[32767];
    __IO uint16_t uc_48;                /**< <tt>\b 0x006A0002:</tt> AFE_ADC_ZERO UC_48 Register */
    __R  uint16_t rsv_0x6a0004_0x6b0001[32767];
    __IO uint16_t uc_49;                /**< <tt>\b 0x006B0002:</tt> AFE_ADC_ZERO UC_49 Register */
    __R  uint16_t rsv_0x6b0004_0x6c0001[32767];
    __IO uint16_t uc_50;                /**< <tt>\b 0x006C0002:</tt> AFE_ADC_ZERO UC_50 Register */
    __R  uint16_t rsv_0x6c0004_0x6d0001[32767];
    __IO uint16_t uc_51;                /**< <tt>\b 0x006D0002:</tt> AFE_ADC_ZERO UC_51 Register */
    __R  uint16_t rsv_0x6d0004_0x6e0001[32767];
    __IO uint16_t uc_52;                /**< <tt>\b 0x006E0002:</tt> AFE_ADC_ZERO UC_52 Register */
    __R  uint8_t  rsv_0x6e0004_0x6f0000[65533];
    __IO uint8_t  ucaddr;               /**< <tt>\b 0x006F0001:</tt> AFE_ADC_ZERO UCADDR Register */
    __R  uint8_t  rsv_0x6f0002_0x700000[65535];
    __IO uint8_t  ft_pword;             /**< <tt>\b 0x00700001:</tt> AFE_ADC_ZERO FT_PWORD Register */
    __R  uint8_t  rsv_0x700002_0x770002[458753];
    __IO uint32_t adc_trim0;            /**< <tt>\b 0x00770003:</tt> AFE_ADC_ZERO ADC_TRIM0 Register */
    __R  uint8_t  rsv_0x770007_0x780001[65531];
    __IO uint16_t adc_trim1;            /**< <tt>\b 0x00780002:</tt> AFE_ADC_ZERO ADC_TRIM1 Register */
    __R  uint16_t rsv_0x780004_0x790001[32767];
    __IO uint16_t ana_trim;             /**< <tt>\b 0x00790002:</tt> AFE_ADC_ZERO ANA_TRIM Register */
    __R  uint8_t  rsv_0x790004_0x7a0000[65533];
    __IO uint8_t  sys_ctrl;             /**< <tt>\b 0x007A0001:</tt> AFE_ADC_ZERO SYS_CTRL Register */
    __R  uint8_t  rsv_0x7a0002_0x7c0000[131071];
    __IO uint8_t  ts_ctrl;              /**< <tt>\b 0x007C0001:</tt> AFE_ADC_ZERO TS_CTRL Register */
} mxc_afe_adc_zero_regs_t;

/* Register offsets for module AFE_ADC_ZERO */
/**
 * @ingroup    afe_adc_zero_registers
 * @defgroup   AFE_ADC_ZERO_Register_Offsets Register Offsets
 * @brief      AFE_ADC_ZERO Peripheral Register Offsets from the AFE_ADC_ZERO Base Peripheral Address.
 * @{
 */
#define MXC_R_AFE_ADC_ZERO_PD              ((uint32_t)0x00000001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x0001</tt> */
#define MXC_R_AFE_ADC_ZERO_CONV_START      ((uint32_t)0x00010001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x10001</tt> */
#define MXC_R_AFE_ADC_ZERO_SEQ_START       ((uint32_t)0x00020001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x20001</tt> */
#define MXC_R_AFE_ADC_ZERO_CAL_START       ((uint32_t)0x00030001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x30001</tt> */
#define MXC_R_AFE_ADC_ZERO_GP0_CTRL        ((uint32_t)0x00040001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x40001</tt> */
#define MXC_R_AFE_ADC_ZERO_GP1_CTRL        ((uint32_t)0x00050001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x50001</tt> */
#define MXC_R_AFE_ADC_ZERO_GP_CONV         ((uint32_t)0x00060001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x60001</tt> */
#define MXC_R_AFE_ADC_ZERO_GP_SEQ_ADDR     ((uint32_t)0x00070001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x70001</tt> */
#define MXC_R_AFE_ADC_ZERO_FILTER          ((uint32_t)0x00080001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x80001</tt> */
#define MXC_R_AFE_ADC_ZERO_CTRL            ((uint32_t)0x00090001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x90001</tt> */
#define MXC_R_AFE_ADC_ZERO_SOURCE          ((uint32_t)0x000A0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0xA0001</tt> */
#define MXC_R_AFE_ADC_ZERO_MUX_CTRL0       ((uint32_t)0x000B0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0xB0001</tt> */
#define MXC_R_AFE_ADC_ZERO_MUX_CTRL1       ((uint32_t)0x000C0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0xC0001</tt> */
#define MXC_R_AFE_ADC_ZERO_MUX_CTRL2       ((uint32_t)0x000D0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0xD0001</tt> */
#define MXC_R_AFE_ADC_ZERO_PGA             ((uint32_t)0x000E0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0xE0001</tt> */
#define MXC_R_AFE_ADC_ZERO_WAIT_EXT        ((uint32_t)0x000F0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0xF0001</tt> */
#define MXC_R_AFE_ADC_ZERO_WAIT_START      ((uint32_t)0x00100001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x100001</tt> */
#define MXC_R_AFE_ADC_ZERO_PART_ID         ((uint32_t)0x00110003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x110003</tt> */
#define MXC_R_AFE_ADC_ZERO_SYSC_SEL        ((uint32_t)0x00120003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x120003</tt> */
#define MXC_R_AFE_ADC_ZERO_SYS_OFF_A       ((uint32_t)0x00130003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x130003</tt> */
#define MXC_R_AFE_ADC_ZERO_SYS_OFF_B       ((uint32_t)0x00140003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x140003</tt> */
#define MXC_R_AFE_ADC_ZERO_SYS_GAIN_A      ((uint32_t)0x00150003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x150003</tt> */
#define MXC_R_AFE_ADC_ZERO_SYS_GAIN_B      ((uint32_t)0x00160003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x160003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_OFF        ((uint32_t)0x00170003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x170003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_1     ((uint32_t)0x00180003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x180003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_2     ((uint32_t)0x00190003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x190003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_4     ((uint32_t)0x001A0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x1A0003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_8     ((uint32_t)0x001B0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x1B0003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_16    ((uint32_t)0x001C0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x1C0003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_32    ((uint32_t)0x001D0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x1D0003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_64    ((uint32_t)0x001E0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x1E0003</tt> */
#define MXC_R_AFE_ADC_ZERO_SELF_GAIN_128   ((uint32_t)0x001F0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x1F0003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH0        ((uint32_t)0x00200003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x200003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH1        ((uint32_t)0x00210003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x210003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH2        ((uint32_t)0x00220003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x220003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH3        ((uint32_t)0x00230003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x230003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH4        ((uint32_t)0x00240003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x240003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH5        ((uint32_t)0x00250003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x250003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH6        ((uint32_t)0x00260003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x260003</tt> */
#define MXC_R_AFE_ADC_ZERO_LTHRESH7        ((uint32_t)0x00270003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x270003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH0        ((uint32_t)0x00280003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x280003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH1        ((uint32_t)0x00290003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x290003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH2        ((uint32_t)0x002A0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x2A0003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH3        ((uint32_t)0x002B0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x2B0003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH4        ((uint32_t)0x002C0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x2C0003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH5        ((uint32_t)0x002D0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x2D0003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH6        ((uint32_t)0x002E0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x2E0003</tt> */
#define MXC_R_AFE_ADC_ZERO_UTHRESH7        ((uint32_t)0x002F0003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x2F0003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA0           ((uint32_t)0x00300003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x300003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA1           ((uint32_t)0x00310003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x310003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA2           ((uint32_t)0x00320003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x320003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA3           ((uint32_t)0x00330003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x330003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA4           ((uint32_t)0x00340003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x340003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA5           ((uint32_t)0x00350003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x350003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA6           ((uint32_t)0x00360003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x360003</tt> */
#define MXC_R_AFE_ADC_ZERO_DATA7           ((uint32_t)0x00370003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x370003</tt> */
#define MXC_R_AFE_ADC_ZERO_STATUS          ((uint32_t)0x00380003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x380003</tt> */
#define MXC_R_AFE_ADC_ZERO_STATUS_IE       ((uint32_t)0x00390003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x390003</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_0            ((uint32_t)0x003A0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x3A0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_1            ((uint32_t)0x003B0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x3B0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_2            ((uint32_t)0x003C0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x3C0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_3            ((uint32_t)0x003D0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x3D0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_4            ((uint32_t)0x003E0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x3E0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_5            ((uint32_t)0x003F0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x3F0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_6            ((uint32_t)0x00400002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x400002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_7            ((uint32_t)0x00410002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x410002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_8            ((uint32_t)0x00420002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x420002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_9            ((uint32_t)0x00430002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x430002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_10           ((uint32_t)0x00440002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x440002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_11           ((uint32_t)0x00450002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x450002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_12           ((uint32_t)0x00460002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x460002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_13           ((uint32_t)0x00470002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x470002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_14           ((uint32_t)0x00480002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x480002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_15           ((uint32_t)0x00490002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x490002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_16           ((uint32_t)0x004A0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x4A0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_17           ((uint32_t)0x004B0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x4B0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_18           ((uint32_t)0x004C0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x4C0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_19           ((uint32_t)0x004D0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x4D0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_20           ((uint32_t)0x004E0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x4E0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_21           ((uint32_t)0x004F0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x4F0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_22           ((uint32_t)0x00500002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x500002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_23           ((uint32_t)0x00510002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x510002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_24           ((uint32_t)0x00520002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x520002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_25           ((uint32_t)0x00530002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x530002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_26           ((uint32_t)0x00540002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x540002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_27           ((uint32_t)0x00550002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x550002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_28           ((uint32_t)0x00560002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x560002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_29           ((uint32_t)0x00570002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x570002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_30           ((uint32_t)0x00580002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x580002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_31           ((uint32_t)0x00590002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x590002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_32           ((uint32_t)0x005A0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x5A0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_33           ((uint32_t)0x005B0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x5B0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_34           ((uint32_t)0x005C0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x5C0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_35           ((uint32_t)0x005D0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x5D0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_36           ((uint32_t)0x005E0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x5E0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_37           ((uint32_t)0x005F0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x5F0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_38           ((uint32_t)0x00600002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x600002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_39           ((uint32_t)0x00610002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x610002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_40           ((uint32_t)0x00620002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x620002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_41           ((uint32_t)0x00630002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x630002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_42           ((uint32_t)0x00640002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x640002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_43           ((uint32_t)0x00650002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x650002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_44           ((uint32_t)0x00660002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x660002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_45           ((uint32_t)0x00670002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x670002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_46           ((uint32_t)0x00680002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x680002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_47           ((uint32_t)0x00690002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x690002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_48           ((uint32_t)0x006A0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x6A0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_49           ((uint32_t)0x006B0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x6B0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_50           ((uint32_t)0x006C0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x6C0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_51           ((uint32_t)0x006D0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x6D0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UC_52           ((uint32_t)0x006E0002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x6E0002</tt> */
#define MXC_R_AFE_ADC_ZERO_UCADDR          ((uint32_t)0x006F0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x6F0001</tt> */
#define MXC_R_AFE_ADC_ZERO_FT_PWORD        ((uint32_t)0x00700001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x700001</tt> */
#define MXC_R_AFE_ADC_ZERO_ADC_TRIM0       ((uint32_t)0x00770003UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x770003</tt> */
#define MXC_R_AFE_ADC_ZERO_ADC_TRIM1       ((uint32_t)0x00780002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x780002</tt> */
#define MXC_R_AFE_ADC_ZERO_ANA_TRIM        ((uint32_t)0x00790002UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x790002</tt> */
#define MXC_R_AFE_ADC_ZERO_SYS_CTRL        ((uint32_t)0x007A0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x7A0001</tt> */
#define MXC_R_AFE_ADC_ZERO_TS_CTRL         ((uint32_t)0x007C0001UL) /**< Offset from AFE_ADC_ZERO Base Address: <tt> 0x7C0001</tt> */
/**@} end of group afe_adc_zero_registers */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_PD AFE_ADC_ZERO_PD
 * @brief    Power down
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_PD_PD_POS                   0 /**< PD_PD Position */
#define MXC_F_AFE_ADC_ZERO_PD_PD                       ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_PD_PD_POS)) /**< PD_PD Mask */
#define MXC_V_AFE_ADC_ZERO_PD_PD_NORMAL_MODE           ((uint8_t)0x0UL) /**< PD_PD_NORMAL_MODE Value */
#define MXC_S_AFE_ADC_ZERO_PD_PD_NORMAL_MODE           (MXC_V_AFE_ADC_ZERO_PD_PD_NORMAL_MODE << MXC_F_AFE_ADC_ZERO_PD_PD_POS) /**< PD_PD_NORMAL_MODE Setting */
#define MXC_V_AFE_ADC_ZERO_PD_PD_STANDBY_MODE          ((uint8_t)0x1UL) /**< PD_PD_STANDBY_MODE Value */
#define MXC_S_AFE_ADC_ZERO_PD_PD_STANDBY_MODE          (MXC_V_AFE_ADC_ZERO_PD_PD_STANDBY_MODE << MXC_F_AFE_ADC_ZERO_PD_PD_POS) /**< PD_PD_STANDBY_MODE Setting */
#define MXC_V_AFE_ADC_ZERO_PD_PD_SLEEP_MODE            ((uint8_t)0x2UL) /**< PD_PD_SLEEP_MODE Value */
#define MXC_S_AFE_ADC_ZERO_PD_PD_SLEEP_MODE            (MXC_V_AFE_ADC_ZERO_PD_PD_SLEEP_MODE << MXC_F_AFE_ADC_ZERO_PD_PD_POS) /**< PD_PD_SLEEP_MODE Setting */
#define MXC_V_AFE_ADC_ZERO_PD_PD_RESET                 ((uint8_t)0x3UL) /**< PD_PD_RESET Value */
#define MXC_S_AFE_ADC_ZERO_PD_PD_RESET                 (MXC_V_AFE_ADC_ZERO_PD_PD_RESET << MXC_F_AFE_ADC_ZERO_PD_PD_POS) /**< PD_PD_RESET Setting */

/**@} end of group AFE_ADC_ZERO_PD_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_CONV_START AFE_ADC_ZERO_CONV_START
 * @brief    Initiate conversions
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_CONV_START_CONV_TYPE_POS    0 /**< CONV_START_CONV_TYPE Position */
#define MXC_F_AFE_ADC_ZERO_CONV_START_CONV_TYPE        ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_CONV_START_CONV_TYPE_POS)) /**< CONV_START_CONV_TYPE Mask */
#define MXC_V_AFE_ADC_ZERO_CONV_START_CONV_TYPE_SINGLE ((uint8_t)0x0UL) /**< CONV_START_CONV_TYPE_SINGLE Value */
#define MXC_S_AFE_ADC_ZERO_CONV_START_CONV_TYPE_SINGLE (MXC_V_AFE_ADC_ZERO_CONV_START_CONV_TYPE_SINGLE << MXC_F_AFE_ADC_ZERO_CONV_START_CONV_TYPE_POS) /**< CONV_START_CONV_TYPE_SINGLE Setting */
#define MXC_V_AFE_ADC_ZERO_CONV_START_CONV_TYPE_CONTINUOUS ((uint8_t)0x1UL) /**< CONV_START_CONV_TYPE_CONTINUOUS Value */
#define MXC_S_AFE_ADC_ZERO_CONV_START_CONV_TYPE_CONTINUOUS (MXC_V_AFE_ADC_ZERO_CONV_START_CONV_TYPE_CONTINUOUS << MXC_F_AFE_ADC_ZERO_CONV_START_CONV_TYPE_POS) /**< CONV_START_CONV_TYPE_CONTINUOUS Setting */
#define MXC_V_AFE_ADC_ZERO_CONV_START_CONV_TYPE_DUTY_CYCLED_1_TO_4 ((uint8_t)0x2UL) /**< CONV_START_CONV_TYPE_DUTY_CYCLED_1_TO_4 Value */
#define MXC_S_AFE_ADC_ZERO_CONV_START_CONV_TYPE_DUTY_CYCLED_1_TO_4 (MXC_V_AFE_ADC_ZERO_CONV_START_CONV_TYPE_DUTY_CYCLED_1_TO_4 << MXC_F_AFE_ADC_ZERO_CONV_START_CONV_TYPE_POS) /**< CONV_START_CONV_TYPE_DUTY_CYCLED_1_TO_4 Setting */

#define MXC_F_AFE_ADC_ZERO_CONV_START_DEST_POS         4 /**< CONV_START_DEST Position */
#define MXC_F_AFE_ADC_ZERO_CONV_START_DEST             ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_CONV_START_DEST_POS)) /**< CONV_START_DEST Mask */

/**@} end of group AFE_ADC_ZERO_CONV_START_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SEQ_START AFE_ADC_ZERO_SEQ_START
 * @brief    Execute a sequence at written address (0x3A to 0x6E)
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SEQ_START_SEQ_ADDRESS_POS   0 /**< SEQ_START_SEQ_ADDRESS Position */
#define MXC_F_AFE_ADC_ZERO_SEQ_START_SEQ_ADDRESS       ((uint8_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_SEQ_START_SEQ_ADDRESS_POS)) /**< SEQ_START_SEQ_ADDRESS Mask */

/**@} end of group AFE_ADC_ZERO_SEQ_START_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_CAL_START AFE_ADC_ZERO_CAL_START
 * @brief    Execute selected calibration
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_CAL_START_CAL_TYPE_POS      0 /**< CAL_START_CAL_TYPE Position */
#define MXC_F_AFE_ADC_ZERO_CAL_START_CAL_TYPE          ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_CAL_START_CAL_TYPE_POS)) /**< CAL_START_CAL_TYPE Mask */

/**@} end of group AFE_ADC_ZERO_CAL_START_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_GP0_CTRL AFE_ADC_ZERO_GP0_CTRL
 * @brief    Control behavior of GPIO0
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_OSEL_POS       0 /**< GP0_CTRL_GP0_OSEL Position */
#define MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_OSEL           ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_OSEL_POS)) /**< GP0_CTRL_GP0_OSEL Mask */

#define MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_ISEL_POS       4 /**< GP0_CTRL_GP0_ISEL Position */
#define MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_ISEL           ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_ISEL_POS)) /**< GP0_CTRL_GP0_ISEL Mask */

#define MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_DIR_POS        6 /**< GP0_CTRL_GP0_DIR Position */
#define MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_DIR            ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_DIR_POS)) /**< GP0_CTRL_GP0_DIR Mask */

/**@} end of group AFE_ADC_ZERO_GP0_CTRL_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_GP1_CTRL AFE_ADC_ZERO_GP1_CTRL
 * @brief    Control behavior of GPIO1
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_OSEL_POS       0 /**< GP1_CTRL_GP1_OSEL Position */
#define MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_OSEL           ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_OSEL_POS)) /**< GP1_CTRL_GP1_OSEL Mask */

#define MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_ISEL_POS       4 /**< GP1_CTRL_GP1_ISEL Position */
#define MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_ISEL           ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_ISEL_POS)) /**< GP1_CTRL_GP1_ISEL Mask */

#define MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_DIR_POS        6 /**< GP1_CTRL_GP1_DIR Position */
#define MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_DIR            ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_GP1_CTRL_GP1_DIR_POS)) /**< GP1_CTRL_GP1_DIR Mask */

/**@} end of group AFE_ADC_ZERO_GP1_CTRL_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_GP_CONV AFE_ADC_ZERO_GP_CONV
 * @brief    Select conversion to perform when initiated by GPIO
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_POS    0 /**< GP_CONV_GP_CONV_TYPE Position */
#define MXC_F_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE        ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_POS)) /**< GP_CONV_GP_CONV_TYPE Mask */
#define MXC_V_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_SINGLE ((uint8_t)0x0UL) /**< GP_CONV_GP_CONV_TYPE_SINGLE Value */
#define MXC_S_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_SINGLE (MXC_V_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_SINGLE << MXC_F_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_POS) /**< GP_CONV_GP_CONV_TYPE_SINGLE Setting */
#define MXC_V_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_CONTINUOUS ((uint8_t)0x1UL) /**< GP_CONV_GP_CONV_TYPE_CONTINUOUS Value */
#define MXC_S_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_CONTINUOUS (MXC_V_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_CONTINUOUS << MXC_F_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_POS) /**< GP_CONV_GP_CONV_TYPE_CONTINUOUS Setting */
#define MXC_V_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_DUTY_CYCLED_1_TO_4 ((uint8_t)0x2UL) /**< GP_CONV_GP_CONV_TYPE_DUTY_CYCLED_1_TO_4 Value */
#define MXC_S_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_DUTY_CYCLED_1_TO_4 (MXC_V_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_DUTY_CYCLED_1_TO_4 << MXC_F_AFE_ADC_ZERO_GP_CONV_GP_CONV_TYPE_POS) /**< GP_CONV_GP_CONV_TYPE_DUTY_CYCLED_1_TO_4 Setting */

#define MXC_F_AFE_ADC_ZERO_GP_CONV_GP_DEST_POS         4 /**< GP_CONV_GP_DEST Position */
#define MXC_F_AFE_ADC_ZERO_GP_CONV_GP_DEST             ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_GP_CONV_GP_DEST_POS)) /**< GP_CONV_GP_DEST Mask */

/**@} end of group AFE_ADC_ZERO_GP_CONV_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_GP_SEQ_ADDR AFE_ADC_ZERO_GP_SEQ_ADDR
 * @brief    Select target sequencer address initiated by GPIO
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_GP_SEQ_ADDR_GP_SEQ_ADDR_POS 0 /**< GP_SEQ_ADDR_GP_SEQ_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_GP_SEQ_ADDR_GP_SEQ_ADDR     ((uint8_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_GP_SEQ_ADDR_GP_SEQ_ADDR_POS)) /**< GP_SEQ_ADDR_GP_SEQ_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_GP_SEQ_ADDR_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_FILTER AFE_ADC_ZERO_FILTER
 * @brief    Select conversion data rate and filter behavior
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_FILTER_RATE_POS             0 /**< FILTER_RATE Position */
#define MXC_F_AFE_ADC_ZERO_FILTER_RATE                 ((uint8_t)(0xFUL << MXC_F_AFE_ADC_ZERO_FILTER_RATE_POS)) /**< FILTER_RATE Mask */

#define MXC_F_AFE_ADC_ZERO_FILTER_LINEF_POS            4 /**< FILTER_LINEF Position */
#define MXC_F_AFE_ADC_ZERO_FILTER_LINEF                ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_FILTER_LINEF_POS)) /**< FILTER_LINEF Mask */
#define MXC_V_AFE_ADC_ZERO_FILTER_LINEF_SIMULTANEOUS_FIR_REJECT_50HZ_AND_60HZ ((uint8_t)0x0UL) /**< FILTER_LINEF_SIMULTANEOUS_FIR_REJECT_50HZ_AND_60HZ Value */
#define MXC_S_AFE_ADC_ZERO_FILTER_LINEF_SIMULTANEOUS_FIR_REJECT_50HZ_AND_60HZ (MXC_V_AFE_ADC_ZERO_FILTER_LINEF_SIMULTANEOUS_FIR_REJECT_50HZ_AND_60HZ << MXC_F_AFE_ADC_ZERO_FILTER_LINEF_POS) /**< FILTER_LINEF_SIMULTANEOUS_FIR_REJECT_50HZ_AND_60HZ Setting */
#define MXC_V_AFE_ADC_ZERO_FILTER_LINEF_FIR_REJECT_50HZ ((uint8_t)0x1UL) /**< FILTER_LINEF_FIR_REJECT_50HZ Value */
#define MXC_S_AFE_ADC_ZERO_FILTER_LINEF_FIR_REJECT_50HZ (MXC_V_AFE_ADC_ZERO_FILTER_LINEF_FIR_REJECT_50HZ << MXC_F_AFE_ADC_ZERO_FILTER_LINEF_POS) /**< FILTER_LINEF_FIR_REJECT_50HZ Setting */
#define MXC_V_AFE_ADC_ZERO_FILTER_LINEF_FIR_REJECT_60HZ ((uint8_t)0x2UL) /**< FILTER_LINEF_FIR_REJECT_60HZ Value */
#define MXC_S_AFE_ADC_ZERO_FILTER_LINEF_FIR_REJECT_60HZ (MXC_V_AFE_ADC_ZERO_FILTER_LINEF_FIR_REJECT_60HZ << MXC_F_AFE_ADC_ZERO_FILTER_LINEF_POS) /**< FILTER_LINEF_FIR_REJECT_60HZ Setting */
#define MXC_V_AFE_ADC_ZERO_FILTER_LINEF_SINC4          ((uint8_t)0x3UL) /**< FILTER_LINEF_SINC4 Value */
#define MXC_S_AFE_ADC_ZERO_FILTER_LINEF_SINC4          (MXC_V_AFE_ADC_ZERO_FILTER_LINEF_SINC4 << MXC_F_AFE_ADC_ZERO_FILTER_LINEF_POS) /**< FILTER_LINEF_SINC4 Setting */

/**@} end of group AFE_ADC_ZERO_FILTER_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_CTRL AFE_ADC_ZERO_CTRL
 * @brief    Select clock source, data format, ref inputs and ref buffers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS            0 /**< CTRL_REF_SEL Position */
#define MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL                ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS)) /**< CTRL_REF_SEL Mask */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF0P_AND_REF0N ((uint8_t)0x0UL) /**< CTRL_REF_SEL_REF0P_AND_REF0N Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_REF0P_AND_REF0N (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF0P_AND_REF0N << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_REF0P_AND_REF0N Setting */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF1P_AND_REF1N ((uint8_t)0x1UL) /**< CTRL_REF_SEL_REF1P_AND_REF1N Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_REF1P_AND_REF1N (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF1P_AND_REF1N << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_REF1P_AND_REF1N Setting */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF2P_AND_REF2N ((uint8_t)0x2UL) /**< CTRL_REF_SEL_REF2P_AND_REF2N Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_REF2P_AND_REF2N (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF2P_AND_REF2N << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_REF2P_AND_REF2N Setting */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_AVDD_AND_AGND  ((uint8_t)0x3UL) /**< CTRL_REF_SEL_AVDD_AND_AGND Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_AVDD_AND_AGND  (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_AVDD_AND_AGND << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_AVDD_AND_AGND Setting */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF0P_AND_AGND ((uint8_t)0x4UL) /**< CTRL_REF_SEL_REF0P_AND_AGND Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_REF0P_AND_AGND (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF0P_AND_AGND << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_REF0P_AND_AGND Setting */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF1P_AND_AGND ((uint8_t)0x5UL) /**< CTRL_REF_SEL_REF1P_AND_AGND Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_REF1P_AND_AGND (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF1P_AND_AGND << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_REF1P_AND_AGND Setting */
#define MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF2P_AND_AGND ((uint8_t)0x6UL) /**< CTRL_REF_SEL_REF2P_AND_AGND Value */
#define MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_REF2P_AND_AGND (MXC_V_AFE_ADC_ZERO_CTRL_REF_SEL_REF2P_AND_AGND << MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL_POS) /**< CTRL_REF_SEL_REF2P_AND_AGND Setting */

#define MXC_F_AFE_ADC_ZERO_CTRL_REFBUFN_EN_POS         3 /**< CTRL_REFBUFN_EN Position */
#define MXC_F_AFE_ADC_ZERO_CTRL_REFBUFN_EN             ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_CTRL_REFBUFN_EN_POS)) /**< CTRL_REFBUFN_EN Mask */

#define MXC_F_AFE_ADC_ZERO_CTRL_REFBUFP_EN_POS         4 /**< CTRL_REFBUFP_EN Position */
#define MXC_F_AFE_ADC_ZERO_CTRL_REFBUFP_EN             ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_CTRL_REFBUFP_EN_POS)) /**< CTRL_REFBUFP_EN Mask */

#define MXC_F_AFE_ADC_ZERO_CTRL_FORMAT_POS             5 /**< CTRL_FORMAT Position */
#define MXC_F_AFE_ADC_ZERO_CTRL_FORMAT                 ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_CTRL_FORMAT_POS)) /**< CTRL_FORMAT Mask */

#define MXC_F_AFE_ADC_ZERO_CTRL_U_BN_POS               6 /**< CTRL_U_BN Position */
#define MXC_F_AFE_ADC_ZERO_CTRL_U_BN                   ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_CTRL_U_BN_POS)) /**< CTRL_U_BN Mask */

#define MXC_F_AFE_ADC_ZERO_CTRL_EXTCLK_POS             7 /**< CTRL_EXTCLK Position */
#define MXC_F_AFE_ADC_ZERO_CTRL_EXTCLK                 ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_CTRL_EXTCLK_POS)) /**< CTRL_EXTCLK Mask */

/**@} end of group AFE_ADC_ZERO_CTRL_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SOURCE AFE_ADC_ZERO_SOURCE
 * @brief    Configures excitation current, burnout current, and bias voltage sources.
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS        0 /**< SOURCE_IDAC_MODE Position */
#define MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE            ((uint8_t)(0xFUL << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS)) /**< SOURCE_IDAC_MODE Mask */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_10UA   ((uint8_t)0x0UL) /**< SOURCE_IDAC_MODE_CUR_10UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_10UA   (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_10UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_10UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_50UA   ((uint8_t)0x1UL) /**< SOURCE_IDAC_MODE_CUR_50UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_50UA   (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_50UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_50UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_75UA   ((uint8_t)0x2UL) /**< SOURCE_IDAC_MODE_CUR_75UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_75UA   (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_75UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_75UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_100UA  ((uint8_t)0x3UL) /**< SOURCE_IDAC_MODE_CUR_100UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_100UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_100UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_100UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_125UA  ((uint8_t)0x4UL) /**< SOURCE_IDAC_MODE_CUR_125UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_125UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_125UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_125UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_150UA  ((uint8_t)0x5UL) /**< SOURCE_IDAC_MODE_CUR_150UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_150UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_150UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_150UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_175UA  ((uint8_t)0x6UL) /**< SOURCE_IDAC_MODE_CUR_175UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_175UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_175UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_175UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_200UA  ((uint8_t)0x7UL) /**< SOURCE_IDAC_MODE_CUR_200UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_200UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_200UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_200UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_225UA  ((uint8_t)0x8UL) /**< SOURCE_IDAC_MODE_CUR_225UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_225UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_225UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_225UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_250UA  ((uint8_t)0x9UL) /**< SOURCE_IDAC_MODE_CUR_250UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_250UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_250UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_250UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_300UA  ((uint8_t)0xAUL) /**< SOURCE_IDAC_MODE_CUR_300UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_300UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_300UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_300UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_400UA  ((uint8_t)0xBUL) /**< SOURCE_IDAC_MODE_CUR_400UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_400UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_400UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_400UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_600UA  ((uint8_t)0xCUL) /**< SOURCE_IDAC_MODE_CUR_600UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_600UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_600UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_600UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_800UA  ((uint8_t)0xDUL) /**< SOURCE_IDAC_MODE_CUR_800UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_800UA  (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_800UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_800UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_1200UA ((uint8_t)0xEUL) /**< SOURCE_IDAC_MODE_CUR_1200UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_1200UA (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_1200UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_1200UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_1600UA ((uint8_t)0xFUL) /**< SOURCE_IDAC_MODE_CUR_1600UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_1600UA (MXC_V_AFE_ADC_ZERO_SOURCE_IDAC_MODE_CUR_1600UA << MXC_F_AFE_ADC_ZERO_SOURCE_IDAC_MODE_POS) /**< SOURCE_IDAC_MODE_CUR_1600UA Setting */

#define MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE_POS         4 /**< SOURCE_BRN_MODE Position */
#define MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE             ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE_POS)) /**< SOURCE_BRN_MODE Mask */
#define MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_POW_DOWN_CUR_SRC_DISABLED ((uint8_t)0x0UL) /**< SOURCE_BRN_MODE_POW_DOWN_CUR_SRC_DISABLED Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_BRN_MODE_POW_DOWN_CUR_SRC_DISABLED (MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_POW_DOWN_CUR_SRC_DISABLED << MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE_POS) /**< SOURCE_BRN_MODE_POW_DOWN_CUR_SRC_DISABLED Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_0_5UA ((uint8_t)0x1UL) /**< SOURCE_BRN_MODE_CUR_SRC_0_5UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_0_5UA (MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_0_5UA << MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE_POS) /**< SOURCE_BRN_MODE_CUR_SRC_0_5UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_1UA ((uint8_t)0x2UL) /**< SOURCE_BRN_MODE_CUR_SRC_1UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_1UA (MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_1UA << MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE_POS) /**< SOURCE_BRN_MODE_CUR_SRC_1UA Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_10_UA ((uint8_t)0x3UL) /**< SOURCE_BRN_MODE_CUR_SRC_10_UA Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_10_UA (MXC_V_AFE_ADC_ZERO_SOURCE_BRN_MODE_CUR_SRC_10_UA << MXC_F_AFE_ADC_ZERO_SOURCE_BRN_MODE_POS) /**< SOURCE_BRN_MODE_CUR_SRC_10_UA Setting */

#define MXC_F_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_POS       6 /**< SOURCE_VBIAS_MODE Position */
#define MXC_F_AFE_ADC_ZERO_SOURCE_VBIAS_MODE           ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_POS)) /**< SOURCE_VBIAS_MODE Mask */
#define MXC_V_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_ACTIVE_MODE ((uint8_t)0x0UL) /**< SOURCE_VBIAS_MODE_ACTIVE_MODE Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_ACTIVE_MODE (MXC_V_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_ACTIVE_MODE << MXC_F_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_POS) /**< SOURCE_VBIAS_MODE_ACTIVE_MODE Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_125K_OHM ((uint8_t)0x1UL) /**< SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_125K_OHM Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_125K_OHM (MXC_V_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_125K_OHM << MXC_F_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_POS) /**< SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_125K_OHM Setting */
#define MXC_V_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_20K_OHM ((uint8_t)0x2UL) /**< SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_20K_OHM Value */
#define MXC_S_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_20K_OHM (MXC_V_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_20K_OHM << MXC_F_AFE_ADC_ZERO_SOURCE_VBIAS_MODE_POS) /**< SOURCE_VBIAS_MODE_OUTPUT_IMPEDANCE_20K_OHM Setting */

/**@} end of group AFE_ADC_ZERO_SOURCE_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_MUX_CTRL0 AFE_ADC_ZERO_MUX_CTRL0
 * @brief    Selects analog inputs for AINP and AINN
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINN_SEL_POS      0 /**< MUX_CTRL0_AINN_SEL Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINN_SEL          ((uint8_t)(0xFUL << MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINN_SEL_POS)) /**< MUX_CTRL0_AINN_SEL Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINP_SEL_POS      4 /**< MUX_CTRL0_AINP_SEL Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINP_SEL          ((uint8_t)(0xFUL << MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINP_SEL_POS)) /**< MUX_CTRL0_AINP_SEL Mask */

/**@} end of group AFE_ADC_ZERO_MUX_CTRL0_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_MUX_CTRL1 AFE_ADC_ZERO_MUX_CTRL1
 * @brief    Selects excitation current sources and which input they are connected to
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL1_IDAC0_SEL_POS     0 /**< MUX_CTRL1_IDAC0_SEL Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL1_IDAC0_SEL         ((uint8_t)(0xFUL << MXC_F_AFE_ADC_ZERO_MUX_CTRL1_IDAC0_SEL_POS)) /**< MUX_CTRL1_IDAC0_SEL Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL1_IDAC1_SEL_POS     4 /**< MUX_CTRL1_IDAC1_SEL Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL1_IDAC1_SEL         ((uint8_t)(0xFUL << MXC_F_AFE_ADC_ZERO_MUX_CTRL1_IDAC1_SEL_POS)) /**< MUX_CTRL1_IDAC1_SEL Mask */

/**@} end of group AFE_ADC_ZERO_MUX_CTRL1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_MUX_CTRL2 AFE_ADC_ZERO_MUX_CTRL2
 * @brief    Connection of VBIAS source to input mux
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_0_POS   0 /**< MUX_CTRL2_VBIAS_SEL_0 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_0       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_0_POS)) /**< MUX_CTRL2_VBIAS_SEL_0 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_1_POS   1 /**< MUX_CTRL2_VBIAS_SEL_1 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_1       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_1_POS)) /**< MUX_CTRL2_VBIAS_SEL_1 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_2_POS   2 /**< MUX_CTRL2_VBIAS_SEL_2 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_2       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_2_POS)) /**< MUX_CTRL2_VBIAS_SEL_2 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_3_POS   3 /**< MUX_CTRL2_VBIAS_SEL_3 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_3       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_3_POS)) /**< MUX_CTRL2_VBIAS_SEL_3 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_4_POS   4 /**< MUX_CTRL2_VBIAS_SEL_4 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_4       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_4_POS)) /**< MUX_CTRL2_VBIAS_SEL_4 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_5_POS   5 /**< MUX_CTRL2_VBIAS_SEL_5 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_5       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_5_POS)) /**< MUX_CTRL2_VBIAS_SEL_5 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_6_POS   6 /**< MUX_CTRL2_VBIAS_SEL_6 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_6       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_6_POS)) /**< MUX_CTRL2_VBIAS_SEL_6 Mask */

#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_7_POS   7 /**< MUX_CTRL2_VBIAS_SEL_7 Position */
#define MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_7       ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_MUX_CTRL2_VBIAS_SEL_7_POS)) /**< MUX_CTRL2_VBIAS_SEL_7 Mask */

/**@} end of group AFE_ADC_ZERO_MUX_CTRL2_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_PGA AFE_ADC_ZERO_PGA
 * @brief    Signal path control of input buffers and PGA
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS                0 /**< PGA_GAIN Position */
#define MXC_F_AFE_ADC_ZERO_PGA_GAIN                    ((uint8_t)(0x7UL << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS)) /**< PGA_GAIN Mask */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_1X            ((uint8_t)0x0UL) /**< PGA_GAIN_GAIN_1X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_1X            (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_1X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_1X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_2X            ((uint8_t)0x1UL) /**< PGA_GAIN_GAIN_2X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_2X            (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_2X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_2X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_4X            ((uint8_t)0x2UL) /**< PGA_GAIN_GAIN_4X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_4X            (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_4X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_4X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_8X            ((uint8_t)0x3UL) /**< PGA_GAIN_GAIN_8X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_8X            (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_8X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_8X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_16X           ((uint8_t)0x4UL) /**< PGA_GAIN_GAIN_16X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_16X           (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_16X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_16X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_32X           ((uint8_t)0x5UL) /**< PGA_GAIN_GAIN_32X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_32X           (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_32X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_32X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_64X           ((uint8_t)0x6UL) /**< PGA_GAIN_GAIN_64X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_64X           (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_64X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_64X Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_128X          ((uint8_t)0x7UL) /**< PGA_GAIN_GAIN_128X Value */
#define MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_128X          (MXC_V_AFE_ADC_ZERO_PGA_GAIN_GAIN_128X << MXC_F_AFE_ADC_ZERO_PGA_GAIN_POS) /**< PGA_GAIN_GAIN_128X Setting */

#define MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH_POS            4 /**< PGA_SIG_PATH Position */
#define MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH                ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH_POS)) /**< PGA_SIG_PATH Mask */
#define MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_BUFFERED_UNITY_GAIN_PATH ((uint8_t)0x0UL) /**< PGA_SIG_PATH_BUFFERED_UNITY_GAIN_PATH Value */
#define MXC_S_AFE_ADC_ZERO_PGA_SIG_PATH_BUFFERED_UNITY_GAIN_PATH (MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_BUFFERED_UNITY_GAIN_PATH << MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH_POS) /**< PGA_SIG_PATH_BUFFERED_UNITY_GAIN_PATH Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_BYPASS_PATH    ((uint8_t)0x1UL) /**< PGA_SIG_PATH_BYPASS_PATH Value */
#define MXC_S_AFE_ADC_ZERO_PGA_SIG_PATH_BYPASS_PATH    (MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_BYPASS_PATH << MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH_POS) /**< PGA_SIG_PATH_BYPASS_PATH Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_PGA_PATH       ((uint8_t)0x2UL) /**< PGA_SIG_PATH_PGA_PATH Value */
#define MXC_S_AFE_ADC_ZERO_PGA_SIG_PATH_PGA_PATH       (MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_PGA_PATH << MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH_POS) /**< PGA_SIG_PATH_PGA_PATH Setting */
#define MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_RESERVED       ((uint8_t)0x3UL) /**< PGA_SIG_PATH_RESERVED Value */
#define MXC_S_AFE_ADC_ZERO_PGA_SIG_PATH_RESERVED       (MXC_V_AFE_ADC_ZERO_PGA_SIG_PATH_RESERVED << MXC_F_AFE_ADC_ZERO_PGA_SIG_PATH_POS) /**< PGA_SIG_PATH_RESERVED Setting */

/**@} end of group AFE_ADC_ZERO_PGA_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_WAIT_EXT AFE_ADC_ZERO_WAIT_EXT
 * @brief    Extends count range of WAIT command
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_WAIT_EXT_WAIT_EXT_POS       0 /**< WAIT_EXT_WAIT_EXT Position */
#define MXC_F_AFE_ADC_ZERO_WAIT_EXT_WAIT_EXT           ((uint8_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_WAIT_EXT_WAIT_EXT_POS)) /**< WAIT_EXT_WAIT_EXT Mask */

/**@} end of group AFE_ADC_ZERO_WAIT_EXT_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_PART_ID AFE_ADC_ZERO_PART_ID
 * @brief    Silicon Revision ID
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_PART_ID_REV_ID_POS          0 /**< PART_ID_REV_ID Position */
#define MXC_F_AFE_ADC_ZERO_PART_ID_REV_ID              ((uint32_t)(0x7UL << MXC_F_AFE_ADC_ZERO_PART_ID_REV_ID_POS)) /**< PART_ID_REV_ID Mask */

/**@} end of group AFE_ADC_ZERO_PART_ID_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SYSC_SEL AFE_ADC_ZERO_SYSC_SEL
 * @brief    System Calibration Selection
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_0_POS     0 /**< SYSC_SEL_SYSC_SEL_0 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_0         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_0_POS)) /**< SYSC_SEL_SYSC_SEL_0 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_1_POS     2 /**< SYSC_SEL_SYSC_SEL_1 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_1         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_1_POS)) /**< SYSC_SEL_SYSC_SEL_1 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_2_POS     4 /**< SYSC_SEL_SYSC_SEL_2 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_2         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_2_POS)) /**< SYSC_SEL_SYSC_SEL_2 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_3_POS     6 /**< SYSC_SEL_SYSC_SEL_3 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_3         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_3_POS)) /**< SYSC_SEL_SYSC_SEL_3 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_4_POS     8 /**< SYSC_SEL_SYSC_SEL_4 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_4         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_4_POS)) /**< SYSC_SEL_SYSC_SEL_4 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_5_POS     10 /**< SYSC_SEL_SYSC_SEL_5 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_5         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_5_POS)) /**< SYSC_SEL_SYSC_SEL_5 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_6_POS     12 /**< SYSC_SEL_SYSC_SEL_6 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_6         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_6_POS)) /**< SYSC_SEL_SYSC_SEL_6 Mask */

#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_7_POS     14 /**< SYSC_SEL_SYSC_SEL_7 Position */
#define MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_7         ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYSC_SEL_SYSC_SEL_7_POS)) /**< SYSC_SEL_SYSC_SEL_7 Mask */

/**@} end of group AFE_ADC_ZERO_SYSC_SEL_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SYS_OFF_A AFE_ADC_ZERO_SYS_OFF_A
 * @brief    System offset A calibration value to subtract from conversion results
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SYS_OFF_A_SYS_OFF_A_POS     0 /**< SYS_OFF_A_SYS_OFF_A Position */
#define MXC_F_AFE_ADC_ZERO_SYS_OFF_A_SYS_OFF_A         ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SYS_OFF_A_SYS_OFF_A_POS)) /**< SYS_OFF_A_SYS_OFF_A Mask */

/**@} end of group AFE_ADC_ZERO_SYS_OFF_A_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SYS_OFF_B AFE_ADC_ZERO_SYS_OFF_B
 * @brief    System offset B calibration value to subtract from conversion results
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SYS_OFF_B_SYS_OFF_B_POS     0 /**< SYS_OFF_B_SYS_OFF_B Position */
#define MXC_F_AFE_ADC_ZERO_SYS_OFF_B_SYS_OFF_B         ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SYS_OFF_B_SYS_OFF_B_POS)) /**< SYS_OFF_B_SYS_OFF_B Mask */

/**@} end of group AFE_ADC_ZERO_SYS_OFF_B_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SYS_GAIN_A AFE_ADC_ZERO_SYS_GAIN_A
 * @brief    System gain calibration A value to scale offset corrected conversion results
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SYS_GAIN_A_SYS_GAIN_A_POS   0 /**< SYS_GAIN_A_SYS_GAIN_A Position */
#define MXC_F_AFE_ADC_ZERO_SYS_GAIN_A_SYS_GAIN_A       ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SYS_GAIN_A_SYS_GAIN_A_POS)) /**< SYS_GAIN_A_SYS_GAIN_A Mask */

/**@} end of group AFE_ADC_ZERO_SYS_GAIN_A_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SYS_GAIN_B AFE_ADC_ZERO_SYS_GAIN_B
 * @brief    System gain calibration B value to scale offset corrected conversion results
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SYS_GAIN_B_SYS_GAIN_B_POS   0 /**< SYS_GAIN_B_SYS_GAIN_B Position */
#define MXC_F_AFE_ADC_ZERO_SYS_GAIN_B_SYS_GAIN_B       ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SYS_GAIN_B_SYS_GAIN_B_POS)) /**< SYS_GAIN_B_SYS_GAIN_B Mask */

/**@} end of group AFE_ADC_ZERO_SYS_GAIN_B_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_OFF AFE_ADC_ZERO_SELF_OFF
 * @brief    Self-calibration offset value to subtract from conversion results
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_OFF_SELF_OFF_POS       0 /**< SELF_OFF_SELF_OFF Position */
#define MXC_F_AFE_ADC_ZERO_SELF_OFF_SELF_OFF           ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_OFF_SELF_OFF_POS)) /**< SELF_OFF_SELF_OFF Mask */

/**@} end of group AFE_ADC_ZERO_SELF_OFF_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_1 AFE_ADC_ZERO_SELF_GAIN_1
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 1
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_1_SELF_GAIN_1_POS 0 /**< SELF_GAIN_1_SELF_GAIN_1 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_1_SELF_GAIN_1     ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_1_SELF_GAIN_1_POS)) /**< SELF_GAIN_1_SELF_GAIN_1 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_2 AFE_ADC_ZERO_SELF_GAIN_2
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 2
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_2_SELF_GAIN_1_POS 0 /**< SELF_GAIN_2_SELF_GAIN_1 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_2_SELF_GAIN_1     ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_2_SELF_GAIN_1_POS)) /**< SELF_GAIN_2_SELF_GAIN_1 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_2_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_4 AFE_ADC_ZERO_SELF_GAIN_4
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 4
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_4_SELF_GAIN_4_POS 0 /**< SELF_GAIN_4_SELF_GAIN_4 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_4_SELF_GAIN_4     ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_4_SELF_GAIN_4_POS)) /**< SELF_GAIN_4_SELF_GAIN_4 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_4_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_8 AFE_ADC_ZERO_SELF_GAIN_8
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 8
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_8_SELF_GAIN_8_POS 0 /**< SELF_GAIN_8_SELF_GAIN_8 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_8_SELF_GAIN_8     ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_8_SELF_GAIN_8_POS)) /**< SELF_GAIN_8_SELF_GAIN_8 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_8_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_16 AFE_ADC_ZERO_SELF_GAIN_16
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 16
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_16_SELF_GAIN_16_POS 0 /**< SELF_GAIN_16_SELF_GAIN_16 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_16_SELF_GAIN_16   ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_16_SELF_GAIN_16_POS)) /**< SELF_GAIN_16_SELF_GAIN_16 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_16_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_32 AFE_ADC_ZERO_SELF_GAIN_32
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 32
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_32_SELF_GAIN_32_POS 0 /**< SELF_GAIN_32_SELF_GAIN_32 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_32_SELF_GAIN_32   ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_32_SELF_GAIN_32_POS)) /**< SELF_GAIN_32_SELF_GAIN_32 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_32_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_64 AFE_ADC_ZERO_SELF_GAIN_64
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 64
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_64_SELF_GAIN_64_POS 0 /**< SELF_GAIN_64_SELF_GAIN_64 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_64_SELF_GAIN_64   ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_64_SELF_GAIN_64_POS)) /**< SELF_GAIN_64_SELF_GAIN_64 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_64_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SELF_GAIN_128 AFE_ADC_ZERO_SELF_GAIN_128
 * @brief    Self-calibration gain value to scale offset corrected conversion results for
 *           gain 128
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_128_LTHRESH0_POS  0 /**< SELF_GAIN_128_LTHRESH0 Position */
#define MXC_F_AFE_ADC_ZERO_SELF_GAIN_128_LTHRESH0      ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_SELF_GAIN_128_LTHRESH0_POS)) /**< SELF_GAIN_128_LTHRESH0 Mask */

/**@} end of group AFE_ADC_ZERO_SELF_GAIN_128_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH0 AFE_ADC_ZERO_LTHRESH0
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH0_LTHRESH_POS        0 /**< LTHRESH0_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH0_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH0_LTHRESH_POS)) /**< LTHRESH0_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH0_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH1 AFE_ADC_ZERO_LTHRESH1
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH1_LTHRESH_POS        0 /**< LTHRESH1_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH1_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH1_LTHRESH_POS)) /**< LTHRESH1_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH2 AFE_ADC_ZERO_LTHRESH2
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH2_LTHRESH_POS        0 /**< LTHRESH2_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH2_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH2_LTHRESH_POS)) /**< LTHRESH2_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH2_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH3 AFE_ADC_ZERO_LTHRESH3
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH3_LTHRESH_POS        0 /**< LTHRESH3_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH3_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH3_LTHRESH_POS)) /**< LTHRESH3_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH3_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH4 AFE_ADC_ZERO_LTHRESH4
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH4_LTHRESH_POS        0 /**< LTHRESH4_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH4_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH4_LTHRESH_POS)) /**< LTHRESH4_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH4_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH5 AFE_ADC_ZERO_LTHRESH5
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH5_LTHRESH_POS        0 /**< LTHRESH5_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH5_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH5_LTHRESH_POS)) /**< LTHRESH5_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH5_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH6 AFE_ADC_ZERO_LTHRESH6
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH6_LTHRESH_POS        0 /**< LTHRESH6_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH6_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH6_LTHRESH_POS)) /**< LTHRESH6_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH6_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_LTHRESH7 AFE_ADC_ZERO_LTHRESH7
 * @brief    Lower comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_LTHRESH7_LTHRESH_POS        0 /**< LTHRESH7_LTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_LTHRESH7_LTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_LTHRESH7_LTHRESH_POS)) /**< LTHRESH7_LTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_LTHRESH7_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH0 AFE_ADC_ZERO_UTHRESH0
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH0_UTHRESH_POS        0 /**< UTHRESH0_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH0_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH0_UTHRESH_POS)) /**< UTHRESH0_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH0_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH1 AFE_ADC_ZERO_UTHRESH1
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH1_UTHRESH_POS        0 /**< UTHRESH1_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH1_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH1_UTHRESH_POS)) /**< UTHRESH1_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH2 AFE_ADC_ZERO_UTHRESH2
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH2_UTHRESH_POS        0 /**< UTHRESH2_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH2_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH2_UTHRESH_POS)) /**< UTHRESH2_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH2_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH3 AFE_ADC_ZERO_UTHRESH3
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH3_UTHRESH_POS        0 /**< UTHRESH3_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH3_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH3_UTHRESH_POS)) /**< UTHRESH3_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH3_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH4 AFE_ADC_ZERO_UTHRESH4
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH4_UTHRESH_POS        0 /**< UTHRESH4_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH4_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH4_UTHRESH_POS)) /**< UTHRESH4_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH4_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH5 AFE_ADC_ZERO_UTHRESH5
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH5_UTHRESH_POS        0 /**< UTHRESH5_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH5_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH5_UTHRESH_POS)) /**< UTHRESH5_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH5_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH6 AFE_ADC_ZERO_UTHRESH6
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH6_UTHRESH_POS        0 /**< UTHRESH6_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH6_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH6_UTHRESH_POS)) /**< UTHRESH6_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH6_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UTHRESH7 AFE_ADC_ZERO_UTHRESH7
 * @brief    Upper comparison threshold for DATA registers
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UTHRESH7_UTHRESH_POS        0 /**< UTHRESH7_UTHRESH Position */
#define MXC_F_AFE_ADC_ZERO_UTHRESH7_UTHRESH            ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_UTHRESH7_UTHRESH_POS)) /**< UTHRESH7_UTHRESH Mask */

/**@} end of group AFE_ADC_ZERO_UTHRESH7_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA0 AFE_ADC_ZERO_DATA0
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA0_DATA0_POS             0 /**< DATA0_DATA0 Position */
#define MXC_F_AFE_ADC_ZERO_DATA0_DATA0                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA0_DATA0_POS)) /**< DATA0_DATA0 Mask */

/**@} end of group AFE_ADC_ZERO_DATA0_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA1 AFE_ADC_ZERO_DATA1
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA1_DATA1_POS             0 /**< DATA1_DATA1 Position */
#define MXC_F_AFE_ADC_ZERO_DATA1_DATA1                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA1_DATA1_POS)) /**< DATA1_DATA1 Mask */

/**@} end of group AFE_ADC_ZERO_DATA1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA2 AFE_ADC_ZERO_DATA2
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA2_DATA2_POS             0 /**< DATA2_DATA2 Position */
#define MXC_F_AFE_ADC_ZERO_DATA2_DATA2                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA2_DATA2_POS)) /**< DATA2_DATA2 Mask */

/**@} end of group AFE_ADC_ZERO_DATA2_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA3 AFE_ADC_ZERO_DATA3
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA3_DATA3_POS             0 /**< DATA3_DATA3 Position */
#define MXC_F_AFE_ADC_ZERO_DATA3_DATA3                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA3_DATA3_POS)) /**< DATA3_DATA3 Mask */

/**@} end of group AFE_ADC_ZERO_DATA3_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA4 AFE_ADC_ZERO_DATA4
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA4_DATA4_POS             0 /**< DATA4_DATA4 Position */
#define MXC_F_AFE_ADC_ZERO_DATA4_DATA4                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA4_DATA4_POS)) /**< DATA4_DATA4 Mask */

/**@} end of group AFE_ADC_ZERO_DATA4_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA5 AFE_ADC_ZERO_DATA5
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA5_DATA5_POS             0 /**< DATA5_DATA5 Position */
#define MXC_F_AFE_ADC_ZERO_DATA5_DATA5                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA5_DATA5_POS)) /**< DATA5_DATA5 Mask */

/**@} end of group AFE_ADC_ZERO_DATA5_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA6 AFE_ADC_ZERO_DATA6
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA6_DATA6_POS             0 /**< DATA6_DATA6 Position */
#define MXC_F_AFE_ADC_ZERO_DATA6_DATA6                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA6_DATA6_POS)) /**< DATA6_DATA6 Mask */

/**@} end of group AFE_ADC_ZERO_DATA6_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_DATA7 AFE_ADC_ZERO_DATA7
 * @brief    ADC conversion result storage
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_DATA7_DATA7_POS             0 /**< DATA7_DATA7 Position */
#define MXC_F_AFE_ADC_ZERO_DATA7_DATA7                 ((uint32_t)(0xFFFFFFUL << MXC_F_AFE_ADC_ZERO_DATA7_DATA7_POS)) /**< DATA7_DATA7 Mask */

/**@} end of group AFE_ADC_ZERO_DATA7_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_STATUS AFE_ADC_ZERO_STATUS
 * @brief    Device Status, INTB source etc.
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_STATUS_CONV_RDY_POS         0 /**< STATUS_CONV_RDY Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_CONV_RDY             ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_CONV_RDY_POS)) /**< STATUS_CONV_RDY Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_SEQ_RDY_POS          1 /**< STATUS_SEQ_RDY Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_SEQ_RDY              ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_SEQ_RDY_POS)) /**< STATUS_SEQ_RDY Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_CAL_RDY_POS          2 /**< STATUS_CAL_RDY Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_CAL_RDY              ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_CAL_RDY_POS)) /**< STATUS_CAL_RDY Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_WAIT_DONE_POS        3 /**< STATUS_WAIT_DONE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_WAIT_DONE            ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_WAIT_DONE_POS)) /**< STATUS_WAIT_DONE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_DATA_RDY_POS         4 /**< STATUS_DATA_RDY Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_DATA_RDY             ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_DATA_RDY_POS)) /**< STATUS_DATA_RDY Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_SYSGOR_POS           7 /**< STATUS_SYSGOR Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_SYSGOR               ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_SYSGOR_POS)) /**< STATUS_SYSGOR Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_0_POS            8 /**< STATUS_TUR_0 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_0                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_0_POS)) /**< STATUS_TUR_0 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_1_POS            9 /**< STATUS_TUR_1 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_1                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_1_POS)) /**< STATUS_TUR_1 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_2_POS            10 /**< STATUS_TUR_2 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_2                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_2_POS)) /**< STATUS_TUR_2 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_3_POS            11 /**< STATUS_TUR_3 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_3                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_3_POS)) /**< STATUS_TUR_3 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_4_POS            12 /**< STATUS_TUR_4 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_4                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_4_POS)) /**< STATUS_TUR_4 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_5_POS            13 /**< STATUS_TUR_5 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_5                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_5_POS)) /**< STATUS_TUR_5 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_6_POS            14 /**< STATUS_TUR_6 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_6                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_6_POS)) /**< STATUS_TUR_6 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_7_POS            15 /**< STATUS_TUR_7 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TUR_7                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TUR_7_POS)) /**< STATUS_TUR_7 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_0_POS            16 /**< STATUS_TOR_0 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_0                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_0_POS)) /**< STATUS_TOR_0 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_1_POS            17 /**< STATUS_TOR_1 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_1                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_1_POS)) /**< STATUS_TOR_1 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_2_POS            18 /**< STATUS_TOR_2 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_2                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_2_POS)) /**< STATUS_TOR_2 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_3_POS            19 /**< STATUS_TOR_3 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_3                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_3_POS)) /**< STATUS_TOR_3 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_4_POS            20 /**< STATUS_TOR_4 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_4                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_4_POS)) /**< STATUS_TOR_4 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_5_POS            21 /**< STATUS_TOR_5 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_5                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_5_POS)) /**< STATUS_TOR_5 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_6_POS            22 /**< STATUS_TOR_6 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_6                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_6_POS)) /**< STATUS_TOR_6 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_7_POS            23 /**< STATUS_TOR_7 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_TOR_7                ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_TOR_7_POS)) /**< STATUS_TOR_7 Mask */

/**@} end of group AFE_ADC_ZERO_STATUS_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_STATUS_IE AFE_ADC_ZERO_STATUS_IE
 * @brief    Device Status, INTB source etc. Enable Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_CONV_RDY_IE_POS   0 /**< STATUS_IE_CONV_RDY_IE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_CONV_RDY_IE       ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_CONV_RDY_IE_POS)) /**< STATUS_IE_CONV_RDY_IE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_SEQ_RDY_IE_POS    1 /**< STATUS_IE_SEQ_RDY_IE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_SEQ_RDY_IE        ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_SEQ_RDY_IE_POS)) /**< STATUS_IE_SEQ_RDY_IE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_CAL_RDY_IE_POS    2 /**< STATUS_IE_CAL_RDY_IE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_CAL_RDY_IE        ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_CAL_RDY_IE_POS)) /**< STATUS_IE_CAL_RDY_IE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_WAIT_DONE_IE_POS  3 /**< STATUS_IE_WAIT_DONE_IE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_WAIT_DONE_IE      ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_WAIT_DONE_IE_POS)) /**< STATUS_IE_WAIT_DONE_IE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_DATA_RDY_IE_POS   4 /**< STATUS_IE_DATA_RDY_IE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_DATA_RDY_IE       ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_DATA_RDY_IE_POS)) /**< STATUS_IE_DATA_RDY_IE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_SYSGOR_IE_POS     7 /**< STATUS_IE_SYSGOR_IE Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_SYSGOR_IE         ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_SYSGOR_IE_POS)) /**< STATUS_IE_SYSGOR_IE Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_0_POS      8 /**< STATUS_IE_TUR_IE_0 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_0          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_0_POS)) /**< STATUS_IE_TUR_IE_0 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_1_POS      9 /**< STATUS_IE_TUR_IE_1 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_1          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_1_POS)) /**< STATUS_IE_TUR_IE_1 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_2_POS      10 /**< STATUS_IE_TUR_IE_2 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_2          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_2_POS)) /**< STATUS_IE_TUR_IE_2 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_3_POS      11 /**< STATUS_IE_TUR_IE_3 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_3          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_3_POS)) /**< STATUS_IE_TUR_IE_3 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_4_POS      12 /**< STATUS_IE_TUR_IE_4 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_4          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_4_POS)) /**< STATUS_IE_TUR_IE_4 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_5_POS      13 /**< STATUS_IE_TUR_IE_5 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_5          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_5_POS)) /**< STATUS_IE_TUR_IE_5 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_6_POS      14 /**< STATUS_IE_TUR_IE_6 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_6          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_6_POS)) /**< STATUS_IE_TUR_IE_6 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_7_POS      15 /**< STATUS_IE_TUR_IE_7 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_7          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TUR_IE_7_POS)) /**< STATUS_IE_TUR_IE_7 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_0_POS      16 /**< STATUS_IE_TOR_IE_0 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_0          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_0_POS)) /**< STATUS_IE_TOR_IE_0 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_1_POS      17 /**< STATUS_IE_TOR_IE_1 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_1          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_1_POS)) /**< STATUS_IE_TOR_IE_1 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_2_POS      18 /**< STATUS_IE_TOR_IE_2 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_2          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_2_POS)) /**< STATUS_IE_TOR_IE_2 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_3_POS      19 /**< STATUS_IE_TOR_IE_3 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_3          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_3_POS)) /**< STATUS_IE_TOR_IE_3 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_4_POS      20 /**< STATUS_IE_TOR_IE_4 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_4          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_4_POS)) /**< STATUS_IE_TOR_IE_4 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_5_POS      21 /**< STATUS_IE_TOR_IE_5 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_5          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_5_POS)) /**< STATUS_IE_TOR_IE_5 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_6_POS      22 /**< STATUS_IE_TOR_IE_6 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_6          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_6_POS)) /**< STATUS_IE_TOR_IE_6 Mask */

#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_7_POS      23 /**< STATUS_IE_TOR_IE_7 Position */
#define MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_7          ((uint32_t)(0x1UL << MXC_F_AFE_ADC_ZERO_STATUS_IE_TOR_IE_7_POS)) /**< STATUS_IE_TOR_IE_7 Mask */

/**@} end of group AFE_ADC_ZERO_STATUS_IE_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_0 AFE_ADC_ZERO_UC_0
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_0_REG_DATA_POS           0 /**< UC_0_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_0_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_0_REG_DATA_POS)) /**< UC_0_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_0_REG_ADDR_POS           8 /**< UC_0_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_0_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_0_REG_ADDR_POS)) /**< UC_0_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_0_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_1 AFE_ADC_ZERO_UC_1
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_1_REG_DATA_POS           0 /**< UC_1_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_1_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_1_REG_DATA_POS)) /**< UC_1_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_1_REG_ADDR_POS           8 /**< UC_1_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_1_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_1_REG_ADDR_POS)) /**< UC_1_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_2 AFE_ADC_ZERO_UC_2
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_2_REG_DATA_POS           0 /**< UC_2_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_2_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_2_REG_DATA_POS)) /**< UC_2_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_2_REG_ADDR_POS           8 /**< UC_2_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_2_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_2_REG_ADDR_POS)) /**< UC_2_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_2_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_3 AFE_ADC_ZERO_UC_3
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_3_REG_DATA_POS           0 /**< UC_3_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_3_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_3_REG_DATA_POS)) /**< UC_3_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_3_REG_ADDR_POS           8 /**< UC_3_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_3_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_3_REG_ADDR_POS)) /**< UC_3_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_3_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_4 AFE_ADC_ZERO_UC_4
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_4_REG_DATA_POS           0 /**< UC_4_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_4_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_4_REG_DATA_POS)) /**< UC_4_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_4_REG_ADDR_POS           8 /**< UC_4_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_4_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_4_REG_ADDR_POS)) /**< UC_4_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_4_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_5 AFE_ADC_ZERO_UC_5
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_5_REG_DATA_POS           0 /**< UC_5_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_5_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_5_REG_DATA_POS)) /**< UC_5_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_5_REG_ADDR_POS           8 /**< UC_5_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_5_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_5_REG_ADDR_POS)) /**< UC_5_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_5_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_6 AFE_ADC_ZERO_UC_6
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_6_REG_DATA_POS           0 /**< UC_6_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_6_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_6_REG_DATA_POS)) /**< UC_6_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_6_REG_ADDR_POS           8 /**< UC_6_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_6_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_6_REG_ADDR_POS)) /**< UC_6_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_6_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_7 AFE_ADC_ZERO_UC_7
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_7_REG_DATA_POS           0 /**< UC_7_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_7_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_7_REG_DATA_POS)) /**< UC_7_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_7_REG_ADDR_POS           8 /**< UC_7_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_7_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_7_REG_ADDR_POS)) /**< UC_7_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_7_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_8 AFE_ADC_ZERO_UC_8
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_8_REG_DATA_POS           0 /**< UC_8_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_8_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_8_REG_DATA_POS)) /**< UC_8_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_8_REG_ADDR_POS           8 /**< UC_8_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_8_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_8_REG_ADDR_POS)) /**< UC_8_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_8_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_9 AFE_ADC_ZERO_UC_9
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_9_REG_DATA_POS           0 /**< UC_9_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_9_REG_DATA               ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_9_REG_DATA_POS)) /**< UC_9_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_9_REG_ADDR_POS           8 /**< UC_9_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_9_REG_ADDR               ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_9_REG_ADDR_POS)) /**< UC_9_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_9_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_10 AFE_ADC_ZERO_UC_10
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_10_REG_DATA_POS          0 /**< UC_10_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_10_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_10_REG_DATA_POS)) /**< UC_10_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_10_REG_ADDR_POS          8 /**< UC_10_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_10_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_10_REG_ADDR_POS)) /**< UC_10_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_10_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_11 AFE_ADC_ZERO_UC_11
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_11_REG_DATA_POS          0 /**< UC_11_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_11_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_11_REG_DATA_POS)) /**< UC_11_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_11_REG_ADDR_POS          8 /**< UC_11_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_11_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_11_REG_ADDR_POS)) /**< UC_11_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_11_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_12 AFE_ADC_ZERO_UC_12
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_12_REG_DATA_POS          0 /**< UC_12_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_12_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_12_REG_DATA_POS)) /**< UC_12_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_12_REG_ADDR_POS          8 /**< UC_12_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_12_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_12_REG_ADDR_POS)) /**< UC_12_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_12_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_13 AFE_ADC_ZERO_UC_13
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_13_REG_DATA_POS          0 /**< UC_13_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_13_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_13_REG_DATA_POS)) /**< UC_13_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_13_REG_ADDR_POS          8 /**< UC_13_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_13_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_13_REG_ADDR_POS)) /**< UC_13_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_13_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_14 AFE_ADC_ZERO_UC_14
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_14_REG_DATA_POS          0 /**< UC_14_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_14_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_14_REG_DATA_POS)) /**< UC_14_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_14_REG_ADDR_POS          8 /**< UC_14_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_14_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_14_REG_ADDR_POS)) /**< UC_14_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_14_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_15 AFE_ADC_ZERO_UC_15
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_15_REG_DATA_POS          0 /**< UC_15_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_15_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_15_REG_DATA_POS)) /**< UC_15_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_15_REG_ADDR_POS          8 /**< UC_15_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_15_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_15_REG_ADDR_POS)) /**< UC_15_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_15_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_16 AFE_ADC_ZERO_UC_16
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_16_REG_DATA_POS          0 /**< UC_16_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_16_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_16_REG_DATA_POS)) /**< UC_16_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_16_REG_ADDR_POS          8 /**< UC_16_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_16_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_16_REG_ADDR_POS)) /**< UC_16_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_16_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_17 AFE_ADC_ZERO_UC_17
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_17_REG_DATA_POS          0 /**< UC_17_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_17_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_17_REG_DATA_POS)) /**< UC_17_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_17_REG_ADDR_POS          8 /**< UC_17_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_17_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_17_REG_ADDR_POS)) /**< UC_17_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_17_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_18 AFE_ADC_ZERO_UC_18
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_18_REG_DATA_POS          0 /**< UC_18_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_18_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_18_REG_DATA_POS)) /**< UC_18_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_18_REG_ADDR_POS          8 /**< UC_18_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_18_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_18_REG_ADDR_POS)) /**< UC_18_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_18_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_19 AFE_ADC_ZERO_UC_19
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_19_REG_DATA_POS          0 /**< UC_19_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_19_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_19_REG_DATA_POS)) /**< UC_19_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_19_REG_ADDR_POS          8 /**< UC_19_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_19_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_19_REG_ADDR_POS)) /**< UC_19_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_19_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_20 AFE_ADC_ZERO_UC_20
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_20_REG_DATA_POS          0 /**< UC_20_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_20_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_20_REG_DATA_POS)) /**< UC_20_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_20_REG_ADDR_POS          8 /**< UC_20_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_20_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_20_REG_ADDR_POS)) /**< UC_20_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_20_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_21 AFE_ADC_ZERO_UC_21
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_21_REG_DATA_POS          0 /**< UC_21_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_21_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_21_REG_DATA_POS)) /**< UC_21_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_21_REG_ADDR_POS          8 /**< UC_21_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_21_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_21_REG_ADDR_POS)) /**< UC_21_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_21_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_22 AFE_ADC_ZERO_UC_22
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_22_REG_DATA_POS          0 /**< UC_22_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_22_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_22_REG_DATA_POS)) /**< UC_22_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_22_REG_ADDR_POS          8 /**< UC_22_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_22_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_22_REG_ADDR_POS)) /**< UC_22_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_22_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_23 AFE_ADC_ZERO_UC_23
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_23_REG_DATA_POS          0 /**< UC_23_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_23_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_23_REG_DATA_POS)) /**< UC_23_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_23_REG_ADDR_POS          8 /**< UC_23_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_23_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_23_REG_ADDR_POS)) /**< UC_23_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_23_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_24 AFE_ADC_ZERO_UC_24
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_24_REG_DATA_POS          0 /**< UC_24_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_24_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_24_REG_DATA_POS)) /**< UC_24_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_24_REG_ADDR_POS          8 /**< UC_24_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_24_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_24_REG_ADDR_POS)) /**< UC_24_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_24_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_25 AFE_ADC_ZERO_UC_25
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_25_REG_DATA_POS          0 /**< UC_25_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_25_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_25_REG_DATA_POS)) /**< UC_25_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_25_REG_ADDR_POS          8 /**< UC_25_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_25_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_25_REG_ADDR_POS)) /**< UC_25_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_25_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_26 AFE_ADC_ZERO_UC_26
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_26_REG_DATA_POS          0 /**< UC_26_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_26_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_26_REG_DATA_POS)) /**< UC_26_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_26_REG_ADDR_POS          8 /**< UC_26_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_26_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_26_REG_ADDR_POS)) /**< UC_26_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_26_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_27 AFE_ADC_ZERO_UC_27
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_27_REG_DATA_POS          0 /**< UC_27_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_27_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_27_REG_DATA_POS)) /**< UC_27_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_27_REG_ADDR_POS          8 /**< UC_27_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_27_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_27_REG_ADDR_POS)) /**< UC_27_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_27_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_28 AFE_ADC_ZERO_UC_28
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_28_REG_DATA_POS          0 /**< UC_28_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_28_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_28_REG_DATA_POS)) /**< UC_28_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_28_REG_ADDR_POS          8 /**< UC_28_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_28_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_28_REG_ADDR_POS)) /**< UC_28_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_28_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_29 AFE_ADC_ZERO_UC_29
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_29_REG_DATA_POS          0 /**< UC_29_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_29_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_29_REG_DATA_POS)) /**< UC_29_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_29_REG_ADDR_POS          8 /**< UC_29_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_29_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_29_REG_ADDR_POS)) /**< UC_29_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_29_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_30 AFE_ADC_ZERO_UC_30
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_30_REG_DATA_POS          0 /**< UC_30_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_30_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_30_REG_DATA_POS)) /**< UC_30_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_30_REG_ADDR_POS          8 /**< UC_30_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_30_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_30_REG_ADDR_POS)) /**< UC_30_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_30_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_31 AFE_ADC_ZERO_UC_31
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_31_REG_DATA_POS          0 /**< UC_31_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_31_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_31_REG_DATA_POS)) /**< UC_31_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_31_REG_ADDR_POS          8 /**< UC_31_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_31_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_31_REG_ADDR_POS)) /**< UC_31_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_31_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_32 AFE_ADC_ZERO_UC_32
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_32_REG_DATA_POS          0 /**< UC_32_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_32_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_32_REG_DATA_POS)) /**< UC_32_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_32_REG_ADDR_POS          8 /**< UC_32_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_32_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_32_REG_ADDR_POS)) /**< UC_32_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_32_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_33 AFE_ADC_ZERO_UC_33
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_33_REG_DATA_POS          0 /**< UC_33_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_33_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_33_REG_DATA_POS)) /**< UC_33_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_33_REG_ADDR_POS          8 /**< UC_33_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_33_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_33_REG_ADDR_POS)) /**< UC_33_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_33_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_34 AFE_ADC_ZERO_UC_34
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_34_REG_DATA_POS          0 /**< UC_34_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_34_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_34_REG_DATA_POS)) /**< UC_34_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_34_REG_ADDR_POS          8 /**< UC_34_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_34_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_34_REG_ADDR_POS)) /**< UC_34_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_34_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_35 AFE_ADC_ZERO_UC_35
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_35_REG_DATA_POS          0 /**< UC_35_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_35_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_35_REG_DATA_POS)) /**< UC_35_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_35_REG_ADDR_POS          8 /**< UC_35_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_35_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_35_REG_ADDR_POS)) /**< UC_35_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_35_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_36 AFE_ADC_ZERO_UC_36
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_36_REG_DATA_POS          0 /**< UC_36_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_36_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_36_REG_DATA_POS)) /**< UC_36_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_36_REG_ADDR_POS          8 /**< UC_36_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_36_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_36_REG_ADDR_POS)) /**< UC_36_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_36_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_37 AFE_ADC_ZERO_UC_37
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_37_REG_DATA_POS          0 /**< UC_37_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_37_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_37_REG_DATA_POS)) /**< UC_37_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_37_REG_ADDR_POS          8 /**< UC_37_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_37_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_37_REG_ADDR_POS)) /**< UC_37_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_37_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_38 AFE_ADC_ZERO_UC_38
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_38_REG_DATA_POS          0 /**< UC_38_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_38_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_38_REG_DATA_POS)) /**< UC_38_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_38_REG_ADDR_POS          8 /**< UC_38_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_38_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_38_REG_ADDR_POS)) /**< UC_38_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_38_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_39 AFE_ADC_ZERO_UC_39
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_39_REG_DATA_POS          0 /**< UC_39_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_39_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_39_REG_DATA_POS)) /**< UC_39_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_39_REG_ADDR_POS          8 /**< UC_39_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_39_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_39_REG_ADDR_POS)) /**< UC_39_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_39_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_40 AFE_ADC_ZERO_UC_40
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_40_REG_DATA_POS          0 /**< UC_40_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_40_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_40_REG_DATA_POS)) /**< UC_40_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_40_REG_ADDR_POS          8 /**< UC_40_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_40_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_40_REG_ADDR_POS)) /**< UC_40_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_40_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_41 AFE_ADC_ZERO_UC_41
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_41_REG_DATA_POS          0 /**< UC_41_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_41_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_41_REG_DATA_POS)) /**< UC_41_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_41_REG_ADDR_POS          8 /**< UC_41_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_41_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_41_REG_ADDR_POS)) /**< UC_41_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_41_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_42 AFE_ADC_ZERO_UC_42
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_42_REG_DATA_POS          0 /**< UC_42_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_42_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_42_REG_DATA_POS)) /**< UC_42_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_42_REG_ADDR_POS          8 /**< UC_42_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_42_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_42_REG_ADDR_POS)) /**< UC_42_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_42_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_43 AFE_ADC_ZERO_UC_43
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_43_REG_DATA_POS          0 /**< UC_43_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_43_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_43_REG_DATA_POS)) /**< UC_43_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_43_REG_ADDR_POS          8 /**< UC_43_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_43_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_43_REG_ADDR_POS)) /**< UC_43_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_43_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_44 AFE_ADC_ZERO_UC_44
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_44_REG_DATA_POS          0 /**< UC_44_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_44_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_44_REG_DATA_POS)) /**< UC_44_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_44_REG_ADDR_POS          8 /**< UC_44_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_44_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_44_REG_ADDR_POS)) /**< UC_44_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_44_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_45 AFE_ADC_ZERO_UC_45
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_45_REG_DATA_POS          0 /**< UC_45_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_45_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_45_REG_DATA_POS)) /**< UC_45_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_45_REG_ADDR_POS          8 /**< UC_45_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_45_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_45_REG_ADDR_POS)) /**< UC_45_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_45_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_46 AFE_ADC_ZERO_UC_46
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_46_REG_DATA_POS          0 /**< UC_46_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_46_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_46_REG_DATA_POS)) /**< UC_46_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_46_REG_ADDR_POS          8 /**< UC_46_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_46_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_46_REG_ADDR_POS)) /**< UC_46_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_46_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_47 AFE_ADC_ZERO_UC_47
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_47_REG_DATA_POS          0 /**< UC_47_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_47_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_47_REG_DATA_POS)) /**< UC_47_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_47_REG_ADDR_POS          8 /**< UC_47_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_47_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_47_REG_ADDR_POS)) /**< UC_47_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_47_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_48 AFE_ADC_ZERO_UC_48
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_48_REG_DATA_POS          0 /**< UC_48_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_48_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_48_REG_DATA_POS)) /**< UC_48_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_48_REG_ADDR_POS          8 /**< UC_48_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_48_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_48_REG_ADDR_POS)) /**< UC_48_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_48_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_49 AFE_ADC_ZERO_UC_49
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_49_REG_DATA_POS          0 /**< UC_49_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_49_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_49_REG_DATA_POS)) /**< UC_49_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_49_REG_ADDR_POS          8 /**< UC_49_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_49_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_49_REG_ADDR_POS)) /**< UC_49_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_49_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_50 AFE_ADC_ZERO_UC_50
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_50_REG_DATA_POS          0 /**< UC_50_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_50_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_50_REG_DATA_POS)) /**< UC_50_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_50_REG_ADDR_POS          8 /**< UC_50_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_50_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_50_REG_ADDR_POS)) /**< UC_50_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_50_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_51 AFE_ADC_ZERO_UC_51
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_51_REG_DATA_POS          0 /**< UC_51_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_51_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_51_REG_DATA_POS)) /**< UC_51_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_51_REG_ADDR_POS          8 /**< UC_51_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_51_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_51_REG_ADDR_POS)) /**< UC_51_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_51_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UC_52 AFE_ADC_ZERO_UC_52
 * @brief    Sequencer Register
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UC_52_REG_DATA_POS          0 /**< UC_52_REG_DATA Position */
#define MXC_F_AFE_ADC_ZERO_UC_52_REG_DATA              ((uint16_t)(0xFFUL << MXC_F_AFE_ADC_ZERO_UC_52_REG_DATA_POS)) /**< UC_52_REG_DATA Mask */

#define MXC_F_AFE_ADC_ZERO_UC_52_REG_ADDR_POS          8 /**< UC_52_REG_ADDR Position */
#define MXC_F_AFE_ADC_ZERO_UC_52_REG_ADDR              ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UC_52_REG_ADDR_POS)) /**< UC_52_REG_ADDR Mask */

/**@} end of group AFE_ADC_ZERO_UC_52_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_UCADDR AFE_ADC_ZERO_UCADDR
 * @brief    Address of currently executing sequence command
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_UCADDR_UCADDR_POS           0 /**< UCADDR_UCADDR Position */
#define MXC_F_AFE_ADC_ZERO_UCADDR_UCADDR               ((uint8_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_UCADDR_UCADDR_POS)) /**< UCADDR_UCADDR Mask */

/**@} end of group AFE_ADC_ZERO_UCADDR_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_FT_PWORD AFE_ADC_ZERO_FT_PWORD
 * @brief    FT Password
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_POS       0 /**< FT_PWORD_FT_PWORD Position */
#define MXC_F_AFE_ADC_ZERO_FT_PWORD_FT_PWORD           ((uint8_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_POS)) /**< FT_PWORD_FT_PWORD Mask */
#define MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_1   ((uint8_t)0x48UL) /**< FT_PWORD_FT_PWORD_PWORD_1 Value */
#define MXC_S_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_1   (MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_1 << MXC_F_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_POS) /**< FT_PWORD_FT_PWORD_PWORD_1 Setting */
#define MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_2   ((uint8_t)0x7BUL) /**< FT_PWORD_FT_PWORD_PWORD_2 Value */
#define MXC_S_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_2   (MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_2 << MXC_F_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_POS) /**< FT_PWORD_FT_PWORD_PWORD_2 Setting */

/**@} end of group AFE_ADC_ZERO_FT_PWORD_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_ADC_TRIM0 AFE_ADC_ZERO_ADC_TRIM0
 * @brief    Various ADC Analog Trims
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_VGB_TRIM_POS      0 /**< ADC_TRIM0_VGB_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_VGB_TRIM          ((uint32_t)(0x3FUL << MXC_F_AFE_ADC_ZERO_ADC_TRIM0_VGB_TRIM_POS)) /**< ADC_TRIM0_VGB_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_IP_TRIM_POS       6 /**< ADC_TRIM0_IP_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_IP_TRIM           ((uint32_t)(0x1FUL << MXC_F_AFE_ADC_ZERO_ADC_TRIM0_IP_TRIM_POS)) /**< ADC_TRIM0_IP_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_IB_TRIM_POS       11 /**< ADC_TRIM0_IB_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_IB_TRIM           ((uint32_t)(0x3FUL << MXC_F_AFE_ADC_ZERO_ADC_TRIM0_IB_TRIM_POS)) /**< ADC_TRIM0_IB_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_CLKSKEW_POS       17 /**< ADC_TRIM0_CLKSKEW Position */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM0_CLKSKEW           ((uint32_t)(0x3UL << MXC_F_AFE_ADC_ZERO_ADC_TRIM0_CLKSKEW_POS)) /**< ADC_TRIM0_CLKSKEW Mask */

/**@} end of group AFE_ADC_ZERO_ADC_TRIM0_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_ADC_TRIM1 AFE_ADC_ZERO_ADC_TRIM1
 * @brief    Various ADC Analog Trims
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM1_IDAC0_TRIM_POS    0 /**< ADC_TRIM1_IDAC0_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM1_IDAC0_TRIM        ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_ADC_TRIM1_IDAC0_TRIM_POS)) /**< ADC_TRIM1_IDAC0_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ADC_TRIM1_IDAC1_TRIM_POS    7 /**< ADC_TRIM1_IDAC1_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ADC_TRIM1_IDAC1_TRIM        ((uint16_t)(0x7FUL << MXC_F_AFE_ADC_ZERO_ADC_TRIM1_IDAC1_TRIM_POS)) /**< ADC_TRIM1_IDAC1_TRIM Mask */

/**@} end of group AFE_ADC_ZERO_ADC_TRIM1_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_ANA_TRIM AFE_ADC_ZERO_ANA_TRIM
 * @brief    Various Analog Trims
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_LDO_TRIM_POS       0 /**< ANA_TRIM_LDO_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_LDO_TRIM           ((uint16_t)(0xFUL << MXC_F_AFE_ADC_ZERO_ANA_TRIM_LDO_TRIM_POS)) /**< ANA_TRIM_LDO_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_OSC_TRIM_POS       4 /**< ANA_TRIM_OSC_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_OSC_TRIM           ((uint16_t)(0x3FUL << MXC_F_AFE_ADC_ZERO_ANA_TRIM_OSC_TRIM_POS)) /**< ANA_TRIM_OSC_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_OSC_FSEL_POS       10 /**< ANA_TRIM_OSC_FSEL Position */
#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_OSC_FSEL           ((uint16_t)(0x3UL << MXC_F_AFE_ADC_ZERO_ANA_TRIM_OSC_FSEL_POS)) /**< ANA_TRIM_OSC_FSEL Mask */

#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_PKG_TRIM_POS       12 /**< ANA_TRIM_PKG_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_PKG_TRIM           ((uint16_t)(0x3UL << MXC_F_AFE_ADC_ZERO_ANA_TRIM_PKG_TRIM_POS)) /**< ANA_TRIM_PKG_TRIM Mask */

#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_RES_TRIM_POS       14 /**< ANA_TRIM_RES_TRIM Position */
#define MXC_F_AFE_ADC_ZERO_ANA_TRIM_RES_TRIM           ((uint16_t)(0x1UL << MXC_F_AFE_ADC_ZERO_ANA_TRIM_RES_TRIM_POS)) /**< ANA_TRIM_RES_TRIM Mask */

/**@} end of group AFE_ADC_ZERO_ANA_TRIM_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_SYS_CTRL AFE_ADC_ZERO_SYS_CTRL
 * @brief    System Calibration Selection
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_POS    0 /**< SYS_CTRL_ANA_SRC_SEL Position */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL        ((uint8_t)(0x3UL << MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_POS)) /**< SYS_CTRL_ANA_SRC_SEL Mask */
#define MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC0_BANK ((uint8_t)0x0UL) /**< SYS_CTRL_ANA_SRC_SEL_ADC0_BANK Value */
#define MXC_S_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC0_BANK (MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC0_BANK << MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_POS) /**< SYS_CTRL_ANA_SRC_SEL_ADC0_BANK Setting */
#define MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC1_BANK ((uint8_t)0x1UL) /**< SYS_CTRL_ANA_SRC_SEL_ADC1_BANK Value */
#define MXC_S_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC1_BANK (MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC1_BANK << MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_POS) /**< SYS_CTRL_ANA_SRC_SEL_ADC1_BANK Setting */
#define MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_DAC12_BANK ((uint8_t)0x2UL) /**< SYS_CTRL_ANA_SRC_SEL_DAC12_BANK Value */
#define MXC_S_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_DAC12_BANK (MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_DAC12_BANK << MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_POS) /**< SYS_CTRL_ANA_SRC_SEL_DAC12_BANK Setting */
#define MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_HART_BANK ((uint8_t)0x3UL) /**< SYS_CTRL_ANA_SRC_SEL_HART_BANK Value */
#define MXC_S_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_HART_BANK (MXC_V_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_HART_BANK << MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_POS) /**< SYS_CTRL_ANA_SRC_SEL_HART_BANK Setting */

#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC5_POS           2 /**< SYS_CTRL_CRC5 Position */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC5               ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC5_POS)) /**< SYS_CTRL_CRC5 Mask */

#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN_POS        4 /**< SYS_CTRL_HART_EN Position */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN            ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN_POS)) /**< SYS_CTRL_HART_EN Mask */

#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_SPI_ABORT_DIS_POS  5 /**< SYS_CTRL_SPI_ABORT_DIS Position */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_SPI_ABORT_DIS      ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_SYS_CTRL_SPI_ABORT_DIS_POS)) /**< SYS_CTRL_SPI_ABORT_DIS Mask */

#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG_POS       6 /**< SYS_CTRL_POR_FLAG Position */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG           ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG_POS)) /**< SYS_CTRL_POR_FLAG Mask */

#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC_INV_POS        7 /**< SYS_CTRL_CRC_INV Position */
#define MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC_INV            ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC_INV_POS)) /**< SYS_CTRL_CRC_INV Mask */

/**@} end of group AFE_ADC_ZERO_SYS_CTRL_Register */

/**
 * @ingroup  afe_adc_zero_registers
 * @defgroup AFE_ADC_ZERO_TS_CTRL AFE_ADC_ZERO_TS_CTRL
 * @brief    Temperature Sensor Control
 * @{
 */
#define MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_EN_POS           0 /**< TS_CTRL_TS_EN Position */
#define MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_EN               ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_EN_POS)) /**< TS_CTRL_TS_EN Mask */

#define MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_CONV_EN_POS      1 /**< TS_CTRL_TS_CONV_EN Position */
#define MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_CONV_EN          ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_CONV_EN_POS)) /**< TS_CTRL_TS_CONV_EN Mask */

#define MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_INTG_RDY_POS     2 /**< TS_CTRL_TS_INTG_RDY Position */
#define MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_INTG_RDY         ((uint8_t)(0x1UL << MXC_F_AFE_ADC_ZERO_TS_CTRL_TS_INTG_RDY_POS)) /**< TS_CTRL_TS_INTG_RDY Mask */

/**@} end of group AFE_ADC_ZERO_TS_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AFE_ADC_ZERO_REGS_H_
