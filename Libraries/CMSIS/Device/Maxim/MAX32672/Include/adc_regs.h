/**
 * @file    adc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the ADC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup adc_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_ADC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_ADC_REGS_H_

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
 * @ingroup     adc
 * @defgroup    adc_registers ADC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the ADC Peripheral Module.
 * @details     Inter-Integrated Circuit.
 */

/**
 * @ingroup adc_registers
 * Structure type to access the ADC Registers.
 */
typedef struct {
    __IO uint32_t ctrl0;                /**< <tt>\b 0x00:</tt> ADC CTRL0 Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x04:</tt> ADC CTRL1 Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x08:</tt> ADC CLKCTRL Register */
    __IO uint32_t sampclkctrl;          /**< <tt>\b 0x0C:</tt> ADC SAMPCLKCTRL Register */
    __IO uint32_t chsel0;               /**< <tt>\b 0x10:</tt> ADC CHSEL0 Register */
    __IO uint32_t chsel1;               /**< <tt>\b 0x14:</tt> ADC CHSEL1 Register */
    __IO uint32_t chsel2;               /**< <tt>\b 0x18:</tt> ADC CHSEL2 Register */
    __IO uint32_t chsel3;               /**< <tt>\b 0x1C:</tt> ADC CHSEL3 Register */
    __R  uint32_t rsv_0x20_0x2f[4];
    __IO uint32_t restart;              /**< <tt>\b 0x30:</tt> ADC RESTART Register */
    __R  uint32_t rsv_0x34_0x3b[2];
    __IO uint32_t datafmt;              /**< <tt>\b 0x3C:</tt> ADC DATAFMT Register */
    __IO uint32_t fifodmactrl;          /**< <tt>\b 0x40:</tt> ADC FIFODMACTRL Register */
    __IO uint32_t data;                 /**< <tt>\b 0x44:</tt> ADC DATA Register */
    __IO uint32_t status;               /**< <tt>\b 0x48:</tt> ADC STATUS Register */
    __IO uint32_t chstatus;             /**< <tt>\b 0x4C:</tt> ADC CHSTATUS Register */
    __IO uint32_t inten;                /**< <tt>\b 0x50:</tt> ADC INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x54:</tt> ADC INTFL Register */
    __R  uint32_t rsv_0x58_0x5f[2];
    __IO uint32_t sfraddroffset;        /**< <tt>\b 0x60:</tt> ADC SFRADDROFFSET Register */
    __IO uint32_t sfraddr;              /**< <tt>\b 0x64:</tt> ADC SFRADDR Register */
    __IO uint32_t sfrwrdata;            /**< <tt>\b 0x68:</tt> ADC SFRWRDATA Register */
    __IO uint32_t sfrrddata;            /**< <tt>\b 0x6C:</tt> ADC SFRRDDATA Register */
    __IO uint32_t sfrstatus;            /**< <tt>\b 0x70:</tt> ADC SFRSTATUS Register */
} mxc_adc_regs_t;

/* Register offsets for module ADC */
/**
 * @ingroup    adc_registers
 * @defgroup   ADC_Register_Offsets Register Offsets
 * @brief      ADC Peripheral Register Offsets from the ADC Base Peripheral Address.
 * @{
 */
#define MXC_R_ADC_CTRL0                    ((uint32_t)0x00000000UL) /**< Offset from ADC Base Address: <tt> 0x0000</tt> */
#define MXC_R_ADC_CTRL1                    ((uint32_t)0x00000004UL) /**< Offset from ADC Base Address: <tt> 0x0004</tt> */
#define MXC_R_ADC_CLKCTRL                  ((uint32_t)0x00000008UL) /**< Offset from ADC Base Address: <tt> 0x0008</tt> */
#define MXC_R_ADC_SAMPCLKCTRL              ((uint32_t)0x0000000CUL) /**< Offset from ADC Base Address: <tt> 0x000C</tt> */
#define MXC_R_ADC_CHSEL0                   ((uint32_t)0x00000010UL) /**< Offset from ADC Base Address: <tt> 0x0010</tt> */
#define MXC_R_ADC_CHSEL1                   ((uint32_t)0x00000014UL) /**< Offset from ADC Base Address: <tt> 0x0014</tt> */
#define MXC_R_ADC_CHSEL2                   ((uint32_t)0x00000018UL) /**< Offset from ADC Base Address: <tt> 0x0018</tt> */
#define MXC_R_ADC_CHSEL3                   ((uint32_t)0x0000001CUL) /**< Offset from ADC Base Address: <tt> 0x001C</tt> */
#define MXC_R_ADC_RESTART                  ((uint32_t)0x00000030UL) /**< Offset from ADC Base Address: <tt> 0x0030</tt> */
#define MXC_R_ADC_DATAFMT                  ((uint32_t)0x0000003CUL) /**< Offset from ADC Base Address: <tt> 0x003C</tt> */
#define MXC_R_ADC_FIFODMACTRL              ((uint32_t)0x00000040UL) /**< Offset from ADC Base Address: <tt> 0x0040</tt> */
#define MXC_R_ADC_DATA                     ((uint32_t)0x00000044UL) /**< Offset from ADC Base Address: <tt> 0x0044</tt> */
#define MXC_R_ADC_STATUS                   ((uint32_t)0x00000048UL) /**< Offset from ADC Base Address: <tt> 0x0048</tt> */
#define MXC_R_ADC_CHSTATUS                 ((uint32_t)0x0000004CUL) /**< Offset from ADC Base Address: <tt> 0x004C</tt> */
#define MXC_R_ADC_INTEN                    ((uint32_t)0x00000050UL) /**< Offset from ADC Base Address: <tt> 0x0050</tt> */
#define MXC_R_ADC_INTFL                    ((uint32_t)0x00000054UL) /**< Offset from ADC Base Address: <tt> 0x0054</tt> */
#define MXC_R_ADC_SFRADDROFFSET            ((uint32_t)0x00000060UL) /**< Offset from ADC Base Address: <tt> 0x0060</tt> */
#define MXC_R_ADC_SFRADDR                  ((uint32_t)0x00000064UL) /**< Offset from ADC Base Address: <tt> 0x0064</tt> */
#define MXC_R_ADC_SFRWRDATA                ((uint32_t)0x00000068UL) /**< Offset from ADC Base Address: <tt> 0x0068</tt> */
#define MXC_R_ADC_SFRRDDATA                ((uint32_t)0x0000006CUL) /**< Offset from ADC Base Address: <tt> 0x006C</tt> */
#define MXC_R_ADC_SFRSTATUS                ((uint32_t)0x00000070UL) /**< Offset from ADC Base Address: <tt> 0x0070</tt> */
/**@} end of group adc_registers */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CTRL0 ADC_CTRL0
 * @brief    Control Register 0.
 * @{
 */
#define MXC_F_ADC_CTRL0_ADC_EN_POS                     0 /**< CTRL0_ADC_EN Position */
#define MXC_F_ADC_CTRL0_ADC_EN                         ((uint32_t)(0x1UL << MXC_F_ADC_CTRL0_ADC_EN_POS)) /**< CTRL0_ADC_EN Mask */

#define MXC_F_ADC_CTRL0_BIAS_EN_POS                    1 /**< CTRL0_BIAS_EN Position */
#define MXC_F_ADC_CTRL0_BIAS_EN                        ((uint32_t)(0x1UL << MXC_F_ADC_CTRL0_BIAS_EN_POS)) /**< CTRL0_BIAS_EN Mask */

#define MXC_F_ADC_CTRL0_SKIP_CAL_POS                   2 /**< CTRL0_SKIP_CAL Position */
#define MXC_F_ADC_CTRL0_SKIP_CAL                       ((uint32_t)(0x1UL << MXC_F_ADC_CTRL0_SKIP_CAL_POS)) /**< CTRL0_SKIP_CAL Mask */

#define MXC_F_ADC_CTRL0_CHOP_FORCE_POS                 3 /**< CTRL0_CHOP_FORCE Position */
#define MXC_F_ADC_CTRL0_CHOP_FORCE                     ((uint32_t)(0x1UL << MXC_F_ADC_CTRL0_CHOP_FORCE_POS)) /**< CTRL0_CHOP_FORCE Mask */

#define MXC_F_ADC_CTRL0_RESETB_POS                     4 /**< CTRL0_RESETB Position */
#define MXC_F_ADC_CTRL0_RESETB                         ((uint32_t)(0x1UL << MXC_F_ADC_CTRL0_RESETB_POS)) /**< CTRL0_RESETB Mask */

/**@} end of group ADC_CTRL0_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CTRL1 ADC_CTRL1
 * @brief    Control Register 1.
 * @{
 */
#define MXC_F_ADC_CTRL1_START_POS                      0 /**< CTRL1_START Position */
#define MXC_F_ADC_CTRL1_START                          ((uint32_t)(0x1UL << MXC_F_ADC_CTRL1_START_POS)) /**< CTRL1_START Mask */

#define MXC_F_ADC_CTRL1_TRIG_MODE_POS                  1 /**< CTRL1_TRIG_MODE Position */
#define MXC_F_ADC_CTRL1_TRIG_MODE                      ((uint32_t)(0x1UL << MXC_F_ADC_CTRL1_TRIG_MODE_POS)) /**< CTRL1_TRIG_MODE Mask */

#define MXC_F_ADC_CTRL1_CNV_MODE_POS                   2 /**< CTRL1_CNV_MODE Position */
#define MXC_F_ADC_CTRL1_CNV_MODE                       ((uint32_t)(0x1UL << MXC_F_ADC_CTRL1_CNV_MODE_POS)) /**< CTRL1_CNV_MODE Mask */

#define MXC_F_ADC_CTRL1_SAMP_CK_OFF_POS                3 /**< CTRL1_SAMP_CK_OFF Position */
#define MXC_F_ADC_CTRL1_SAMP_CK_OFF                    ((uint32_t)(0x1UL << MXC_F_ADC_CTRL1_SAMP_CK_OFF_POS)) /**< CTRL1_SAMP_CK_OFF Mask */

#define MXC_F_ADC_CTRL1_TRIG_SEL_POS                   4 /**< CTRL1_TRIG_SEL Position */
#define MXC_F_ADC_CTRL1_TRIG_SEL                       ((uint32_t)(0x7UL << MXC_F_ADC_CTRL1_TRIG_SEL_POS)) /**< CTRL1_TRIG_SEL Mask */

#define MXC_F_ADC_CTRL1_TS_SEL_POS                     7 /**< CTRL1_TS_SEL Position */
#define MXC_F_ADC_CTRL1_TS_SEL                         ((uint32_t)(0x1UL << MXC_F_ADC_CTRL1_TS_SEL_POS)) /**< CTRL1_TS_SEL Mask */

#define MXC_F_ADC_CTRL1_AVG_POS                        8 /**< CTRL1_AVG Position */
#define MXC_F_ADC_CTRL1_AVG                            ((uint32_t)(0x7UL << MXC_F_ADC_CTRL1_AVG_POS)) /**< CTRL1_AVG Mask */
#define MXC_V_ADC_CTRL1_AVG_AVG1                       ((uint32_t)0x0UL) /**< CTRL1_AVG_AVG1 Value */
#define MXC_S_ADC_CTRL1_AVG_AVG1                       (MXC_V_ADC_CTRL1_AVG_AVG1 << MXC_F_ADC_CTRL1_AVG_POS) /**< CTRL1_AVG_AVG1 Setting */
#define MXC_V_ADC_CTRL1_AVG_AVG2                       ((uint32_t)0x1UL) /**< CTRL1_AVG_AVG2 Value */
#define MXC_S_ADC_CTRL1_AVG_AVG2                       (MXC_V_ADC_CTRL1_AVG_AVG2 << MXC_F_ADC_CTRL1_AVG_POS) /**< CTRL1_AVG_AVG2 Setting */
#define MXC_V_ADC_CTRL1_AVG_AVG4                       ((uint32_t)0x2UL) /**< CTRL1_AVG_AVG4 Value */
#define MXC_S_ADC_CTRL1_AVG_AVG4                       (MXC_V_ADC_CTRL1_AVG_AVG4 << MXC_F_ADC_CTRL1_AVG_POS) /**< CTRL1_AVG_AVG4 Setting */
#define MXC_V_ADC_CTRL1_AVG_AVG8                       ((uint32_t)0x3UL) /**< CTRL1_AVG_AVG8 Value */
#define MXC_S_ADC_CTRL1_AVG_AVG8                       (MXC_V_ADC_CTRL1_AVG_AVG8 << MXC_F_ADC_CTRL1_AVG_POS) /**< CTRL1_AVG_AVG8 Setting */
#define MXC_V_ADC_CTRL1_AVG_AVG16                      ((uint32_t)0x4UL) /**< CTRL1_AVG_AVG16 Value */
#define MXC_S_ADC_CTRL1_AVG_AVG16                      (MXC_V_ADC_CTRL1_AVG_AVG16 << MXC_F_ADC_CTRL1_AVG_POS) /**< CTRL1_AVG_AVG16 Setting */
#define MXC_V_ADC_CTRL1_AVG_AVG32                      ((uint32_t)0x5UL) /**< CTRL1_AVG_AVG32 Value */
#define MXC_S_ADC_CTRL1_AVG_AVG32                      (MXC_V_ADC_CTRL1_AVG_AVG32 << MXC_F_ADC_CTRL1_AVG_POS) /**< CTRL1_AVG_AVG32 Setting */

#define MXC_F_ADC_CTRL1_NUM_SLOTS_POS                  16 /**< CTRL1_NUM_SLOTS Position */
#define MXC_F_ADC_CTRL1_NUM_SLOTS                      ((uint32_t)(0x1FUL << MXC_F_ADC_CTRL1_NUM_SLOTS_POS)) /**< CTRL1_NUM_SLOTS Mask */

/**@} end of group ADC_CTRL1_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CLKCTRL ADC_CLKCTRL
 * @brief    Clock Control Register.
 * @{
 */
#define MXC_F_ADC_CLKCTRL_CLKSEL_POS                   0 /**< CLKCTRL_CLKSEL Position */
#define MXC_F_ADC_CLKCTRL_CLKSEL                       ((uint32_t)(0x3UL << MXC_F_ADC_CLKCTRL_CLKSEL_POS)) /**< CLKCTRL_CLKSEL Mask */
#define MXC_V_ADC_CLKCTRL_CLKSEL_HCLK                  ((uint32_t)0x0UL) /**< CLKCTRL_CLKSEL_HCLK Value */
#define MXC_S_ADC_CLKCTRL_CLKSEL_HCLK                  (MXC_V_ADC_CLKCTRL_CLKSEL_HCLK << MXC_F_ADC_CLKCTRL_CLKSEL_POS) /**< CLKCTRL_CLKSEL_HCLK Setting */
#define MXC_V_ADC_CLKCTRL_CLKSEL_CLK_ADC0              ((uint32_t)0x1UL) /**< CLKCTRL_CLKSEL_CLK_ADC0 Value */
#define MXC_S_ADC_CLKCTRL_CLKSEL_CLK_ADC0              (MXC_V_ADC_CLKCTRL_CLKSEL_CLK_ADC0 << MXC_F_ADC_CLKCTRL_CLKSEL_POS) /**< CLKCTRL_CLKSEL_CLK_ADC0 Setting */
#define MXC_V_ADC_CLKCTRL_CLKSEL_CLK_ADC1              ((uint32_t)0x2UL) /**< CLKCTRL_CLKSEL_CLK_ADC1 Value */
#define MXC_S_ADC_CLKCTRL_CLKSEL_CLK_ADC1              (MXC_V_ADC_CLKCTRL_CLKSEL_CLK_ADC1 << MXC_F_ADC_CLKCTRL_CLKSEL_POS) /**< CLKCTRL_CLKSEL_CLK_ADC1 Setting */
#define MXC_V_ADC_CLKCTRL_CLKSEL_CLK_ADC2              ((uint32_t)0x3UL) /**< CLKCTRL_CLKSEL_CLK_ADC2 Value */
#define MXC_S_ADC_CLKCTRL_CLKSEL_CLK_ADC2              (MXC_V_ADC_CLKCTRL_CLKSEL_CLK_ADC2 << MXC_F_ADC_CLKCTRL_CLKSEL_POS) /**< CLKCTRL_CLKSEL_CLK_ADC2 Setting */

#define MXC_F_ADC_CLKCTRL_CLKDIV_POS                   4 /**< CLKCTRL_CLKDIV Position */
#define MXC_F_ADC_CLKCTRL_CLKDIV                       ((uint32_t)(0x7UL << MXC_F_ADC_CLKCTRL_CLKDIV_POS)) /**< CLKCTRL_CLKDIV Mask */
#define MXC_V_ADC_CLKCTRL_CLKDIV_DIV2                  ((uint32_t)0x0UL) /**< CLKCTRL_CLKDIV_DIV2 Value */
#define MXC_S_ADC_CLKCTRL_CLKDIV_DIV2                  (MXC_V_ADC_CLKCTRL_CLKDIV_DIV2 << MXC_F_ADC_CLKCTRL_CLKDIV_POS) /**< CLKCTRL_CLKDIV_DIV2 Setting */
#define MXC_V_ADC_CLKCTRL_CLKDIV_DIV4                  ((uint32_t)0x1UL) /**< CLKCTRL_CLKDIV_DIV4 Value */
#define MXC_S_ADC_CLKCTRL_CLKDIV_DIV4                  (MXC_V_ADC_CLKCTRL_CLKDIV_DIV4 << MXC_F_ADC_CLKCTRL_CLKDIV_POS) /**< CLKCTRL_CLKDIV_DIV4 Setting */
#define MXC_V_ADC_CLKCTRL_CLKDIV_DIV8                  ((uint32_t)0x2UL) /**< CLKCTRL_CLKDIV_DIV8 Value */
#define MXC_S_ADC_CLKCTRL_CLKDIV_DIV8                  (MXC_V_ADC_CLKCTRL_CLKDIV_DIV8 << MXC_F_ADC_CLKCTRL_CLKDIV_POS) /**< CLKCTRL_CLKDIV_DIV8 Setting */
#define MXC_V_ADC_CLKCTRL_CLKDIV_DIV16                 ((uint32_t)0x3UL) /**< CLKCTRL_CLKDIV_DIV16 Value */
#define MXC_S_ADC_CLKCTRL_CLKDIV_DIV16                 (MXC_V_ADC_CLKCTRL_CLKDIV_DIV16 << MXC_F_ADC_CLKCTRL_CLKDIV_POS) /**< CLKCTRL_CLKDIV_DIV16 Setting */
#define MXC_V_ADC_CLKCTRL_CLKDIV_DIV1                  ((uint32_t)0x4UL) /**< CLKCTRL_CLKDIV_DIV1 Value */
#define MXC_S_ADC_CLKCTRL_CLKDIV_DIV1                  (MXC_V_ADC_CLKCTRL_CLKDIV_DIV1 << MXC_F_ADC_CLKCTRL_CLKDIV_POS) /**< CLKCTRL_CLKDIV_DIV1 Setting */

/**@} end of group ADC_CLKCTRL_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_SAMPCLKCTRL ADC_SAMPCLKCTRL
 * @brief    Sample Clock Control Register.
 * @{
 */
#define MXC_F_ADC_SAMPCLKCTRL_TRACK_CNT_POS            0 /**< SAMPCLKCTRL_TRACK_CNT Position */
#define MXC_F_ADC_SAMPCLKCTRL_TRACK_CNT                ((uint32_t)(0xFFUL << MXC_F_ADC_SAMPCLKCTRL_TRACK_CNT_POS)) /**< SAMPCLKCTRL_TRACK_CNT Mask */

#define MXC_F_ADC_SAMPCLKCTRL_IDLE_CNT_POS             16 /**< SAMPCLKCTRL_IDLE_CNT Position */
#define MXC_F_ADC_SAMPCLKCTRL_IDLE_CNT                 ((uint32_t)(0xFFFFUL << MXC_F_ADC_SAMPCLKCTRL_IDLE_CNT_POS)) /**< SAMPCLKCTRL_IDLE_CNT Mask */

/**@} end of group ADC_SAMPCLKCTRL_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CHSEL0 ADC_CHSEL0
 * @brief    Channel Select Register 0.
 * @{
 */
#define MXC_F_ADC_CHSEL0_SLOT0_ID_POS                  0 /**< CHSEL0_SLOT0_ID Position */
#define MXC_F_ADC_CHSEL0_SLOT0_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL0_SLOT0_ID_POS)) /**< CHSEL0_SLOT0_ID Mask */

#define MXC_F_ADC_CHSEL0_SLOT1_ID_POS                  8 /**< CHSEL0_SLOT1_ID Position */
#define MXC_F_ADC_CHSEL0_SLOT1_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL0_SLOT1_ID_POS)) /**< CHSEL0_SLOT1_ID Mask */

#define MXC_F_ADC_CHSEL0_SLOT2_ID_POS                  16 /**< CHSEL0_SLOT2_ID Position */
#define MXC_F_ADC_CHSEL0_SLOT2_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL0_SLOT2_ID_POS)) /**< CHSEL0_SLOT2_ID Mask */

#define MXC_F_ADC_CHSEL0_SLOT3_ID_POS                  24 /**< CHSEL0_SLOT3_ID Position */
#define MXC_F_ADC_CHSEL0_SLOT3_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL0_SLOT3_ID_POS)) /**< CHSEL0_SLOT3_ID Mask */

/**@} end of group ADC_CHSEL0_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CHSEL1 ADC_CHSEL1
 * @brief    Channel Select Register 1.
 * @{
 */
#define MXC_F_ADC_CHSEL1_SLOT4_ID_POS                  0 /**< CHSEL1_SLOT4_ID Position */
#define MXC_F_ADC_CHSEL1_SLOT4_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL1_SLOT4_ID_POS)) /**< CHSEL1_SLOT4_ID Mask */

#define MXC_F_ADC_CHSEL1_SLOT5_ID_POS                  8 /**< CHSEL1_SLOT5_ID Position */
#define MXC_F_ADC_CHSEL1_SLOT5_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL1_SLOT5_ID_POS)) /**< CHSEL1_SLOT5_ID Mask */

#define MXC_F_ADC_CHSEL1_SLOT6_ID_POS                  16 /**< CHSEL1_SLOT6_ID Position */
#define MXC_F_ADC_CHSEL1_SLOT6_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL1_SLOT6_ID_POS)) /**< CHSEL1_SLOT6_ID Mask */

#define MXC_F_ADC_CHSEL1_SLOT7_ID_POS                  24 /**< CHSEL1_SLOT7_ID Position */
#define MXC_F_ADC_CHSEL1_SLOT7_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL1_SLOT7_ID_POS)) /**< CHSEL1_SLOT7_ID Mask */

/**@} end of group ADC_CHSEL1_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CHSEL2 ADC_CHSEL2
 * @brief    Channel Select Register 2.
 * @{
 */
#define MXC_F_ADC_CHSEL2_SLOT8_ID_POS                  0 /**< CHSEL2_SLOT8_ID Position */
#define MXC_F_ADC_CHSEL2_SLOT8_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL2_SLOT8_ID_POS)) /**< CHSEL2_SLOT8_ID Mask */

#define MXC_F_ADC_CHSEL2_SLOT9_ID_POS                  8 /**< CHSEL2_SLOT9_ID Position */
#define MXC_F_ADC_CHSEL2_SLOT9_ID                      ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL2_SLOT9_ID_POS)) /**< CHSEL2_SLOT9_ID Mask */

#define MXC_F_ADC_CHSEL2_SLOT10_ID_POS                 16 /**< CHSEL2_SLOT10_ID Position */
#define MXC_F_ADC_CHSEL2_SLOT10_ID                     ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL2_SLOT10_ID_POS)) /**< CHSEL2_SLOT10_ID Mask */

#define MXC_F_ADC_CHSEL2_SLOT11_ID_POS                 24 /**< CHSEL2_SLOT11_ID Position */
#define MXC_F_ADC_CHSEL2_SLOT11_ID                     ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL2_SLOT11_ID_POS)) /**< CHSEL2_SLOT11_ID Mask */

/**@} end of group ADC_CHSEL2_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CHSEL3 ADC_CHSEL3
 * @brief    Channel Select Register 3.
 * @{
 */
#define MXC_F_ADC_CHSEL3_SLOT12_ID_POS                 0 /**< CHSEL3_SLOT12_ID Position */
#define MXC_F_ADC_CHSEL3_SLOT12_ID                     ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL3_SLOT12_ID_POS)) /**< CHSEL3_SLOT12_ID Mask */

#define MXC_F_ADC_CHSEL3_SLOT13_ID_POS                 8 /**< CHSEL3_SLOT13_ID Position */
#define MXC_F_ADC_CHSEL3_SLOT13_ID                     ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL3_SLOT13_ID_POS)) /**< CHSEL3_SLOT13_ID Mask */

#define MXC_F_ADC_CHSEL3_SLOT14_ID_POS                 16 /**< CHSEL3_SLOT14_ID Position */
#define MXC_F_ADC_CHSEL3_SLOT14_ID                     ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL3_SLOT14_ID_POS)) /**< CHSEL3_SLOT14_ID Mask */

#define MXC_F_ADC_CHSEL3_SLOT15_ID_POS                 24 /**< CHSEL3_SLOT15_ID Position */
#define MXC_F_ADC_CHSEL3_SLOT15_ID                     ((uint32_t)(0x1FUL << MXC_F_ADC_CHSEL3_SLOT15_ID_POS)) /**< CHSEL3_SLOT15_ID Mask */

/**@} end of group ADC_CHSEL3_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_RESTART ADC_RESTART
 * @brief    Restart Count Control Register
 * @{
 */
#define MXC_F_ADC_RESTART_CNT_POS                      0 /**< RESTART_CNT Position */
#define MXC_F_ADC_RESTART_CNT                          ((uint32_t)(0xFFFFUL << MXC_F_ADC_RESTART_CNT_POS)) /**< RESTART_CNT Mask */

/**@} end of group ADC_RESTART_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_DATAFMT ADC_DATAFMT
 * @brief    Channel Data Format Register
 * @{
 */
#define MXC_F_ADC_DATAFMT_MODE_POS                     0 /**< DATAFMT_MODE Position */
#define MXC_F_ADC_DATAFMT_MODE                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_ADC_DATAFMT_MODE_POS)) /**< DATAFMT_MODE Mask */

/**@} end of group ADC_DATAFMT_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_FIFODMACTRL ADC_FIFODMACTRL
 * @brief    FIFO and DMA control
 * @{
 */
#define MXC_F_ADC_FIFODMACTRL_DMA_EN_POS               0 /**< FIFODMACTRL_DMA_EN Position */
#define MXC_F_ADC_FIFODMACTRL_DMA_EN                   ((uint32_t)(0x1UL << MXC_F_ADC_FIFODMACTRL_DMA_EN_POS)) /**< FIFODMACTRL_DMA_EN Mask */

#define MXC_F_ADC_FIFODMACTRL_FLUSH_POS                1 /**< FIFODMACTRL_FLUSH Position */
#define MXC_F_ADC_FIFODMACTRL_FLUSH                    ((uint32_t)(0x1UL << MXC_F_ADC_FIFODMACTRL_FLUSH_POS)) /**< FIFODMACTRL_FLUSH Mask */

#define MXC_F_ADC_FIFODMACTRL_DATA_FORMAT_POS          2 /**< FIFODMACTRL_DATA_FORMAT Position */
#define MXC_F_ADC_FIFODMACTRL_DATA_FORMAT              ((uint32_t)(0x3UL << MXC_F_ADC_FIFODMACTRL_DATA_FORMAT_POS)) /**< FIFODMACTRL_DATA_FORMAT Mask */
#define MXC_V_ADC_FIFODMACTRL_DATA_FORMAT_DATA_STATUS  ((uint32_t)0x0UL) /**< FIFODMACTRL_DATA_FORMAT_DATA_STATUS Value */
#define MXC_S_ADC_FIFODMACTRL_DATA_FORMAT_DATA_STATUS  (MXC_V_ADC_FIFODMACTRL_DATA_FORMAT_DATA_STATUS << MXC_F_ADC_FIFODMACTRL_DATA_FORMAT_POS) /**< FIFODMACTRL_DATA_FORMAT_DATA_STATUS Setting */
#define MXC_V_ADC_FIFODMACTRL_DATA_FORMAT_DATA_ONLY    ((uint32_t)0x1UL) /**< FIFODMACTRL_DATA_FORMAT_DATA_ONLY Value */
#define MXC_S_ADC_FIFODMACTRL_DATA_FORMAT_DATA_ONLY    (MXC_V_ADC_FIFODMACTRL_DATA_FORMAT_DATA_ONLY << MXC_F_ADC_FIFODMACTRL_DATA_FORMAT_POS) /**< FIFODMACTRL_DATA_FORMAT_DATA_ONLY Setting */
#define MXC_V_ADC_FIFODMACTRL_DATA_FORMAT_RAW_DATA_ONLY ((uint32_t)0x2UL) /**< FIFODMACTRL_DATA_FORMAT_RAW_DATA_ONLY Value */
#define MXC_S_ADC_FIFODMACTRL_DATA_FORMAT_RAW_DATA_ONLY (MXC_V_ADC_FIFODMACTRL_DATA_FORMAT_RAW_DATA_ONLY << MXC_F_ADC_FIFODMACTRL_DATA_FORMAT_POS) /**< FIFODMACTRL_DATA_FORMAT_RAW_DATA_ONLY Setting */

#define MXC_F_ADC_FIFODMACTRL_THRESH_POS               8 /**< FIFODMACTRL_THRESH Position */
#define MXC_F_ADC_FIFODMACTRL_THRESH                   ((uint32_t)(0xFFUL << MXC_F_ADC_FIFODMACTRL_THRESH_POS)) /**< FIFODMACTRL_THRESH Mask */

/**@} end of group ADC_FIFODMACTRL_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_DATA ADC_DATA
 * @brief    Data Register (FIFO).
 * @{
 */
#define MXC_F_ADC_DATA_DATA_POS                        0 /**< DATA_DATA Position */
#define MXC_F_ADC_DATA_DATA                            ((uint32_t)(0xFFFFUL << MXC_F_ADC_DATA_DATA_POS)) /**< DATA_DATA Mask */

#define MXC_F_ADC_DATA_CHAN_POS                        16 /**< DATA_CHAN Position */
#define MXC_F_ADC_DATA_CHAN                            ((uint32_t)(0x1FUL << MXC_F_ADC_DATA_CHAN_POS)) /**< DATA_CHAN Mask */

#define MXC_F_ADC_DATA_INVALID_POS                     24 /**< DATA_INVALID Position */
#define MXC_F_ADC_DATA_INVALID                         ((uint32_t)(0x1UL << MXC_F_ADC_DATA_INVALID_POS)) /**< DATA_INVALID Mask */

#define MXC_F_ADC_DATA_CLIPPED_POS                     31 /**< DATA_CLIPPED Position */
#define MXC_F_ADC_DATA_CLIPPED                         ((uint32_t)(0x1UL << MXC_F_ADC_DATA_CLIPPED_POS)) /**< DATA_CLIPPED Mask */

/**@} end of group ADC_DATA_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_STATUS ADC_STATUS
 * @brief    Status Register
 * @{
 */
#define MXC_F_ADC_STATUS_READY_POS                     0 /**< STATUS_READY Position */
#define MXC_F_ADC_STATUS_READY                         ((uint32_t)(0x1UL << MXC_F_ADC_STATUS_READY_POS)) /**< STATUS_READY Mask */

#define MXC_F_ADC_STATUS_EMPTY_POS                     1 /**< STATUS_EMPTY Position */
#define MXC_F_ADC_STATUS_EMPTY                         ((uint32_t)(0x1UL << MXC_F_ADC_STATUS_EMPTY_POS)) /**< STATUS_EMPTY Mask */

#define MXC_F_ADC_STATUS_FULL_POS                      2 /**< STATUS_FULL Position */
#define MXC_F_ADC_STATUS_FULL                          ((uint32_t)(0x1UL << MXC_F_ADC_STATUS_FULL_POS)) /**< STATUS_FULL Mask */

#define MXC_F_ADC_STATUS_FIFO_LEVEL_POS                8 /**< STATUS_FIFO_LEVEL Position */
#define MXC_F_ADC_STATUS_FIFO_LEVEL                    ((uint32_t)(0xFFUL << MXC_F_ADC_STATUS_FIFO_LEVEL_POS)) /**< STATUS_FIFO_LEVEL Mask */

/**@} end of group ADC_STATUS_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_CHSTATUS ADC_CHSTATUS
 * @brief    Channel Status
 * @{
 */
#define MXC_F_ADC_CHSTATUS_CLIPPED_POS                 0 /**< CHSTATUS_CLIPPED Position */
#define MXC_F_ADC_CHSTATUS_CLIPPED                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_ADC_CHSTATUS_CLIPPED_POS)) /**< CHSTATUS_CLIPPED Mask */

/**@} end of group ADC_CHSTATUS_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_INTEN ADC_INTEN
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_ADC_INTEN_READY_POS                      0 /**< INTEN_READY Position */
#define MXC_F_ADC_INTEN_READY                          ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_READY_POS)) /**< INTEN_READY Mask */

#define MXC_F_ADC_INTEN_ABORT_POS                      2 /**< INTEN_ABORT Position */
#define MXC_F_ADC_INTEN_ABORT                          ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_ABORT_POS)) /**< INTEN_ABORT Mask */

#define MXC_F_ADC_INTEN_START_DET_POS                  3 /**< INTEN_START_DET Position */
#define MXC_F_ADC_INTEN_START_DET                      ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_START_DET_POS)) /**< INTEN_START_DET Mask */

#define MXC_F_ADC_INTEN_SEQ_STARTED_POS                4 /**< INTEN_SEQ_STARTED Position */
#define MXC_F_ADC_INTEN_SEQ_STARTED                    ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_SEQ_STARTED_POS)) /**< INTEN_SEQ_STARTED Mask */

#define MXC_F_ADC_INTEN_SEQ_DONE_POS                   5 /**< INTEN_SEQ_DONE Position */
#define MXC_F_ADC_INTEN_SEQ_DONE                       ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_SEQ_DONE_POS)) /**< INTEN_SEQ_DONE Mask */

#define MXC_F_ADC_INTEN_CONV_DONE_POS                  6 /**< INTEN_CONV_DONE Position */
#define MXC_F_ADC_INTEN_CONV_DONE                      ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_CONV_DONE_POS)) /**< INTEN_CONV_DONE Mask */

#define MXC_F_ADC_INTEN_CLIPPED_POS                    7 /**< INTEN_CLIPPED Position */
#define MXC_F_ADC_INTEN_CLIPPED                        ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_CLIPPED_POS)) /**< INTEN_CLIPPED Mask */

#define MXC_F_ADC_INTEN_FIFO_LVL_POS                   8 /**< INTEN_FIFO_LVL Position */
#define MXC_F_ADC_INTEN_FIFO_LVL                       ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_FIFO_LVL_POS)) /**< INTEN_FIFO_LVL Mask */

#define MXC_F_ADC_INTEN_FIFO_UFL_POS                   9 /**< INTEN_FIFO_UFL Position */
#define MXC_F_ADC_INTEN_FIFO_UFL                       ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_FIFO_UFL_POS)) /**< INTEN_FIFO_UFL Mask */

#define MXC_F_ADC_INTEN_FIFO_OFL_POS                   10 /**< INTEN_FIFO_OFL Position */
#define MXC_F_ADC_INTEN_FIFO_OFL                       ((uint32_t)(0x1UL << MXC_F_ADC_INTEN_FIFO_OFL_POS)) /**< INTEN_FIFO_OFL Mask */

/**@} end of group ADC_INTEN_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_INTFL ADC_INTFL
 * @brief    Interrupt Flags Register.
 * @{
 */
#define MXC_F_ADC_INTFL_READY_POS                      0 /**< INTFL_READY Position */
#define MXC_F_ADC_INTFL_READY                          ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_READY_POS)) /**< INTFL_READY Mask */

#define MXC_F_ADC_INTFL_ABORT_POS                      2 /**< INTFL_ABORT Position */
#define MXC_F_ADC_INTFL_ABORT                          ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_ABORT_POS)) /**< INTFL_ABORT Mask */

#define MXC_F_ADC_INTFL_START_DET_POS                  3 /**< INTFL_START_DET Position */
#define MXC_F_ADC_INTFL_START_DET                      ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_START_DET_POS)) /**< INTFL_START_DET Mask */

#define MXC_F_ADC_INTFL_SEQ_STARTED_POS                4 /**< INTFL_SEQ_STARTED Position */
#define MXC_F_ADC_INTFL_SEQ_STARTED                    ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_SEQ_STARTED_POS)) /**< INTFL_SEQ_STARTED Mask */

#define MXC_F_ADC_INTFL_SEQ_DONE_POS                   5 /**< INTFL_SEQ_DONE Position */
#define MXC_F_ADC_INTFL_SEQ_DONE                       ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_SEQ_DONE_POS)) /**< INTFL_SEQ_DONE Mask */

#define MXC_F_ADC_INTFL_CONV_DONE_POS                  6 /**< INTFL_CONV_DONE Position */
#define MXC_F_ADC_INTFL_CONV_DONE                      ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_CONV_DONE_POS)) /**< INTFL_CONV_DONE Mask */

#define MXC_F_ADC_INTFL_CLIPPED_POS                    7 /**< INTFL_CLIPPED Position */
#define MXC_F_ADC_INTFL_CLIPPED                        ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_CLIPPED_POS)) /**< INTFL_CLIPPED Mask */

#define MXC_F_ADC_INTFL_FIFO_LVL_POS                   8 /**< INTFL_FIFO_LVL Position */
#define MXC_F_ADC_INTFL_FIFO_LVL                       ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_FIFO_LVL_POS)) /**< INTFL_FIFO_LVL Mask */

#define MXC_F_ADC_INTFL_FIFO_UFL_POS                   9 /**< INTFL_FIFO_UFL Position */
#define MXC_F_ADC_INTFL_FIFO_UFL                       ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_FIFO_UFL_POS)) /**< INTFL_FIFO_UFL Mask */

#define MXC_F_ADC_INTFL_FIFO_OFL_POS                   10 /**< INTFL_FIFO_OFL Position */
#define MXC_F_ADC_INTFL_FIFO_OFL                       ((uint32_t)(0x1UL << MXC_F_ADC_INTFL_FIFO_OFL_POS)) /**< INTFL_FIFO_OFL Mask */

/**@} end of group ADC_INTFL_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_SFRADDROFFSET ADC_SFRADDROFFSET
 * @brief    SFR Address Offset Register
 * @{
 */
#define MXC_F_ADC_SFRADDROFFSET_OFFSET_POS             0 /**< SFRADDROFFSET_OFFSET Position */
#define MXC_F_ADC_SFRADDROFFSET_OFFSET                 ((uint32_t)(0xFFUL << MXC_F_ADC_SFRADDROFFSET_OFFSET_POS)) /**< SFRADDROFFSET_OFFSET Mask */

/**@} end of group ADC_SFRADDROFFSET_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_SFRADDR ADC_SFRADDR
 * @brief    SFR Address Register
 * @{
 */
#define MXC_F_ADC_SFRADDR_ADDR_POS                     0 /**< SFRADDR_ADDR Position */
#define MXC_F_ADC_SFRADDR_ADDR                         ((uint32_t)(0xFFUL << MXC_F_ADC_SFRADDR_ADDR_POS)) /**< SFRADDR_ADDR Mask */

/**@} end of group ADC_SFRADDR_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_SFRWRDATA ADC_SFRWRDATA
 * @brief    SFR Write Data Register
 * @{
 */
#define MXC_F_ADC_SFRWRDATA_DATA_POS                   0 /**< SFRWRDATA_DATA Position */
#define MXC_F_ADC_SFRWRDATA_DATA                       ((uint32_t)(0xFFUL << MXC_F_ADC_SFRWRDATA_DATA_POS)) /**< SFRWRDATA_DATA Mask */

/**@} end of group ADC_SFRWRDATA_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_SFRRDDATA ADC_SFRRDDATA
 * @brief    SFR Read Data Register
 * @{
 */
#define MXC_F_ADC_SFRRDDATA_DATA_POS                   0 /**< SFRRDDATA_DATA Position */
#define MXC_F_ADC_SFRRDDATA_DATA                       ((uint32_t)(0xFFUL << MXC_F_ADC_SFRRDDATA_DATA_POS)) /**< SFRRDDATA_DATA Mask */

/**@} end of group ADC_SFRRDDATA_Register */

/**
 * @ingroup  adc_registers
 * @defgroup ADC_SFRSTATUS ADC_SFRSTATUS
 * @brief    SFR Status Register
 * @{
 */
#define MXC_F_ADC_SFRSTATUS_NACK_POS                   0 /**< SFRSTATUS_NACK Position */
#define MXC_F_ADC_SFRSTATUS_NACK                       ((uint32_t)(0x1UL << MXC_F_ADC_SFRSTATUS_NACK_POS)) /**< SFRSTATUS_NACK Mask */

/**@} end of group ADC_SFRSTATUS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_ADC_REGS_H_
