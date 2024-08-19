/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_MCR_REGS_H_

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
 * @ingroup     mcr
 * @defgroup    mcr_registers MCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @details     Misc Control.
 */

/**
 * @ingroup mcr_registers
 * Structure type to access the MCR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0;
    __IO uint32_t rst;                  /**< <tt>\b 0x04:</tt> MCR RST Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x08:</tt> MCR CLKCTRL Register */
    __IO uint32_t aincomp;              /**< <tt>\b 0x0C:</tt> MCR AINCOMP Register */
    __IO uint32_t lppioctrl;            /**< <tt>\b 0x10:</tt> MCR LPPIOCTRL Register */
    __R  uint32_t rsv_0x14_0x23[4];
    __IO uint32_t pclkdis;              /**< <tt>\b 0x24:</tt> MCR PCLKDIS Register */
    __R  uint32_t rsv_0x28_0x33[3];
    __IO uint32_t aeskey;               /**< <tt>\b 0x34:</tt> MCR AESKEY Register */
    __IO uint32_t adc_cfg0;             /**< <tt>\b 0x38:</tt> MCR ADC_CFG0 Register */
    __IO uint32_t adc_cfg1;             /**< <tt>\b 0x3C:</tt> MCR ADC_CFG1 Register */
    __IO uint32_t adc_cfg2;             /**< <tt>\b 0x40:</tt> MCR ADC_CFG2 Register */
    __IO uint32_t adc_cfg3;             /**< <tt>\b 0x44:</tt> MCR ADC_CFG3 Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_RST                      ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_CLKCTRL                  ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_AINCOMP                  ((uint32_t)0x0000000CUL) /**< Offset from MCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_MCR_LPPIOCTRL                ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_PCLKDIS                  ((uint32_t)0x00000024UL) /**< Offset from MCR Base Address: <tt> 0x0024</tt> */
#define MXC_R_MCR_AESKEY                   ((uint32_t)0x00000034UL) /**< Offset from MCR Base Address: <tt> 0x0034</tt> */
#define MXC_R_MCR_ADC_CFG0                 ((uint32_t)0x00000038UL) /**< Offset from MCR Base Address: <tt> 0x0038</tt> */
#define MXC_R_MCR_ADC_CFG1                 ((uint32_t)0x0000003CUL) /**< Offset from MCR Base Address: <tt> 0x003C</tt> */
#define MXC_R_MCR_ADC_CFG2                 ((uint32_t)0x00000040UL) /**< Offset from MCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_MCR_ADC_CFG3                 ((uint32_t)0x00000044UL) /**< Offset from MCR Base Address: <tt> 0x0044</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RST MCR_RST
 * @brief    Low Power Reset Control Register
 * @{
 */
#define MXC_F_MCR_RST_LPTMR0_POS                       0 /**< RST_LPTMR0 Position */
#define MXC_F_MCR_RST_LPTMR0                           ((uint32_t)(0x1UL << MXC_F_MCR_RST_LPTMR0_POS)) /**< RST_LPTMR0 Mask */

#define MXC_F_MCR_RST_LPTMR1_POS                       1 /**< RST_LPTMR1 Position */
#define MXC_F_MCR_RST_LPTMR1                           ((uint32_t)(0x1UL << MXC_F_MCR_RST_LPTMR1_POS)) /**< RST_LPTMR1 Mask */

#define MXC_F_MCR_RST_LPUART0_POS                      2 /**< RST_LPUART0 Position */
#define MXC_F_MCR_RST_LPUART0                          ((uint32_t)(0x1UL << MXC_F_MCR_RST_LPUART0_POS)) /**< RST_LPUART0 Mask */

#define MXC_F_MCR_RST_RTC_POS                          3 /**< RST_RTC Position */
#define MXC_F_MCR_RST_RTC                              ((uint32_t)(0x1UL << MXC_F_MCR_RST_RTC_POS)) /**< RST_RTC Mask */

/**@} end of group MCR_RST_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CLKCTRL MCR_CLKCTRL
 * @brief    Clock Control.
 * @{
 */
#define MXC_F_MCR_CLKCTRL_ERTCO_PD_POS                 16 /**< CLKCTRL_ERTCO_PD Position */
#define MXC_F_MCR_CLKCTRL_ERTCO_PD                     ((uint32_t)(0x1UL << MXC_F_MCR_CLKCTRL_ERTCO_PD_POS)) /**< CLKCTRL_ERTCO_PD Mask */

#define MXC_F_MCR_CLKCTRL_ERTCO_EN_POS                 17 /**< CLKCTRL_ERTCO_EN Position */
#define MXC_F_MCR_CLKCTRL_ERTCO_EN                     ((uint32_t)(0x1UL << MXC_F_MCR_CLKCTRL_ERTCO_EN_POS)) /**< CLKCTRL_ERTCO_EN Mask */

/**@} end of group MCR_CLKCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_AINCOMP MCR_AINCOMP
 * @brief    AIN Comparator.
 * @{
 */
#define MXC_F_MCR_AINCOMP_PD_POS                       0 /**< AINCOMP_PD Position */
#define MXC_F_MCR_AINCOMP_PD                           ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_PD_POS)) /**< AINCOMP_PD Mask */

#define MXC_F_MCR_AINCOMP_HYST_POS                     2 /**< AINCOMP_HYST Position */
#define MXC_F_MCR_AINCOMP_HYST                         ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_HYST_POS)) /**< AINCOMP_HYST Mask */

#define MXC_F_MCR_AINCOMP_NSEL_COMP0_POS               16 /**< AINCOMP_NSEL_COMP0 Position */
#define MXC_F_MCR_AINCOMP_NSEL_COMP0                   ((uint32_t)(0xFUL << MXC_F_MCR_AINCOMP_NSEL_COMP0_POS)) /**< AINCOMP_NSEL_COMP0 Mask */

#define MXC_F_MCR_AINCOMP_PSEL_COMP0_POS               20 /**< AINCOMP_PSEL_COMP0 Position */
#define MXC_F_MCR_AINCOMP_PSEL_COMP0                   ((uint32_t)(0xFUL << MXC_F_MCR_AINCOMP_PSEL_COMP0_POS)) /**< AINCOMP_PSEL_COMP0 Mask */

#define MXC_F_MCR_AINCOMP_NSEL_COMP1_POS               24 /**< AINCOMP_NSEL_COMP1 Position */
#define MXC_F_MCR_AINCOMP_NSEL_COMP1                   ((uint32_t)(0xFUL << MXC_F_MCR_AINCOMP_NSEL_COMP1_POS)) /**< AINCOMP_NSEL_COMP1 Mask */

#define MXC_F_MCR_AINCOMP_PSEL_COMP1_POS               28 /**< AINCOMP_PSEL_COMP1 Position */
#define MXC_F_MCR_AINCOMP_PSEL_COMP1                   ((uint32_t)(0xFUL << MXC_F_MCR_AINCOMP_PSEL_COMP1_POS)) /**< AINCOMP_PSEL_COMP1 Mask */

/**@} end of group MCR_AINCOMP_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LPPIOCTRL MCR_LPPIOCTRL
 * @brief    Low Power Peripheral IO Control Register.
 * @{
 */
#define MXC_F_MCR_LPPIOCTRL_LPTMR0_I_POS               0 /**< LPPIOCTRL_LPTMR0_I Position */
#define MXC_F_MCR_LPPIOCTRL_LPTMR0_I                   ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPTMR0_I_POS)) /**< LPPIOCTRL_LPTMR0_I Mask */

#define MXC_F_MCR_LPPIOCTRL_LPTMR0_O_POS               1 /**< LPPIOCTRL_LPTMR0_O Position */
#define MXC_F_MCR_LPPIOCTRL_LPTMR0_O                   ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPTMR0_O_POS)) /**< LPPIOCTRL_LPTMR0_O Mask */

#define MXC_F_MCR_LPPIOCTRL_LPTMR1_I_POS               2 /**< LPPIOCTRL_LPTMR1_I Position */
#define MXC_F_MCR_LPPIOCTRL_LPTMR1_I                   ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPTMR1_I_POS)) /**< LPPIOCTRL_LPTMR1_I Mask */

#define MXC_F_MCR_LPPIOCTRL_LPTMR1_O_POS               3 /**< LPPIOCTRL_LPTMR1_O Position */
#define MXC_F_MCR_LPPIOCTRL_LPTMR1_O                   ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPTMR1_O_POS)) /**< LPPIOCTRL_LPTMR1_O Mask */

#define MXC_F_MCR_LPPIOCTRL_LPUART0_RX_POS             4 /**< LPPIOCTRL_LPUART0_RX Position */
#define MXC_F_MCR_LPPIOCTRL_LPUART0_RX                 ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPUART0_RX_POS)) /**< LPPIOCTRL_LPUART0_RX Mask */

#define MXC_F_MCR_LPPIOCTRL_LPUART0_TX_POS             5 /**< LPPIOCTRL_LPUART0_TX Position */
#define MXC_F_MCR_LPPIOCTRL_LPUART0_TX                 ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPUART0_TX_POS)) /**< LPPIOCTRL_LPUART0_TX Mask */

#define MXC_F_MCR_LPPIOCTRL_LPUART0_CTS_POS            6 /**< LPPIOCTRL_LPUART0_CTS Position */
#define MXC_F_MCR_LPPIOCTRL_LPUART0_CTS                ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPUART0_CTS_POS)) /**< LPPIOCTRL_LPUART0_CTS Mask */

#define MXC_F_MCR_LPPIOCTRL_LPUART0_RTS_POS            7 /**< LPPIOCTRL_LPUART0_RTS Position */
#define MXC_F_MCR_LPPIOCTRL_LPUART0_RTS                ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_LPUART0_RTS_POS)) /**< LPPIOCTRL_LPUART0_RTS Mask */

/**@} end of group MCR_LPPIOCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PCLKDIS MCR_PCLKDIS
 * @brief    Low Power Peripheral Clock Disable.
 * @{
 */
#define MXC_F_MCR_PCLKDIS_LPTMR0_POS                   0 /**< PCLKDIS_LPTMR0 Position */
#define MXC_F_MCR_PCLKDIS_LPTMR0                       ((uint32_t)(0x1UL << MXC_F_MCR_PCLKDIS_LPTMR0_POS)) /**< PCLKDIS_LPTMR0 Mask */

#define MXC_F_MCR_PCLKDIS_LPTMR1_POS                   1 /**< PCLKDIS_LPTMR1 Position */
#define MXC_F_MCR_PCLKDIS_LPTMR1                       ((uint32_t)(0x1UL << MXC_F_MCR_PCLKDIS_LPTMR1_POS)) /**< PCLKDIS_LPTMR1 Mask */

#define MXC_F_MCR_PCLKDIS_LPUART0_POS                  2 /**< PCLKDIS_LPUART0 Position */
#define MXC_F_MCR_PCLKDIS_LPUART0                      ((uint32_t)(0x1UL << MXC_F_MCR_PCLKDIS_LPUART0_POS)) /**< PCLKDIS_LPUART0 Mask */

/**@} end of group MCR_PCLKDIS_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_AESKEY MCR_AESKEY
 * @brief    AES Key Pointer and Status.
 * @{
 */
#define MXC_F_MCR_AESKEY_PTR_POS                       0 /**< AESKEY_PTR Position */
#define MXC_F_MCR_AESKEY_PTR                           ((uint32_t)(0xFFFFUL << MXC_F_MCR_AESKEY_PTR_POS)) /**< AESKEY_PTR Mask */

/**@} end of group MCR_AESKEY_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADC_CFG0 MCR_ADC_CFG0
 * @brief    ADC Cfig Register0.
 * @{
 */
#define MXC_F_MCR_ADC_CFG0_LP_5K_DIS_POS               0 /**< ADC_CFG0_LP_5K_DIS Position */
#define MXC_F_MCR_ADC_CFG0_LP_5K_DIS                   ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG0_LP_5K_DIS_POS)) /**< ADC_CFG0_LP_5K_DIS Mask */

#define MXC_F_MCR_ADC_CFG0_LP_50K_DIS_POS              1 /**< ADC_CFG0_LP_50K_DIS Position */
#define MXC_F_MCR_ADC_CFG0_LP_50K_DIS                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG0_LP_50K_DIS_POS)) /**< ADC_CFG0_LP_50K_DIS Mask */

#define MXC_F_MCR_ADC_CFG0_EXT_REF_POS                 2 /**< ADC_CFG0_EXT_REF Position */
#define MXC_F_MCR_ADC_CFG0_EXT_REF                     ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG0_EXT_REF_POS)) /**< ADC_CFG0_EXT_REF Mask */

#define MXC_F_MCR_ADC_CFG0_REF_SEL_POS                 3 /**< ADC_CFG0_REF_SEL Position */
#define MXC_F_MCR_ADC_CFG0_REF_SEL                     ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG0_REF_SEL_POS)) /**< ADC_CFG0_REF_SEL Mask */

/**@} end of group MCR_ADC_CFG0_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADC_CFG1 MCR_ADC_CFG1
 * @brief    ADC Config Register1.
 * @{
 */
#define MXC_F_MCR_ADC_CFG1_CH0_PU_DYN_POS              0 /**< ADC_CFG1_CH0_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH0_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH0_PU_DYN_POS)) /**< ADC_CFG1_CH0_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH1_PU_DYN_POS              1 /**< ADC_CFG1_CH1_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH1_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH1_PU_DYN_POS)) /**< ADC_CFG1_CH1_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH2_PU_DYN_POS              2 /**< ADC_CFG1_CH2_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH2_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH2_PU_DYN_POS)) /**< ADC_CFG1_CH2_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH3_PU_DYN_POS              3 /**< ADC_CFG1_CH3_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH3_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH3_PU_DYN_POS)) /**< ADC_CFG1_CH3_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH4_PU_DYN_POS              4 /**< ADC_CFG1_CH4_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH4_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH4_PU_DYN_POS)) /**< ADC_CFG1_CH4_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH5_PU_DYN_POS              5 /**< ADC_CFG1_CH5_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH5_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH5_PU_DYN_POS)) /**< ADC_CFG1_CH5_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH6_PU_DYN_POS              6 /**< ADC_CFG1_CH6_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH6_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH6_PU_DYN_POS)) /**< ADC_CFG1_CH6_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH7_PU_DYN_POS              7 /**< ADC_CFG1_CH7_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH7_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH7_PU_DYN_POS)) /**< ADC_CFG1_CH7_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH8_PU_DYN_POS              8 /**< ADC_CFG1_CH8_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH8_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH8_PU_DYN_POS)) /**< ADC_CFG1_CH8_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH9_PU_DYN_POS              9 /**< ADC_CFG1_CH9_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH9_PU_DYN                  ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH9_PU_DYN_POS)) /**< ADC_CFG1_CH9_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH10_PU_DYN_POS             10 /**< ADC_CFG1_CH10_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH10_PU_DYN                 ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH10_PU_DYN_POS)) /**< ADC_CFG1_CH10_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH11_PU_DYN_POS             11 /**< ADC_CFG1_CH11_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH11_PU_DYN                 ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH11_PU_DYN_POS)) /**< ADC_CFG1_CH11_PU_DYN Mask */

#define MXC_F_MCR_ADC_CFG1_CH12_PU_DYN_POS             12 /**< ADC_CFG1_CH12_PU_DYN Position */
#define MXC_F_MCR_ADC_CFG1_CH12_PU_DYN                 ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG1_CH12_PU_DYN_POS)) /**< ADC_CFG1_CH12_PU_DYN Mask */

/**@} end of group MCR_ADC_CFG1_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADC_CFG2 MCR_ADC_CFG2
 * @brief    ADC Config Register2.
 * @{
 */
#define MXC_F_MCR_ADC_CFG2_CH0_POS                     0 /**< ADC_CFG2_CH0 Position */
#define MXC_F_MCR_ADC_CFG2_CH0                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH0_POS)) /**< ADC_CFG2_CH0 Mask */
#define MXC_V_MCR_ADC_CFG2_CH0_DIV1                    ((uint32_t)0x0UL) /**< ADC_CFG2_CH0_DIV1 Value */
#define MXC_S_MCR_ADC_CFG2_CH0_DIV1                    (MXC_V_MCR_ADC_CFG2_CH0_DIV1 << MXC_F_MCR_ADC_CFG2_CH0_POS) /**< ADC_CFG2_CH0_DIV1 Setting */
#define MXC_V_MCR_ADC_CFG2_CH0_DIV2_5K                 ((uint32_t)0x1UL) /**< ADC_CFG2_CH0_DIV2_5K Value */
#define MXC_S_MCR_ADC_CFG2_CH0_DIV2_5K                 (MXC_V_MCR_ADC_CFG2_CH0_DIV2_5K << MXC_F_MCR_ADC_CFG2_CH0_POS) /**< ADC_CFG2_CH0_DIV2_5K Setting */
#define MXC_V_MCR_ADC_CFG2_CH0_DIV2_50K                ((uint32_t)0x2UL) /**< ADC_CFG2_CH0_DIV2_50K Value */
#define MXC_S_MCR_ADC_CFG2_CH0_DIV2_50K                (MXC_V_MCR_ADC_CFG2_CH0_DIV2_50K << MXC_F_MCR_ADC_CFG2_CH0_POS) /**< ADC_CFG2_CH0_DIV2_50K Setting */

#define MXC_F_MCR_ADC_CFG2_CH1_POS                     2 /**< ADC_CFG2_CH1 Position */
#define MXC_F_MCR_ADC_CFG2_CH1                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH1_POS)) /**< ADC_CFG2_CH1 Mask */

#define MXC_F_MCR_ADC_CFG2_CH2_POS                     4 /**< ADC_CFG2_CH2 Position */
#define MXC_F_MCR_ADC_CFG2_CH2                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH2_POS)) /**< ADC_CFG2_CH2 Mask */

#define MXC_F_MCR_ADC_CFG2_CH3_POS                     6 /**< ADC_CFG2_CH3 Position */
#define MXC_F_MCR_ADC_CFG2_CH3                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH3_POS)) /**< ADC_CFG2_CH3 Mask */

#define MXC_F_MCR_ADC_CFG2_CH4_POS                     8 /**< ADC_CFG2_CH4 Position */
#define MXC_F_MCR_ADC_CFG2_CH4                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH4_POS)) /**< ADC_CFG2_CH4 Mask */

#define MXC_F_MCR_ADC_CFG2_CH5_POS                     10 /**< ADC_CFG2_CH5 Position */
#define MXC_F_MCR_ADC_CFG2_CH5                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH5_POS)) /**< ADC_CFG2_CH5 Mask */

#define MXC_F_MCR_ADC_CFG2_CH6_POS                     12 /**< ADC_CFG2_CH6 Position */
#define MXC_F_MCR_ADC_CFG2_CH6                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH6_POS)) /**< ADC_CFG2_CH6 Mask */

#define MXC_F_MCR_ADC_CFG2_CH7_POS                     14 /**< ADC_CFG2_CH7 Position */
#define MXC_F_MCR_ADC_CFG2_CH7                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH7_POS)) /**< ADC_CFG2_CH7 Mask */

#define MXC_F_MCR_ADC_CFG2_CH8_POS                     16 /**< ADC_CFG2_CH8 Position */
#define MXC_F_MCR_ADC_CFG2_CH8                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH8_POS)) /**< ADC_CFG2_CH8 Mask */

#define MXC_F_MCR_ADC_CFG2_CH9_POS                     18 /**< ADC_CFG2_CH9 Position */
#define MXC_F_MCR_ADC_CFG2_CH9                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH9_POS)) /**< ADC_CFG2_CH9 Mask */

#define MXC_F_MCR_ADC_CFG2_CH10_POS                    20 /**< ADC_CFG2_CH10 Position */
#define MXC_F_MCR_ADC_CFG2_CH10                        ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH10_POS)) /**< ADC_CFG2_CH10 Mask */

#define MXC_F_MCR_ADC_CFG2_CH11_POS                    22 /**< ADC_CFG2_CH11 Position */
#define MXC_F_MCR_ADC_CFG2_CH11                        ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH11_POS)) /**< ADC_CFG2_CH11 Mask */

#define MXC_F_MCR_ADC_CFG2_CH12_POS                    24 /**< ADC_CFG2_CH12 Position */
#define MXC_F_MCR_ADC_CFG2_CH12                        ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG2_CH12_POS)) /**< ADC_CFG2_CH12 Mask */

/**@} end of group MCR_ADC_CFG2_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADC_CFG3 MCR_ADC_CFG3
 * @brief    ADC Config Register3.
 * @{
 */
#define MXC_F_MCR_ADC_CFG3_VREFM_POS                   0 /**< ADC_CFG3_VREFM Position */
#define MXC_F_MCR_ADC_CFG3_VREFM                       ((uint32_t)(0x7FUL << MXC_F_MCR_ADC_CFG3_VREFM_POS)) /**< ADC_CFG3_VREFM Mask */

#define MXC_F_MCR_ADC_CFG3_VREFP_POS                   8 /**< ADC_CFG3_VREFP Position */
#define MXC_F_MCR_ADC_CFG3_VREFP                       ((uint32_t)(0x7FUL << MXC_F_MCR_ADC_CFG3_VREFP_POS)) /**< ADC_CFG3_VREFP Mask */

#define MXC_F_MCR_ADC_CFG3_IDRV_POS                    16 /**< ADC_CFG3_IDRV Position */
#define MXC_F_MCR_ADC_CFG3_IDRV                        ((uint32_t)(0xFUL << MXC_F_MCR_ADC_CFG3_IDRV_POS)) /**< ADC_CFG3_IDRV Mask */

#define MXC_F_MCR_ADC_CFG3_VCM_POS                     20 /**< ADC_CFG3_VCM Position */
#define MXC_F_MCR_ADC_CFG3_VCM                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG3_VCM_POS)) /**< ADC_CFG3_VCM Mask */

#define MXC_F_MCR_ADC_CFG3_ATB_POS                     22 /**< ADC_CFG3_ATB Position */
#define MXC_F_MCR_ADC_CFG3_ATB                         ((uint32_t)(0x3UL << MXC_F_MCR_ADC_CFG3_ATB_POS)) /**< ADC_CFG3_ATB Mask */

#define MXC_F_MCR_ADC_CFG3_D_IBOOST_POS                24 /**< ADC_CFG3_D_IBOOST Position */
#define MXC_F_MCR_ADC_CFG3_D_IBOOST                    ((uint32_t)(0x1UL << MXC_F_MCR_ADC_CFG3_D_IBOOST_POS)) /**< ADC_CFG3_D_IBOOST Mask */

/**@} end of group MCR_ADC_CFG3_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_MCR_REGS_H_
