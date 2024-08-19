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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_MCR_REGS_H_

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
    __IO uint32_t adccfg0;              /**< <tt>\b 0x38:</tt> MCR ADCCFG0 Register */
    __IO uint32_t adccfg1;              /**< <tt>\b 0x3C:</tt> MCR ADCCFG1 Register */
    __IO uint32_t adccfg2;              /**< <tt>\b 0x40:</tt> MCR ADCCFG2 Register */
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
#define MXC_R_MCR_ADCCFG0                  ((uint32_t)0x00000038UL) /**< Offset from MCR Base Address: <tt> 0x0038</tt> */
#define MXC_R_MCR_ADCCFG1                  ((uint32_t)0x0000003CUL) /**< Offset from MCR Base Address: <tt> 0x003C</tt> */
#define MXC_R_MCR_ADCCFG2                  ((uint32_t)0x00000040UL) /**< Offset from MCR Base Address: <tt> 0x0040</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RST MCR_RST
 * @brief    Reset Register.
 * @{
 */
#define MXC_F_MCR_RST_TMR3_POS                         0 /**< RST_TMR3 Position */
#define MXC_F_MCR_RST_TMR3                             ((uint32_t)(0x1UL << MXC_F_MCR_RST_TMR3_POS)) /**< RST_TMR3 Mask */

#define MXC_F_MCR_RST_RTC_POS                          3 /**< RST_RTC Position */
#define MXC_F_MCR_RST_RTC                              ((uint32_t)(0x1UL << MXC_F_MCR_RST_RTC_POS)) /**< RST_RTC Mask */

/**@} end of group MCR_RST_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CLKCTRL MCR_CLKCTRL
 * @brief    System CLock Control Register.
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
 * @brief    AIN Comparator Control Register.
 * @{
 */
#define MXC_F_MCR_AINCOMP_PD_POS                       0 /**< AINCOMP_PD Position */
#define MXC_F_MCR_AINCOMP_PD                           ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_PD_POS)) /**< AINCOMP_PD Mask */

#define MXC_F_MCR_AINCOMP_HYST_POS                     2 /**< AINCOMP_HYST Position */
#define MXC_F_MCR_AINCOMP_HYST                         ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_HYST_POS)) /**< AINCOMP_HYST Mask */

#define MXC_F_MCR_AINCOMP_NSEL_COMP0_POS               16 /**< AINCOMP_NSEL_COMP0 Position */
#define MXC_F_MCR_AINCOMP_NSEL_COMP0                   ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_NSEL_COMP0_POS)) /**< AINCOMP_NSEL_COMP0 Mask */

#define MXC_F_MCR_AINCOMP_PSEL_COMP0_POS               20 /**< AINCOMP_PSEL_COMP0 Position */
#define MXC_F_MCR_AINCOMP_PSEL_COMP0                   ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_PSEL_COMP0_POS)) /**< AINCOMP_PSEL_COMP0 Mask */

#define MXC_F_MCR_AINCOMP_NSEL_COMP1_POS               24 /**< AINCOMP_NSEL_COMP1 Position */
#define MXC_F_MCR_AINCOMP_NSEL_COMP1                   ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_NSEL_COMP1_POS)) /**< AINCOMP_NSEL_COMP1 Mask */

#define MXC_F_MCR_AINCOMP_PSEL_COMP1_POS               28 /**< AINCOMP_PSEL_COMP1 Position */
#define MXC_F_MCR_AINCOMP_PSEL_COMP1                   ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_PSEL_COMP1_POS)) /**< AINCOMP_PSEL_COMP1 Mask */

/**@} end of group MCR_AINCOMP_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LPPIOCTRL MCR_LPPIOCTRL
 * @brief    Low Power Peripheral IO Control Register.
 * @{
 */
#define MXC_F_MCR_LPPIOCTRL_TMR3_IN_POS                0 /**< LPPIOCTRL_TMR3_IN Position */
#define MXC_F_MCR_LPPIOCTRL_TMR3_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_TMR3_IN_POS)) /**< LPPIOCTRL_TMR3_IN Mask */

#define MXC_F_MCR_LPPIOCTRL_TMR3_OUT_POS               1 /**< LPPIOCTRL_TMR3_OUT Position */
#define MXC_F_MCR_LPPIOCTRL_TMR3_OUT                   ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_TMR3_OUT_POS)) /**< LPPIOCTRL_TMR3_OUT Mask */

#define MXC_F_MCR_LPPIOCTRL_TMR3_OUT_N_POS             2 /**< LPPIOCTRL_TMR3_OUT_N Position */
#define MXC_F_MCR_LPPIOCTRL_TMR3_OUT_N                 ((uint32_t)(0x1UL << MXC_F_MCR_LPPIOCTRL_TMR3_OUT_N_POS)) /**< LPPIOCTRL_TMR3_OUT_N Mask */

/**@} end of group MCR_LPPIOCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PCLKDIS MCR_PCLKDIS
 * @brief    Peripheral Clock Disable Register.
 * @{
 */
#define MXC_F_MCR_PCLKDIS_TMR3_POS                     0 /**< PCLKDIS_TMR3 Position */
#define MXC_F_MCR_PCLKDIS_TMR3                         ((uint32_t)(0x1UL << MXC_F_MCR_PCLKDIS_TMR3_POS)) /**< PCLKDIS_TMR3 Mask */

/**@} end of group MCR_PCLKDIS_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_AESKEY MCR_AESKEY
 * @brief    AES Key Pointer and Status Register.
 * @{
 */
#define MXC_F_MCR_AESKEY_PTR_POS                       0 /**< AESKEY_PTR Position */
#define MXC_F_MCR_AESKEY_PTR                           ((uint32_t)(0xFFFFUL << MXC_F_MCR_AESKEY_PTR_POS)) /**< AESKEY_PTR Mask */

/**@} end of group MCR_AESKEY_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADCCFG0 MCR_ADCCFG0
 * @brief    ADC Config Register 0.
 * @{
 */
#define MXC_F_MCR_ADCCFG0_LP_EXTCLK_EN_POS             0 /**< ADCCFG0_LP_EXTCLK_EN Position */
#define MXC_F_MCR_ADCCFG0_LP_EXTCLK_EN                 ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_LP_EXTCLK_EN_POS)) /**< ADCCFG0_LP_EXTCLK_EN Mask */

#define MXC_F_MCR_ADCCFG0_EXT_REF_POS                  2 /**< ADCCFG0_EXT_REF Position */
#define MXC_F_MCR_ADCCFG0_EXT_REF                      ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_EXT_REF_POS)) /**< ADCCFG0_EXT_REF Mask */

#define MXC_F_MCR_ADCCFG0_INT_REF_POS                  3 /**< ADCCFG0_INT_REF Position */
#define MXC_F_MCR_ADCCFG0_INT_REF                      ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG0_INT_REF_POS)) /**< ADCCFG0_INT_REF Mask */

/**@} end of group MCR_ADCCFG0_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADCCFG1 MCR_ADCCFG1
 * @brief    ADC Config Register 1.
 * @{
 */
#define MXC_F_MCR_ADCCFG1_THRU_PAD_SW_EN_POS           0 /**< ADCCFG1_THRU_PAD_SW_EN Position */
#define MXC_F_MCR_ADCCFG1_THRU_PAD_SW_EN               ((uint32_t)(0xFUL << MXC_F_MCR_ADCCFG1_THRU_PAD_SW_EN_POS)) /**< ADCCFG1_THRU_PAD_SW_EN Mask */

#define MXC_F_MCR_ADCCFG1_AIN_INP_EN_POS               4 /**< ADCCFG1_AIN_INP_EN Position */
#define MXC_F_MCR_ADCCFG1_AIN_INP_EN                   ((uint32_t)(0xFUL << MXC_F_MCR_ADCCFG1_AIN_INP_EN_POS)) /**< ADCCFG1_AIN_INP_EN Mask */

#define MXC_F_MCR_ADCCFG1_THRU_EN_POS                  8 /**< ADCCFG1_THRU_EN Position */
#define MXC_F_MCR_ADCCFG1_THRU_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG1_THRU_EN_POS)) /**< ADCCFG1_THRU_EN Mask */

#define MXC_F_MCR_ADCCFG1_AMP_EN_POS                   9 /**< ADCCFG1_AMP_EN Position */
#define MXC_F_MCR_ADCCFG1_AMP_EN                       ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG1_AMP_EN_POS)) /**< ADCCFG1_AMP_EN Mask */

#define MXC_F_MCR_ADCCFG1_AMP_RRI_EN_POS               10 /**< ADCCFG1_AMP_RRI_EN Position */
#define MXC_F_MCR_ADCCFG1_AMP_RRI_EN                   ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG1_AMP_RRI_EN_POS)) /**< ADCCFG1_AMP_RRI_EN Mask */

#define MXC_F_MCR_ADCCFG1_DIVSEL_POS                   11 /**< ADCCFG1_DIVSEL Position */
#define MXC_F_MCR_ADCCFG1_DIVSEL                       ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG1_DIVSEL_POS)) /**< ADCCFG1_DIVSEL Mask */

/**@} end of group MCR_ADCCFG1_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ADCCFG2 MCR_ADCCFG2
 * @brief    ADC Config Register 2.
 * @{
 */
#define MXC_F_MCR_ADCCFG2_VREFM_POS                    0 /**< ADCCFG2_VREFM Position */
#define MXC_F_MCR_ADCCFG2_VREFM                        ((uint32_t)(0x7FUL << MXC_F_MCR_ADCCFG2_VREFM_POS)) /**< ADCCFG2_VREFM Mask */

#define MXC_F_MCR_ADCCFG2_VREFP_POS                    8 /**< ADCCFG2_VREFP Position */
#define MXC_F_MCR_ADCCFG2_VREFP                        ((uint32_t)(0x7FUL << MXC_F_MCR_ADCCFG2_VREFP_POS)) /**< ADCCFG2_VREFP Mask */

#define MXC_F_MCR_ADCCFG2_IDRV_POS                     16 /**< ADCCFG2_IDRV Position */
#define MXC_F_MCR_ADCCFG2_IDRV                         ((uint32_t)(0xFUL << MXC_F_MCR_ADCCFG2_IDRV_POS)) /**< ADCCFG2_IDRV Mask */

#define MXC_F_MCR_ADCCFG2_VCM_POS                      20 /**< ADCCFG2_VCM Position */
#define MXC_F_MCR_ADCCFG2_VCM                          ((uint32_t)(0x3UL << MXC_F_MCR_ADCCFG2_VCM_POS)) /**< ADCCFG2_VCM Mask */

#define MXC_F_MCR_ADCCFG2_D_IBOOST_POS                 24 /**< ADCCFG2_D_IBOOST Position */
#define MXC_F_MCR_ADCCFG2_D_IBOOST                     ((uint32_t)(0x1UL << MXC_F_MCR_ADCCFG2_D_IBOOST_POS)) /**< ADCCFG2_D_IBOOST Mask */

/**@} end of group MCR_ADCCFG2_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_MCR_REGS_H_
