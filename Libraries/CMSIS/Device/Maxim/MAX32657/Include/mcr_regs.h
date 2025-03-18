/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mcr_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_

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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
    __IO uint32_t outen;                /**< <tt>\b 0x08:</tt> MCR OUTEN Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __R  uint32_t rsv_0x14_0x2f[7];
    __IO uint32_t bypass0;              /**< <tt>\b 0x30:</tt> MCR BYPASS0 Register */
    __IO uint32_t bypass1;              /**< <tt>\b 0x34:</tt> MCR BYPASS1 Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t data0;                /**< <tt>\b 0x40:</tt> MCR DATA0 Register */
    __IO uint32_t data1;                /**< <tt>\b 0x44:</tt> MCR DATA1 Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_RST                      ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_OUTEN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_BYPASS0                  ((uint32_t)0x00000030UL) /**< Offset from MCR Base Address: <tt> 0x0030</tt> */
#define MXC_R_MCR_BYPASS1                  ((uint32_t)0x00000034UL) /**< Offset from MCR Base Address: <tt> 0x0034</tt> */
#define MXC_R_MCR_DATA0                    ((uint32_t)0x00000040UL) /**< Offset from MCR Base Address: <tt> 0x0040</tt> */
#define MXC_R_MCR_DATA1                    ((uint32_t)0x00000044UL) /**< Offset from MCR Base Address: <tt> 0x0044</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RST MCR_RST
 * @brief    Reset Register.
 * @{
 */
#define MXC_F_MCR_RST_BOOST_POS                        0 /**< RST_BOOST Position */
#define MXC_F_MCR_RST_BOOST                            ((uint32_t)(0x1UL << MXC_F_MCR_RST_BOOST_POS)) /**< RST_BOOST Mask */

/**@} end of group MCR_RST_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_OUTEN MCR_OUTEN
 * @brief    Output Enable Register.
 * @{
 */
#define MXC_F_MCR_OUTEN_SQWOUT_EN_POS                  0 /**< OUTEN_SQWOUT_EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT_EN_POS)) /**< OUTEN_SQWOUT_EN Mask */

/**@} end of group MCR_OUTEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_CLKSEL_POS                      0 /**< CTRL_CLKSEL Position */
#define MXC_F_MCR_CTRL_CLKSEL                          ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_CLKSEL_POS)) /**< CTRL_CLKSEL Mask */
#define MXC_V_MCR_CTRL_CLKSEL_ERTCO                    ((uint32_t)0x0UL) /**< CTRL_CLKSEL_ERTCO Value */
#define MXC_S_MCR_CTRL_CLKSEL_ERTCO                    (MXC_V_MCR_CTRL_CLKSEL_ERTCO << MXC_F_MCR_CTRL_CLKSEL_POS) /**< CTRL_CLKSEL_ERTCO Setting */
#define MXC_V_MCR_CTRL_CLKSEL_INRO_DIV4                ((uint32_t)0x1UL) /**< CTRL_CLKSEL_INRO_DIV4 Value */
#define MXC_S_MCR_CTRL_CLKSEL_INRO_DIV4                (MXC_V_MCR_CTRL_CLKSEL_INRO_DIV4 << MXC_F_MCR_CTRL_CLKSEL_POS) /**< CTRL_CLKSEL_INRO_DIV4 Setting */
#define MXC_V_MCR_CTRL_CLKSEL_RTC_IN_DIV8              ((uint32_t)0x2UL) /**< CTRL_CLKSEL_RTC_IN_DIV8 Value */
#define MXC_S_MCR_CTRL_CLKSEL_RTC_IN_DIV8              (MXC_V_MCR_CTRL_CLKSEL_RTC_IN_DIV8 << MXC_F_MCR_CTRL_CLKSEL_POS) /**< CTRL_CLKSEL_RTC_IN_DIV8 Setting */

#define MXC_F_MCR_CTRL_ERTCO32K_EN_POS                 3 /**< CTRL_ERTCO32K_EN Position */
#define MXC_F_MCR_CTRL_ERTCO32K_EN                     ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO32K_EN_POS)) /**< CTRL_ERTCO32K_EN Mask */

#define MXC_F_MCR_CTRL_ERTCO_EN_POS                    5 /**< CTRL_ERTCO_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_EN                        ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_EN_POS)) /**< CTRL_ERTCO_EN Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_BYPASS0 MCR_BYPASS0
 * @brief    Flash Signature Check Bypass Register
 * @{
 */
#define MXC_V_MCR_BYPASS0                              ((uint32_t)0xB0CCF487UL) /**< MCR_BYPASS0 Flash Signature Check Bypass Value */
#define MXC_S_MCR_BYPASS0                              MXC_V_MCR_BYPASS0 /**< MCR_BYPASS0 Flash Signature Check Bypass Setting */

/**@} end of group MCR_BYPASS0_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_
