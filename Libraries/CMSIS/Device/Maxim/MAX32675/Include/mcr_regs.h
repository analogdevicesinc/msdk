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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_MCR_REGS_H_

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
 * @details     Miscellaneous Control Registers.
 */

/**
 * @ingroup mcr_registers
 * Structure type to access the MCR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0;
    __IO uint32_t rst;                  /**< <tt>\b 0x04:</tt> MCR RST Register */
    __R  uint32_t rsv_0x8_0xf[2];
    __IO uint32_t lppioctrl;            /**< <tt>\b 0x10:</tt> MCR LPPIOCTRL Register */
    __R  uint32_t rsv_0x14_0x23[4];
    __IO uint32_t clkdis;               /**< <tt>\b 0x24:</tt> MCR CLKDIS Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_RST                      ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_LPPIOCTRL                ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_CLKDIS                   ((uint32_t)0x00000024UL) /**< Offset from MCR Base Address: <tt> 0x0024</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RST MCR_RST
 * @brief    Reset control register 0.
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
 * @defgroup MCR_LPPIOCTRL MCR_LPPIOCTRL
 * @brief    Low-power peripheral IO control.
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
 * @defgroup MCR_CLKDIS MCR_CLKDIS
 * @brief    Peripheral clock control register.
 * @{
 */
#define MXC_F_MCR_CLKDIS_LPTMR0_POS                    0 /**< CLKDIS_LPTMR0 Position */
#define MXC_F_MCR_CLKDIS_LPTMR0                        ((uint32_t)(0x1UL << MXC_F_MCR_CLKDIS_LPTMR0_POS)) /**< CLKDIS_LPTMR0 Mask */

#define MXC_F_MCR_CLKDIS_LPTMR1_POS                    1 /**< CLKDIS_LPTMR1 Position */
#define MXC_F_MCR_CLKDIS_LPTMR1                        ((uint32_t)(0x1UL << MXC_F_MCR_CLKDIS_LPTMR1_POS)) /**< CLKDIS_LPTMR1 Mask */

#define MXC_F_MCR_CLKDIS_LPUART0_POS                   2 /**< CLKDIS_LPUART0 Position */
#define MXC_F_MCR_CLKDIS_LPUART0                       ((uint32_t)(0x1UL << MXC_F_MCR_CLKDIS_LPUART0_POS)) /**< CLKDIS_LPUART0 Mask */

/**@} end of group MCR_CLKDIS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_MCR_REGS_H_
