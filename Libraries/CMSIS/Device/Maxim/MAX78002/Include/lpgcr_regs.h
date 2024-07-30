/**
 * @file    lpgcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the LPGCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup lpgcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_LPGCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_LPGCR_REGS_H_

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
 * @ingroup     lpgcr
 * @defgroup    lpgcr_registers LPGCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the LPGCR Peripheral Module.
 * @details     Low Power Global Control.
 */

/**
 * @ingroup lpgcr_registers
 * Structure type to access the LPGCR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t rst;                  /**< <tt>\b 0x08:</tt> LPGCR RST Register */
    __IO uint32_t pclkdis;              /**< <tt>\b 0x0C:</tt> LPGCR PCLKDIS Register */
} mxc_lpgcr_regs_t;

/* Register offsets for module LPGCR */
/**
 * @ingroup    lpgcr_registers
 * @defgroup   LPGCR_Register_Offsets Register Offsets
 * @brief      LPGCR Peripheral Register Offsets from the LPGCR Base Peripheral Address.
 * @{
 */
#define MXC_R_LPGCR_RST                    ((uint32_t)0x00000008UL) /**< Offset from LPGCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_LPGCR_PCLKDIS                ((uint32_t)0x0000000CUL) /**< Offset from LPGCR Base Address: <tt> 0x000C</tt> */
/**@} end of group lpgcr_registers */

/**
 * @ingroup  lpgcr_registers
 * @defgroup LPGCR_RST LPGCR_RST
 * @brief    Low Power Reset Register.
 * @{
 */
#define MXC_F_LPGCR_RST_GPIO2_POS                      0 /**< RST_GPIO2 Position */
#define MXC_F_LPGCR_RST_GPIO2                          ((uint32_t)(0x1UL << MXC_F_LPGCR_RST_GPIO2_POS)) /**< RST_GPIO2 Mask */

#define MXC_F_LPGCR_RST_WDT1_POS                       1 /**< RST_WDT1 Position */
#define MXC_F_LPGCR_RST_WDT1                           ((uint32_t)(0x1UL << MXC_F_LPGCR_RST_WDT1_POS)) /**< RST_WDT1 Mask */

#define MXC_F_LPGCR_RST_TMR4_POS                       2 /**< RST_TMR4 Position */
#define MXC_F_LPGCR_RST_TMR4                           ((uint32_t)(0x1UL << MXC_F_LPGCR_RST_TMR4_POS)) /**< RST_TMR4 Mask */

#define MXC_F_LPGCR_RST_TMR5_POS                       3 /**< RST_TMR5 Position */
#define MXC_F_LPGCR_RST_TMR5                           ((uint32_t)(0x1UL << MXC_F_LPGCR_RST_TMR5_POS)) /**< RST_TMR5 Mask */

#define MXC_F_LPGCR_RST_UART3_POS                      4 /**< RST_UART3 Position */
#define MXC_F_LPGCR_RST_UART3                          ((uint32_t)(0x1UL << MXC_F_LPGCR_RST_UART3_POS)) /**< RST_UART3 Mask */

#define MXC_F_LPGCR_RST_LPCOMP_POS                     6 /**< RST_LPCOMP Position */
#define MXC_F_LPGCR_RST_LPCOMP                         ((uint32_t)(0x1UL << MXC_F_LPGCR_RST_LPCOMP_POS)) /**< RST_LPCOMP Mask */

/**@} end of group LPGCR_RST_Register */

/**
 * @ingroup  lpgcr_registers
 * @defgroup LPGCR_PCLKDIS LPGCR_PCLKDIS
 * @brief    Low Power Peripheral Clock Disable Register.
 * @{
 */
#define MXC_F_LPGCR_PCLKDIS_GPIO2_POS                  0 /**< PCLKDIS_GPIO2 Position */
#define MXC_F_LPGCR_PCLKDIS_GPIO2                      ((uint32_t)(0x1UL << MXC_F_LPGCR_PCLKDIS_GPIO2_POS)) /**< PCLKDIS_GPIO2 Mask */

#define MXC_F_LPGCR_PCLKDIS_WDT1_POS                   1 /**< PCLKDIS_WDT1 Position */
#define MXC_F_LPGCR_PCLKDIS_WDT1                       ((uint32_t)(0x1UL << MXC_F_LPGCR_PCLKDIS_WDT1_POS)) /**< PCLKDIS_WDT1 Mask */

#define MXC_F_LPGCR_PCLKDIS_TMR4_POS                   2 /**< PCLKDIS_TMR4 Position */
#define MXC_F_LPGCR_PCLKDIS_TMR4                       ((uint32_t)(0x1UL << MXC_F_LPGCR_PCLKDIS_TMR4_POS)) /**< PCLKDIS_TMR4 Mask */

#define MXC_F_LPGCR_PCLKDIS_TMR5_POS                   3 /**< PCLKDIS_TMR5 Position */
#define MXC_F_LPGCR_PCLKDIS_TMR5                       ((uint32_t)(0x1UL << MXC_F_LPGCR_PCLKDIS_TMR5_POS)) /**< PCLKDIS_TMR5 Mask */

#define MXC_F_LPGCR_PCLKDIS_UART3_POS                  4 /**< PCLKDIS_UART3 Position */
#define MXC_F_LPGCR_PCLKDIS_UART3                      ((uint32_t)(0x1UL << MXC_F_LPGCR_PCLKDIS_UART3_POS)) /**< PCLKDIS_UART3 Mask */

#define MXC_F_LPGCR_PCLKDIS_LPCOMP_POS                 6 /**< PCLKDIS_LPCOMP Position */
#define MXC_F_LPGCR_PCLKDIS_LPCOMP                     ((uint32_t)(0x1UL << MXC_F_LPGCR_PCLKDIS_LPCOMP_POS)) /**< PCLKDIS_LPCOMP Mask */

/**@} end of group LPGCR_PCLKDIS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_LPGCR_REGS_H_
