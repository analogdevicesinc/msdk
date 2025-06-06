/**
 * @file    wdt_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the WDT Peripheral Module.
 * @note    This file is @generated.
 * @ingroup wdt_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_WDT_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_WDT_REGS_H_

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
 * @ingroup     wdt
 * @defgroup    wdt_registers WDT_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the WDT Peripheral Module.
 * @details     Windowed Watchdog Timer
 */

/**
 * @ingroup wdt_registers
 * Structure type to access the WDT Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> WDT CTRL Register */
    __O  uint32_t rst;                  /**< <tt>\b 0x04:</tt> WDT RST Register */
    __IO uint32_t clksel;               /**< <tt>\b 0x08:</tt> WDT CLKSEL Register */
    __I  uint32_t cnt;                  /**< <tt>\b 0x0C:</tt> WDT CNT Register */
} mxc_wdt_regs_t;

/* Register offsets for module WDT */
/**
 * @ingroup    wdt_registers
 * @defgroup   WDT_Register_Offsets Register Offsets
 * @brief      WDT Peripheral Register Offsets from the WDT Base Peripheral Address.
 * @{
 */
#define MXC_R_WDT_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from WDT Base Address: <tt> 0x0000</tt> */
#define MXC_R_WDT_RST                      ((uint32_t)0x00000004UL) /**< Offset from WDT Base Address: <tt> 0x0004</tt> */
#define MXC_R_WDT_CLKSEL                   ((uint32_t)0x00000008UL) /**< Offset from WDT Base Address: <tt> 0x0008</tt> */
#define MXC_R_WDT_CNT                      ((uint32_t)0x0000000CUL) /**< Offset from WDT Base Address: <tt> 0x000C</tt> */
/**@} end of group wdt_registers */

/**
 * @ingroup  wdt_registers
 * @defgroup WDT_CTRL WDT_CTRL
 * @brief    Watchdog Timer Control Register.
 * @{
 */
#define MXC_F_WDT_CTRL_INT_LATE_VAL_POS                0 /**< CTRL_INT_LATE_VAL Position */
#define MXC_F_WDT_CTRL_INT_LATE_VAL                    ((uint32_t)(0xFUL << MXC_F_WDT_CTRL_INT_LATE_VAL_POS)) /**< CTRL_INT_LATE_VAL Mask */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW31          ((uint32_t)0x0UL) /**< CTRL_INT_LATE_VAL_WDT2POW31 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW31          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW31 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW31 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW30          ((uint32_t)0x1UL) /**< CTRL_INT_LATE_VAL_WDT2POW30 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW30          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW30 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW30 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW29          ((uint32_t)0x2UL) /**< CTRL_INT_LATE_VAL_WDT2POW29 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW29          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW29 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW29 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW28          ((uint32_t)0x3UL) /**< CTRL_INT_LATE_VAL_WDT2POW28 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW28          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW28 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW28 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW27          ((uint32_t)0x4UL) /**< CTRL_INT_LATE_VAL_WDT2POW27 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW27          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW27 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW27 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW26          ((uint32_t)0x5UL) /**< CTRL_INT_LATE_VAL_WDT2POW26 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW26          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW26 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW26 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW25          ((uint32_t)0x6UL) /**< CTRL_INT_LATE_VAL_WDT2POW25 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW25          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW25 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW25 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW24          ((uint32_t)0x7UL) /**< CTRL_INT_LATE_VAL_WDT2POW24 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW24          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW24 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW24 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW23          ((uint32_t)0x8UL) /**< CTRL_INT_LATE_VAL_WDT2POW23 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW23          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW23 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW23 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW22          ((uint32_t)0x9UL) /**< CTRL_INT_LATE_VAL_WDT2POW22 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW22          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW22 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW22 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW21          ((uint32_t)0xAUL) /**< CTRL_INT_LATE_VAL_WDT2POW21 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW21          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW21 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW21 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW20          ((uint32_t)0xBUL) /**< CTRL_INT_LATE_VAL_WDT2POW20 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW20          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW20 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW20 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW19          ((uint32_t)0xCUL) /**< CTRL_INT_LATE_VAL_WDT2POW19 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW19          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW19 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW19 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW18          ((uint32_t)0xDUL) /**< CTRL_INT_LATE_VAL_WDT2POW18 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW18          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW18 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW18 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW17          ((uint32_t)0xEUL) /**< CTRL_INT_LATE_VAL_WDT2POW17 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW17          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW17 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW17 Setting */
#define MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW16          ((uint32_t)0xFUL) /**< CTRL_INT_LATE_VAL_WDT2POW16 Value */
#define MXC_S_WDT_CTRL_INT_LATE_VAL_WDT2POW16          (MXC_V_WDT_CTRL_INT_LATE_VAL_WDT2POW16 << MXC_F_WDT_CTRL_INT_LATE_VAL_POS) /**< CTRL_INT_LATE_VAL_WDT2POW16 Setting */

#define MXC_F_WDT_CTRL_RST_LATE_VAL_POS                4 /**< CTRL_RST_LATE_VAL Position */
#define MXC_F_WDT_CTRL_RST_LATE_VAL                    ((uint32_t)(0xFUL << MXC_F_WDT_CTRL_RST_LATE_VAL_POS)) /**< CTRL_RST_LATE_VAL Mask */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW31          ((uint32_t)0x0UL) /**< CTRL_RST_LATE_VAL_WDT2POW31 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW31          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW31 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW31 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW30          ((uint32_t)0x1UL) /**< CTRL_RST_LATE_VAL_WDT2POW30 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW30          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW30 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW30 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW29          ((uint32_t)0x2UL) /**< CTRL_RST_LATE_VAL_WDT2POW29 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW29          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW29 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW29 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW28          ((uint32_t)0x3UL) /**< CTRL_RST_LATE_VAL_WDT2POW28 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW28          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW28 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW28 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW27          ((uint32_t)0x4UL) /**< CTRL_RST_LATE_VAL_WDT2POW27 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW27          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW27 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW27 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW26          ((uint32_t)0x5UL) /**< CTRL_RST_LATE_VAL_WDT2POW26 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW26          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW26 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW26 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW25          ((uint32_t)0x6UL) /**< CTRL_RST_LATE_VAL_WDT2POW25 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW25          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW25 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW25 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW24          ((uint32_t)0x7UL) /**< CTRL_RST_LATE_VAL_WDT2POW24 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW24          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW24 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW24 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW23          ((uint32_t)0x8UL) /**< CTRL_RST_LATE_VAL_WDT2POW23 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW23          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW23 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW23 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW22          ((uint32_t)0x9UL) /**< CTRL_RST_LATE_VAL_WDT2POW22 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW22          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW22 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW22 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW21          ((uint32_t)0xAUL) /**< CTRL_RST_LATE_VAL_WDT2POW21 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW21          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW21 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW21 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW20          ((uint32_t)0xBUL) /**< CTRL_RST_LATE_VAL_WDT2POW20 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW20          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW20 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW20 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW19          ((uint32_t)0xCUL) /**< CTRL_RST_LATE_VAL_WDT2POW19 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW19          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW19 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW19 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW18          ((uint32_t)0xDUL) /**< CTRL_RST_LATE_VAL_WDT2POW18 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW18          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW18 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW18 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW17          ((uint32_t)0xEUL) /**< CTRL_RST_LATE_VAL_WDT2POW17 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW17          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW17 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW17 Setting */
#define MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW16          ((uint32_t)0xFUL) /**< CTRL_RST_LATE_VAL_WDT2POW16 Value */
#define MXC_S_WDT_CTRL_RST_LATE_VAL_WDT2POW16          (MXC_V_WDT_CTRL_RST_LATE_VAL_WDT2POW16 << MXC_F_WDT_CTRL_RST_LATE_VAL_POS) /**< CTRL_RST_LATE_VAL_WDT2POW16 Setting */

#define MXC_F_WDT_CTRL_EN_POS                          8 /**< CTRL_EN Position */
#define MXC_F_WDT_CTRL_EN                              ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_WDT_CTRL_INT_LATE_POS                    9 /**< CTRL_INT_LATE Position */
#define MXC_F_WDT_CTRL_INT_LATE                        ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_INT_LATE_POS)) /**< CTRL_INT_LATE Mask */

#define MXC_F_WDT_CTRL_WDT_INT_EN_POS                  10 /**< CTRL_WDT_INT_EN Position */
#define MXC_F_WDT_CTRL_WDT_INT_EN                      ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_WDT_INT_EN_POS)) /**< CTRL_WDT_INT_EN Mask */

#define MXC_F_WDT_CTRL_WDT_RST_EN_POS                  11 /**< CTRL_WDT_RST_EN Position */
#define MXC_F_WDT_CTRL_WDT_RST_EN                      ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_WDT_RST_EN_POS)) /**< CTRL_WDT_RST_EN Mask */

#define MXC_F_WDT_CTRL_INT_EARLY_POS                   12 /**< CTRL_INT_EARLY Position */
#define MXC_F_WDT_CTRL_INT_EARLY                       ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_INT_EARLY_POS)) /**< CTRL_INT_EARLY Mask */

#define MXC_F_WDT_CTRL_INT_EARLY_VAL_POS               16 /**< CTRL_INT_EARLY_VAL Position */
#define MXC_F_WDT_CTRL_INT_EARLY_VAL                   ((uint32_t)(0xFUL << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS)) /**< CTRL_INT_EARLY_VAL Mask */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW31         ((uint32_t)0x0UL) /**< CTRL_INT_EARLY_VAL_WDT2POW31 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW31         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW31 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW31 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW30         ((uint32_t)0x1UL) /**< CTRL_INT_EARLY_VAL_WDT2POW30 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW30         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW30 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW30 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW29         ((uint32_t)0x2UL) /**< CTRL_INT_EARLY_VAL_WDT2POW29 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW29         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW29 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW29 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW28         ((uint32_t)0x3UL) /**< CTRL_INT_EARLY_VAL_WDT2POW28 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW28         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW28 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW28 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW27         ((uint32_t)0x4UL) /**< CTRL_INT_EARLY_VAL_WDT2POW27 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW27         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW27 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW27 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW26         ((uint32_t)0x5UL) /**< CTRL_INT_EARLY_VAL_WDT2POW26 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW26         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW26 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW26 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW25         ((uint32_t)0x6UL) /**< CTRL_INT_EARLY_VAL_WDT2POW25 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW25         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW25 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW25 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW24         ((uint32_t)0x7UL) /**< CTRL_INT_EARLY_VAL_WDT2POW24 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW24         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW24 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW24 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW23         ((uint32_t)0x8UL) /**< CTRL_INT_EARLY_VAL_WDT2POW23 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW23         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW23 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW23 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW22         ((uint32_t)0x9UL) /**< CTRL_INT_EARLY_VAL_WDT2POW22 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW22         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW22 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW22 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW21         ((uint32_t)0xAUL) /**< CTRL_INT_EARLY_VAL_WDT2POW21 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW21         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW21 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW21 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW20         ((uint32_t)0xBUL) /**< CTRL_INT_EARLY_VAL_WDT2POW20 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW20         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW20 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW20 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW19         ((uint32_t)0xCUL) /**< CTRL_INT_EARLY_VAL_WDT2POW19 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW19         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW19 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW19 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW18         ((uint32_t)0xDUL) /**< CTRL_INT_EARLY_VAL_WDT2POW18 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW18         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW18 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW18 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW17         ((uint32_t)0xEUL) /**< CTRL_INT_EARLY_VAL_WDT2POW17 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW17         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW17 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW17 Setting */
#define MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW16         ((uint32_t)0xFUL) /**< CTRL_INT_EARLY_VAL_WDT2POW16 Value */
#define MXC_S_WDT_CTRL_INT_EARLY_VAL_WDT2POW16         (MXC_V_WDT_CTRL_INT_EARLY_VAL_WDT2POW16 << MXC_F_WDT_CTRL_INT_EARLY_VAL_POS) /**< CTRL_INT_EARLY_VAL_WDT2POW16 Setting */

#define MXC_F_WDT_CTRL_RST_EARLY_VAL_POS               20 /**< CTRL_RST_EARLY_VAL Position */
#define MXC_F_WDT_CTRL_RST_EARLY_VAL                   ((uint32_t)(0xFUL << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS)) /**< CTRL_RST_EARLY_VAL Mask */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW31         ((uint32_t)0x0UL) /**< CTRL_RST_EARLY_VAL_WDT2POW31 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW31         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW31 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW31 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW30         ((uint32_t)0x1UL) /**< CTRL_RST_EARLY_VAL_WDT2POW30 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW30         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW30 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW30 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW29         ((uint32_t)0x2UL) /**< CTRL_RST_EARLY_VAL_WDT2POW29 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW29         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW29 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW29 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW28         ((uint32_t)0x3UL) /**< CTRL_RST_EARLY_VAL_WDT2POW28 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW28         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW28 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW28 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW27         ((uint32_t)0x4UL) /**< CTRL_RST_EARLY_VAL_WDT2POW27 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW27         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW27 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW27 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW26         ((uint32_t)0x5UL) /**< CTRL_RST_EARLY_VAL_WDT2POW26 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW26         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW26 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW26 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW25         ((uint32_t)0x6UL) /**< CTRL_RST_EARLY_VAL_WDT2POW25 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW25         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW25 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW25 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW24         ((uint32_t)0x7UL) /**< CTRL_RST_EARLY_VAL_WDT2POW24 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW24         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW24 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW24 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW23         ((uint32_t)0x8UL) /**< CTRL_RST_EARLY_VAL_WDT2POW23 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW23         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW23 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW23 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW22         ((uint32_t)0x9UL) /**< CTRL_RST_EARLY_VAL_WDT2POW22 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW22         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW22 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW22 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW21         ((uint32_t)0xAUL) /**< CTRL_RST_EARLY_VAL_WDT2POW21 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW21         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW21 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW21 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW20         ((uint32_t)0xBUL) /**< CTRL_RST_EARLY_VAL_WDT2POW20 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW20         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW20 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW20 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW19         ((uint32_t)0xCUL) /**< CTRL_RST_EARLY_VAL_WDT2POW19 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW19         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW19 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW19 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW18         ((uint32_t)0xDUL) /**< CTRL_RST_EARLY_VAL_WDT2POW18 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW18         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW18 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW18 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW17         ((uint32_t)0xEUL) /**< CTRL_RST_EARLY_VAL_WDT2POW17 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW17         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW17 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW17 Setting */
#define MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW16         ((uint32_t)0xFUL) /**< CTRL_RST_EARLY_VAL_WDT2POW16 Value */
#define MXC_S_WDT_CTRL_RST_EARLY_VAL_WDT2POW16         (MXC_V_WDT_CTRL_RST_EARLY_VAL_WDT2POW16 << MXC_F_WDT_CTRL_RST_EARLY_VAL_POS) /**< CTRL_RST_EARLY_VAL_WDT2POW16 Setting */

#define MXC_F_WDT_CTRL_CLKRDY_IE_POS                   27 /**< CTRL_CLKRDY_IE Position */
#define MXC_F_WDT_CTRL_CLKRDY_IE                       ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_CLKRDY_IE_POS)) /**< CTRL_CLKRDY_IE Mask */

#define MXC_F_WDT_CTRL_CLKRDY_POS                      28 /**< CTRL_CLKRDY Position */
#define MXC_F_WDT_CTRL_CLKRDY                          ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_CLKRDY_POS)) /**< CTRL_CLKRDY Mask */

#define MXC_F_WDT_CTRL_WIN_EN_POS                      29 /**< CTRL_WIN_EN Position */
#define MXC_F_WDT_CTRL_WIN_EN                          ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_WIN_EN_POS)) /**< CTRL_WIN_EN Mask */

#define MXC_F_WDT_CTRL_RST_EARLY_POS                   30 /**< CTRL_RST_EARLY Position */
#define MXC_F_WDT_CTRL_RST_EARLY                       ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_RST_EARLY_POS)) /**< CTRL_RST_EARLY Mask */

#define MXC_F_WDT_CTRL_RST_LATE_POS                    31 /**< CTRL_RST_LATE Position */
#define MXC_F_WDT_CTRL_RST_LATE                        ((uint32_t)(0x1UL << MXC_F_WDT_CTRL_RST_LATE_POS)) /**< CTRL_RST_LATE Mask */

/**@} end of group WDT_CTRL_Register */

/**
 * @ingroup  wdt_registers
 * @defgroup WDT_RST WDT_RST
 * @brief    Windowed Watchdog Timer Reset Register.
 * @{
 */
#define MXC_F_WDT_RST_RESET_POS                        0 /**< RST_RESET Position */
#define MXC_F_WDT_RST_RESET                            ((uint32_t)(0xFFUL << MXC_F_WDT_RST_RESET_POS)) /**< RST_RESET Mask */
#define MXC_V_WDT_RST_RESET_SEQ0                       ((uint32_t)0xA5UL) /**< RST_RESET_SEQ0 Value */
#define MXC_S_WDT_RST_RESET_SEQ0                       (MXC_V_WDT_RST_RESET_SEQ0 << MXC_F_WDT_RST_RESET_POS) /**< RST_RESET_SEQ0 Setting */
#define MXC_V_WDT_RST_RESET_SEQ1                       ((uint32_t)0x5AUL) /**< RST_RESET_SEQ1 Value */
#define MXC_S_WDT_RST_RESET_SEQ1                       (MXC_V_WDT_RST_RESET_SEQ1 << MXC_F_WDT_RST_RESET_POS) /**< RST_RESET_SEQ1 Setting */

/**@} end of group WDT_RST_Register */

/**
 * @ingroup  wdt_registers
 * @defgroup WDT_CLKSEL WDT_CLKSEL
 * @brief    Windowed Watchdog Timer Clock Select Register.
 * @{
 */
#define MXC_F_WDT_CLKSEL_SOURCE_POS                    0 /**< CLKSEL_SOURCE Position */
#define MXC_F_WDT_CLKSEL_SOURCE                        ((uint32_t)(0x7UL << MXC_F_WDT_CLKSEL_SOURCE_POS)) /**< CLKSEL_SOURCE Mask */

/**@} end of group WDT_CLKSEL_Register */

/**
 * @ingroup  wdt_registers
 * @defgroup WDT_CNT WDT_CNT
 * @brief    Windowed Watchdog Timer Count Register.
 * @{
 */
#define MXC_F_WDT_CNT_COUNT_POS                        0 /**< CNT_COUNT Position */
#define MXC_F_WDT_CNT_COUNT                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_WDT_CNT_COUNT_POS)) /**< CNT_COUNT Mask */

/**@} end of group WDT_CNT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_WDT_REGS_H_
