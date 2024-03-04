/**
 * @file    spixfc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixfc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXFC_REGS_H_

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
 * @ingroup     spixfc
 * @defgroup    spixfc_registers SPIXFC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFC Peripheral Module.
 * @details     SPI XiP Flash Configuration Controller
 */

/**
 * @ingroup spixfc_registers
 * Structure type to access the SPIXFC Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x00:</tt> SPIXFC CFG Register */
    __IO uint32_t ss_pol;               /**< <tt>\b 0x04:</tt> SPIXFC SS_POL Register */
    __IO uint32_t gen_ctrl;             /**< <tt>\b 0x08:</tt> SPIXFC GEN_CTRL Register */
    __IO uint32_t fifo_ctrl;            /**< <tt>\b 0x0C:</tt> SPIXFC FIFO_CTRL Register */
    __IO uint32_t sp_ctrl;              /**< <tt>\b 0x10:</tt> SPIXFC SP_CTRL Register */
    __IO uint32_t int_fl;               /**< <tt>\b 0x14:</tt> SPIXFC INT_FL Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x18:</tt> SPIXFC INT_EN Register */
} mxc_spixfc_regs_t;

/* Register offsets for module SPIXFC */
/**
 * @ingroup    spixfc_registers
 * @defgroup   SPIXFC_Register_Offsets Register Offsets
 * @brief      SPIXFC Peripheral Register Offsets from the SPIXFC Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXFC_CFG                   ((uint32_t)0x00000000UL) /**< Offset from SPIXFC Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFC_SS_POL                ((uint32_t)0x00000004UL) /**< Offset from SPIXFC Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXFC_GEN_CTRL              ((uint32_t)0x00000008UL) /**< Offset from SPIXFC Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXFC_FIFO_CTRL             ((uint32_t)0x0000000CUL) /**< Offset from SPIXFC Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXFC_SP_CTRL               ((uint32_t)0x00000010UL) /**< Offset from SPIXFC Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXFC_INT_FL                ((uint32_t)0x00000014UL) /**< Offset from SPIXFC Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIXFC_INT_EN                ((uint32_t)0x00000018UL) /**< Offset from SPIXFC Base Address: <tt> 0x0018</tt> */
/**@} end of group spixfc_registers */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_CFG SPIXFC_CFG
 * @brief    Configuration Register.
 * @{
 */
#define MXC_F_SPIXFC_CFG_SSEL_POS                      0 /**< CFG_SSEL Position */
#define MXC_F_SPIXFC_CFG_SSEL                          ((uint32_t)(0x7UL << MXC_F_SPIXFC_CFG_SSEL_POS)) /**< CFG_SSEL Mask */
#define MXC_V_SPIXFC_CFG_SSEL_SLAVE0                   ((uint32_t)0x0UL) /**< CFG_SSEL_SLAVE0 Value */
#define MXC_S_SPIXFC_CFG_SSEL_SLAVE0                   (MXC_V_SPIXFC_CFG_SSEL_SLAVE0 << MXC_F_SPIXFC_CFG_SSEL_POS) /**< CFG_SSEL_SLAVE0 Setting */

#define MXC_F_SPIXFC_CFG_MODE_POS                      4 /**< CFG_MODE Position */
#define MXC_F_SPIXFC_CFG_MODE                          ((uint32_t)(0x3UL << MXC_F_SPIXFC_CFG_MODE_POS)) /**< CFG_MODE Mask */
#define MXC_V_SPIXFC_CFG_MODE_MODE0                    ((uint32_t)0x0UL) /**< CFG_MODE_MODE0 Value */
#define MXC_S_SPIXFC_CFG_MODE_MODE0                    (MXC_V_SPIXFC_CFG_MODE_MODE0 << MXC_F_SPIXFC_CFG_MODE_POS) /**< CFG_MODE_MODE0 Setting */
#define MXC_V_SPIXFC_CFG_MODE_MODE3                    ((uint32_t)0x3UL) /**< CFG_MODE_MODE3 Value */
#define MXC_S_SPIXFC_CFG_MODE_MODE3                    (MXC_V_SPIXFC_CFG_MODE_MODE3 << MXC_F_SPIXFC_CFG_MODE_POS) /**< CFG_MODE_MODE3 Setting */

#define MXC_F_SPIXFC_CFG_PGSZ_POS                      6 /**< CFG_PGSZ Position */
#define MXC_F_SPIXFC_CFG_PGSZ                          ((uint32_t)(0x3UL << MXC_F_SPIXFC_CFG_PGSZ_POS)) /**< CFG_PGSZ Mask */
#define MXC_V_SPIXFC_CFG_PGSZ_4BYTES                   ((uint32_t)0x0UL) /**< CFG_PGSZ_4BYTES Value */
#define MXC_S_SPIXFC_CFG_PGSZ_4BYTES                   (MXC_V_SPIXFC_CFG_PGSZ_4BYTES << MXC_F_SPIXFC_CFG_PGSZ_POS) /**< CFG_PGSZ_4BYTES Setting */
#define MXC_V_SPIXFC_CFG_PGSZ_8BYTES                   ((uint32_t)0x1UL) /**< CFG_PGSZ_8BYTES Value */
#define MXC_S_SPIXFC_CFG_PGSZ_8BYTES                   (MXC_V_SPIXFC_CFG_PGSZ_8BYTES << MXC_F_SPIXFC_CFG_PGSZ_POS) /**< CFG_PGSZ_8BYTES Setting */
#define MXC_V_SPIXFC_CFG_PGSZ_16BYTES                  ((uint32_t)0x2UL) /**< CFG_PGSZ_16BYTES Value */
#define MXC_S_SPIXFC_CFG_PGSZ_16BYTES                  (MXC_V_SPIXFC_CFG_PGSZ_16BYTES << MXC_F_SPIXFC_CFG_PGSZ_POS) /**< CFG_PGSZ_16BYTES Setting */
#define MXC_V_SPIXFC_CFG_PGSZ_32BYTES                  ((uint32_t)0x3UL) /**< CFG_PGSZ_32BYTES Value */
#define MXC_S_SPIXFC_CFG_PGSZ_32BYTES                  (MXC_V_SPIXFC_CFG_PGSZ_32BYTES << MXC_F_SPIXFC_CFG_PGSZ_POS) /**< CFG_PGSZ_32BYTES Setting */

#define MXC_F_SPIXFC_CFG_HICLK_POS                     8 /**< CFG_HICLK Position */
#define MXC_F_SPIXFC_CFG_HICLK                         ((uint32_t)(0xFUL << MXC_F_SPIXFC_CFG_HICLK_POS)) /**< CFG_HICLK Mask */
#define MXC_V_SPIXFC_CFG_HICLK_16CLK                   ((uint32_t)0x0UL) /**< CFG_HICLK_16CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_16CLK                   (MXC_V_SPIXFC_CFG_HICLK_16CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_16CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_1CLK                    ((uint32_t)0x1UL) /**< CFG_HICLK_1CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_1CLK                    (MXC_V_SPIXFC_CFG_HICLK_1CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_1CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_2CLK                    ((uint32_t)0x2UL) /**< CFG_HICLK_2CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_2CLK                    (MXC_V_SPIXFC_CFG_HICLK_2CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_2CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_3CLK                    ((uint32_t)0x3UL) /**< CFG_HICLK_3CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_3CLK                    (MXC_V_SPIXFC_CFG_HICLK_3CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_3CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_4CLK                    ((uint32_t)0x4UL) /**< CFG_HICLK_4CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_4CLK                    (MXC_V_SPIXFC_CFG_HICLK_4CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_4CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_5CLK                    ((uint32_t)0x5UL) /**< CFG_HICLK_5CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_5CLK                    (MXC_V_SPIXFC_CFG_HICLK_5CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_5CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_6CLK                    ((uint32_t)0x6UL) /**< CFG_HICLK_6CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_6CLK                    (MXC_V_SPIXFC_CFG_HICLK_6CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_6CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_7CLK                    ((uint32_t)0x7UL) /**< CFG_HICLK_7CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_7CLK                    (MXC_V_SPIXFC_CFG_HICLK_7CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_7CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_8CLK                    ((uint32_t)0x8UL) /**< CFG_HICLK_8CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_8CLK                    (MXC_V_SPIXFC_CFG_HICLK_8CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_8CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_9CLK                    ((uint32_t)0x9UL) /**< CFG_HICLK_9CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_9CLK                    (MXC_V_SPIXFC_CFG_HICLK_9CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_9CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_10CLK                   ((uint32_t)0xAUL) /**< CFG_HICLK_10CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_10CLK                   (MXC_V_SPIXFC_CFG_HICLK_10CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_10CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_11CLK                   ((uint32_t)0xBUL) /**< CFG_HICLK_11CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_11CLK                   (MXC_V_SPIXFC_CFG_HICLK_11CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_11CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_12CLK                   ((uint32_t)0xCUL) /**< CFG_HICLK_12CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_12CLK                   (MXC_V_SPIXFC_CFG_HICLK_12CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_12CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_13CLK                   ((uint32_t)0xDUL) /**< CFG_HICLK_13CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_13CLK                   (MXC_V_SPIXFC_CFG_HICLK_13CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_13CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_14CLK                   ((uint32_t)0xEUL) /**< CFG_HICLK_14CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_14CLK                   (MXC_V_SPIXFC_CFG_HICLK_14CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_14CLK Setting */
#define MXC_V_SPIXFC_CFG_HICLK_15CLK                   ((uint32_t)0xFUL) /**< CFG_HICLK_15CLK Value */
#define MXC_S_SPIXFC_CFG_HICLK_15CLK                   (MXC_V_SPIXFC_CFG_HICLK_15CLK << MXC_F_SPIXFC_CFG_HICLK_POS) /**< CFG_HICLK_15CLK Setting */

#define MXC_F_SPIXFC_CFG_LOCLK_POS                     12 /**< CFG_LOCLK Position */
#define MXC_F_SPIXFC_CFG_LOCLK                         ((uint32_t)(0xFUL << MXC_F_SPIXFC_CFG_LOCLK_POS)) /**< CFG_LOCLK Mask */
#define MXC_V_SPIXFC_CFG_LOCLK_16CLK                   ((uint32_t)0x0UL) /**< CFG_LOCLK_16CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_16CLK                   (MXC_V_SPIXFC_CFG_LOCLK_16CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_16CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_1CLK                    ((uint32_t)0x1UL) /**< CFG_LOCLK_1CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_1CLK                    (MXC_V_SPIXFC_CFG_LOCLK_1CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_1CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_2CLK                    ((uint32_t)0x2UL) /**< CFG_LOCLK_2CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_2CLK                    (MXC_V_SPIXFC_CFG_LOCLK_2CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_2CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_3CLK                    ((uint32_t)0x3UL) /**< CFG_LOCLK_3CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_3CLK                    (MXC_V_SPIXFC_CFG_LOCLK_3CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_3CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_4CLK                    ((uint32_t)0x4UL) /**< CFG_LOCLK_4CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_4CLK                    (MXC_V_SPIXFC_CFG_LOCLK_4CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_4CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_5CLK                    ((uint32_t)0x5UL) /**< CFG_LOCLK_5CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_5CLK                    (MXC_V_SPIXFC_CFG_LOCLK_5CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_5CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_6CLK                    ((uint32_t)0x6UL) /**< CFG_LOCLK_6CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_6CLK                    (MXC_V_SPIXFC_CFG_LOCLK_6CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_6CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_7CLK                    ((uint32_t)0x7UL) /**< CFG_LOCLK_7CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_7CLK                    (MXC_V_SPIXFC_CFG_LOCLK_7CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_7CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_8CLK                    ((uint32_t)0x8UL) /**< CFG_LOCLK_8CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_8CLK                    (MXC_V_SPIXFC_CFG_LOCLK_8CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_8CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_9CLK                    ((uint32_t)0x9UL) /**< CFG_LOCLK_9CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_9CLK                    (MXC_V_SPIXFC_CFG_LOCLK_9CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_9CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_10CLK                   ((uint32_t)0xAUL) /**< CFG_LOCLK_10CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_10CLK                   (MXC_V_SPIXFC_CFG_LOCLK_10CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_10CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_11CLK                   ((uint32_t)0xBUL) /**< CFG_LOCLK_11CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_11CLK                   (MXC_V_SPIXFC_CFG_LOCLK_11CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_11CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_12CLK                   ((uint32_t)0xCUL) /**< CFG_LOCLK_12CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_12CLK                   (MXC_V_SPIXFC_CFG_LOCLK_12CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_12CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_13CLK                   ((uint32_t)0xDUL) /**< CFG_LOCLK_13CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_13CLK                   (MXC_V_SPIXFC_CFG_LOCLK_13CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_13CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_14CLK                   ((uint32_t)0xEUL) /**< CFG_LOCLK_14CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_14CLK                   (MXC_V_SPIXFC_CFG_LOCLK_14CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_14CLK Setting */
#define MXC_V_SPIXFC_CFG_LOCLK_15CLK                   ((uint32_t)0xFUL) /**< CFG_LOCLK_15CLK Value */
#define MXC_S_SPIXFC_CFG_LOCLK_15CLK                   (MXC_V_SPIXFC_CFG_LOCLK_15CLK << MXC_F_SPIXFC_CFG_LOCLK_POS) /**< CFG_LOCLK_15CLK Setting */

#define MXC_F_SPIXFC_CFG_SSACT_POS                     16 /**< CFG_SSACT Position */
#define MXC_F_SPIXFC_CFG_SSACT                         ((uint32_t)(0x3UL << MXC_F_SPIXFC_CFG_SSACT_POS)) /**< CFG_SSACT Mask */
#define MXC_V_SPIXFC_CFG_SSACT_0CLK                    ((uint32_t)0x0UL) /**< CFG_SSACT_0CLK Value */
#define MXC_S_SPIXFC_CFG_SSACT_0CLK                    (MXC_V_SPIXFC_CFG_SSACT_0CLK << MXC_F_SPIXFC_CFG_SSACT_POS) /**< CFG_SSACT_0CLK Setting */
#define MXC_V_SPIXFC_CFG_SSACT_2CLK                    ((uint32_t)0x1UL) /**< CFG_SSACT_2CLK Value */
#define MXC_S_SPIXFC_CFG_SSACT_2CLK                    (MXC_V_SPIXFC_CFG_SSACT_2CLK << MXC_F_SPIXFC_CFG_SSACT_POS) /**< CFG_SSACT_2CLK Setting */
#define MXC_V_SPIXFC_CFG_SSACT_4CLK                    ((uint32_t)0x2UL) /**< CFG_SSACT_4CLK Value */
#define MXC_S_SPIXFC_CFG_SSACT_4CLK                    (MXC_V_SPIXFC_CFG_SSACT_4CLK << MXC_F_SPIXFC_CFG_SSACT_POS) /**< CFG_SSACT_4CLK Setting */
#define MXC_V_SPIXFC_CFG_SSACT_8CLK                    ((uint32_t)0x3UL) /**< CFG_SSACT_8CLK Value */
#define MXC_S_SPIXFC_CFG_SSACT_8CLK                    (MXC_V_SPIXFC_CFG_SSACT_8CLK << MXC_F_SPIXFC_CFG_SSACT_POS) /**< CFG_SSACT_8CLK Setting */

#define MXC_F_SPIXFC_CFG_INACT_POS                     18 /**< CFG_INACT Position */
#define MXC_F_SPIXFC_CFG_INACT                         ((uint32_t)(0x3UL << MXC_F_SPIXFC_CFG_INACT_POS)) /**< CFG_INACT Mask */
#define MXC_V_SPIXFC_CFG_INACT_4CLK                    ((uint32_t)0x0UL) /**< CFG_INACT_4CLK Value */
#define MXC_S_SPIXFC_CFG_INACT_4CLK                    (MXC_V_SPIXFC_CFG_INACT_4CLK << MXC_F_SPIXFC_CFG_INACT_POS) /**< CFG_INACT_4CLK Setting */
#define MXC_V_SPIXFC_CFG_INACT_6CLK                    ((uint32_t)0x1UL) /**< CFG_INACT_6CLK Value */
#define MXC_S_SPIXFC_CFG_INACT_6CLK                    (MXC_V_SPIXFC_CFG_INACT_6CLK << MXC_F_SPIXFC_CFG_INACT_POS) /**< CFG_INACT_6CLK Setting */
#define MXC_V_SPIXFC_CFG_INACT_8CLK                    ((uint32_t)0x2UL) /**< CFG_INACT_8CLK Value */
#define MXC_S_SPIXFC_CFG_INACT_8CLK                    (MXC_V_SPIXFC_CFG_INACT_8CLK << MXC_F_SPIXFC_CFG_INACT_POS) /**< CFG_INACT_8CLK Setting */
#define MXC_V_SPIXFC_CFG_INACT_12CLK                   ((uint32_t)0x3UL) /**< CFG_INACT_12CLK Value */
#define MXC_S_SPIXFC_CFG_INACT_12CLK                   (MXC_V_SPIXFC_CFG_INACT_12CLK << MXC_F_SPIXFC_CFG_INACT_POS) /**< CFG_INACT_12CLK Setting */

#define MXC_F_SPIXFC_CFG_IOSMPL_POS                    20 /**< CFG_IOSMPL Position */
#define MXC_F_SPIXFC_CFG_IOSMPL                        ((uint32_t)(0xFUL << MXC_F_SPIXFC_CFG_IOSMPL_POS)) /**< CFG_IOSMPL Mask */
#define MXC_V_SPIXFC_CFG_IOSMPL_NODLY                  ((uint32_t)0x0UL) /**< CFG_IOSMPL_NODLY Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_NODLY                  (MXC_V_SPIXFC_CFG_IOSMPL_NODLY << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_NODLY Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_1CLK                   ((uint32_t)0x1UL) /**< CFG_IOSMPL_1CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_1CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_1CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_1CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_2CLK                   ((uint32_t)0x2UL) /**< CFG_IOSMPL_2CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_2CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_2CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_2CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_3CLK                   ((uint32_t)0x3UL) /**< CFG_IOSMPL_3CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_3CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_3CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_3CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_4CLK                   ((uint32_t)0x4UL) /**< CFG_IOSMPL_4CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_4CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_4CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_4CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_5CLK                   ((uint32_t)0x5UL) /**< CFG_IOSMPL_5CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_5CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_5CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_5CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_6CLK                   ((uint32_t)0x6UL) /**< CFG_IOSMPL_6CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_6CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_6CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_6CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_7CLK                   ((uint32_t)0x7UL) /**< CFG_IOSMPL_7CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_7CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_7CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_7CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_8CLK                   ((uint32_t)0x8UL) /**< CFG_IOSMPL_8CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_8CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_8CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_8CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_9CLK                   ((uint32_t)0x9UL) /**< CFG_IOSMPL_9CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_9CLK                   (MXC_V_SPIXFC_CFG_IOSMPL_9CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_9CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_10CLK                  ((uint32_t)0xAUL) /**< CFG_IOSMPL_10CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_10CLK                  (MXC_V_SPIXFC_CFG_IOSMPL_10CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_10CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_11CLK                  ((uint32_t)0xBUL) /**< CFG_IOSMPL_11CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_11CLK                  (MXC_V_SPIXFC_CFG_IOSMPL_11CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_11CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_12CLK                  ((uint32_t)0xCUL) /**< CFG_IOSMPL_12CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_12CLK                  (MXC_V_SPIXFC_CFG_IOSMPL_12CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_12CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_13CLK                  ((uint32_t)0xDUL) /**< CFG_IOSMPL_13CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_13CLK                  (MXC_V_SPIXFC_CFG_IOSMPL_13CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_13CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_14CLK                  ((uint32_t)0xEUL) /**< CFG_IOSMPL_14CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_14CLK                  (MXC_V_SPIXFC_CFG_IOSMPL_14CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_14CLK Setting */
#define MXC_V_SPIXFC_CFG_IOSMPL_15CLK                  ((uint32_t)0xFUL) /**< CFG_IOSMPL_15CLK Value */
#define MXC_S_SPIXFC_CFG_IOSMPL_15CLK                  (MXC_V_SPIXFC_CFG_IOSMPL_15CLK << MXC_F_SPIXFC_CFG_IOSMPL_POS) /**< CFG_IOSMPL_15CLK Setting */

/**@} end of group SPIXFC_CFG_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_SS_POL SPIXFC_SS_POL
 * @brief    SPIX Controller Slave Select Polarity Register.
 * @{
 */
#define MXC_F_SPIXFC_SS_POL_SSPOL_0_POS                0 /**< SS_POL_SSPOL_0 Position */
#define MXC_F_SPIXFC_SS_POL_SSPOL_0                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_SS_POL_SSPOL_0_POS)) /**< SS_POL_SSPOL_0 Mask */
#define MXC_V_SPIXFC_SS_POL_SSPOL_0_ACTIVELO           ((uint32_t)0x0UL) /**< SS_POL_SSPOL_0_ACTIVELO Value */
#define MXC_S_SPIXFC_SS_POL_SSPOL_0_ACTIVELO           (MXC_V_SPIXFC_SS_POL_SSPOL_0_ACTIVELO << MXC_F_SPIXFC_SS_POL_SSPOL_0_POS) /**< SS_POL_SSPOL_0_ACTIVELO Setting */
#define MXC_V_SPIXFC_SS_POL_SSPOL_0_ACTIVEHI           ((uint32_t)0x1UL) /**< SS_POL_SSPOL_0_ACTIVEHI Value */
#define MXC_S_SPIXFC_SS_POL_SSPOL_0_ACTIVEHI           (MXC_V_SPIXFC_SS_POL_SSPOL_0_ACTIVEHI << MXC_F_SPIXFC_SS_POL_SSPOL_0_POS) /**< SS_POL_SSPOL_0_ACTIVEHI Setting */

/**@} end of group SPIXFC_SS_POL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_GEN_CTRL SPIXFC_GEN_CTRL
 * @brief    SPIX Controller General Controller Register.
 * @{
 */
#define MXC_F_SPIXFC_GEN_CTRL_ENABLE_POS               0 /**< GEN_CTRL_ENABLE Position */
#define MXC_F_SPIXFC_GEN_CTRL_ENABLE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_ENABLE_POS)) /**< GEN_CTRL_ENABLE Mask */
#define MXC_V_SPIXFC_GEN_CTRL_ENABLE_DIS               ((uint32_t)0x0UL) /**< GEN_CTRL_ENABLE_DIS Value */
#define MXC_S_SPIXFC_GEN_CTRL_ENABLE_DIS               (MXC_V_SPIXFC_GEN_CTRL_ENABLE_DIS << MXC_F_SPIXFC_GEN_CTRL_ENABLE_POS) /**< GEN_CTRL_ENABLE_DIS Setting */
#define MXC_V_SPIXFC_GEN_CTRL_ENABLE_EN                ((uint32_t)0x1UL) /**< GEN_CTRL_ENABLE_EN Value */
#define MXC_S_SPIXFC_GEN_CTRL_ENABLE_EN                (MXC_V_SPIXFC_GEN_CTRL_ENABLE_EN << MXC_F_SPIXFC_GEN_CTRL_ENABLE_POS) /**< GEN_CTRL_ENABLE_EN Setting */

#define MXC_F_SPIXFC_GEN_CTRL_TFIFOEN_POS              1 /**< GEN_CTRL_TFIFOEN Position */
#define MXC_F_SPIXFC_GEN_CTRL_TFIFOEN                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_TFIFOEN_POS)) /**< GEN_CTRL_TFIFOEN Mask */
#define MXC_V_SPIXFC_GEN_CTRL_TFIFOEN_DIS              ((uint32_t)0x0UL) /**< GEN_CTRL_TFIFOEN_DIS Value */
#define MXC_S_SPIXFC_GEN_CTRL_TFIFOEN_DIS              (MXC_V_SPIXFC_GEN_CTRL_TFIFOEN_DIS << MXC_F_SPIXFC_GEN_CTRL_TFIFOEN_POS) /**< GEN_CTRL_TFIFOEN_DIS Setting */
#define MXC_V_SPIXFC_GEN_CTRL_TFIFOEN_EN               ((uint32_t)0x1UL) /**< GEN_CTRL_TFIFOEN_EN Value */
#define MXC_S_SPIXFC_GEN_CTRL_TFIFOEN_EN               (MXC_V_SPIXFC_GEN_CTRL_TFIFOEN_EN << MXC_F_SPIXFC_GEN_CTRL_TFIFOEN_POS) /**< GEN_CTRL_TFIFOEN_EN Setting */

#define MXC_F_SPIXFC_GEN_CTRL_RFIFOEN_POS              2 /**< GEN_CTRL_RFIFOEN Position */
#define MXC_F_SPIXFC_GEN_CTRL_RFIFOEN                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_RFIFOEN_POS)) /**< GEN_CTRL_RFIFOEN Mask */
#define MXC_V_SPIXFC_GEN_CTRL_RFIFOEN_DIS              ((uint32_t)0x0UL) /**< GEN_CTRL_RFIFOEN_DIS Value */
#define MXC_S_SPIXFC_GEN_CTRL_RFIFOEN_DIS              (MXC_V_SPIXFC_GEN_CTRL_RFIFOEN_DIS << MXC_F_SPIXFC_GEN_CTRL_RFIFOEN_POS) /**< GEN_CTRL_RFIFOEN_DIS Setting */
#define MXC_V_SPIXFC_GEN_CTRL_RFIFOEN_EN               ((uint32_t)0x1UL) /**< GEN_CTRL_RFIFOEN_EN Value */
#define MXC_S_SPIXFC_GEN_CTRL_RFIFOEN_EN               (MXC_V_SPIXFC_GEN_CTRL_RFIFOEN_EN << MXC_F_SPIXFC_GEN_CTRL_RFIFOEN_POS) /**< GEN_CTRL_RFIFOEN_EN Setting */

#define MXC_F_SPIXFC_GEN_CTRL_BBMODE_POS               3 /**< GEN_CTRL_BBMODE Position */
#define MXC_F_SPIXFC_GEN_CTRL_BBMODE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_BBMODE_POS)) /**< GEN_CTRL_BBMODE Mask */
#define MXC_V_SPIXFC_GEN_CTRL_BBMODE_DIS               ((uint32_t)0x0UL) /**< GEN_CTRL_BBMODE_DIS Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBMODE_DIS               (MXC_V_SPIXFC_GEN_CTRL_BBMODE_DIS << MXC_F_SPIXFC_GEN_CTRL_BBMODE_POS) /**< GEN_CTRL_BBMODE_DIS Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBMODE_EN                ((uint32_t)0x1UL) /**< GEN_CTRL_BBMODE_EN Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBMODE_EN                (MXC_V_SPIXFC_GEN_CTRL_BBMODE_EN << MXC_F_SPIXFC_GEN_CTRL_BBMODE_POS) /**< GEN_CTRL_BBMODE_EN Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SSDR_POS                 4 /**< GEN_CTRL_SSDR Position */
#define MXC_F_SPIXFC_GEN_CTRL_SSDR                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SSDR_POS)) /**< GEN_CTRL_SSDR Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SSDR_OUTPUT0             ((uint32_t)0x0UL) /**< GEN_CTRL_SSDR_OUTPUT0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SSDR_OUTPUT0             (MXC_V_SPIXFC_GEN_CTRL_SSDR_OUTPUT0 << MXC_F_SPIXFC_GEN_CTRL_SSDR_POS) /**< GEN_CTRL_SSDR_OUTPUT0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SSDR_OUTPUT1             ((uint32_t)0x1UL) /**< GEN_CTRL_SSDR_OUTPUT1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SSDR_OUTPUT1             (MXC_V_SPIXFC_GEN_CTRL_SSDR_OUTPUT1 << MXC_F_SPIXFC_GEN_CTRL_SSDR_POS) /**< GEN_CTRL_SSDR_OUTPUT1 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SCKDR_POS                6 /**< GEN_CTRL_SCKDR Position */
#define MXC_F_SPIXFC_GEN_CTRL_SCKDR                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SCKDR_POS)) /**< GEN_CTRL_SCKDR Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SCKDR_SCK0               ((uint32_t)0x0UL) /**< GEN_CTRL_SCKDR_SCK0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SCKDR_SCK0               (MXC_V_SPIXFC_GEN_CTRL_SCKDR_SCK0 << MXC_F_SPIXFC_GEN_CTRL_SCKDR_POS) /**< GEN_CTRL_SCKDR_SCK0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SCKDR_SCK1               ((uint32_t)0x1UL) /**< GEN_CTRL_SCKDR_SCK1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SCKDR_SCK1               (MXC_V_SPIXFC_GEN_CTRL_SCKDR_SCK1 << MXC_F_SPIXFC_GEN_CTRL_SCKDR_POS) /**< GEN_CTRL_SCKDR_SCK1 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SDATAIN_POS              8 /**< GEN_CTRL_SDATAIN Position */
#define MXC_F_SPIXFC_GEN_CTRL_SDATAIN                  ((uint32_t)(0xFUL << MXC_F_SPIXFC_GEN_CTRL_SDATAIN_POS)) /**< GEN_CTRL_SDATAIN Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO0            ((uint32_t)0x1UL) /**< GEN_CTRL_SDATAIN_SDIO0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDATAIN_SDIO0            (MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO0 << MXC_F_SPIXFC_GEN_CTRL_SDATAIN_POS) /**< GEN_CTRL_SDATAIN_SDIO0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO1            ((uint32_t)0x2UL) /**< GEN_CTRL_SDATAIN_SDIO1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDATAIN_SDIO1            (MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO1 << MXC_F_SPIXFC_GEN_CTRL_SDATAIN_POS) /**< GEN_CTRL_SDATAIN_SDIO1 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO2            ((uint32_t)0x4UL) /**< GEN_CTRL_SDATAIN_SDIO2 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDATAIN_SDIO2            (MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO2 << MXC_F_SPIXFC_GEN_CTRL_SDATAIN_POS) /**< GEN_CTRL_SDATAIN_SDIO2 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO3            ((uint32_t)0x8UL) /**< GEN_CTRL_SDATAIN_SDIO3 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDATAIN_SDIO3            (MXC_V_SPIXFC_GEN_CTRL_SDATAIN_SDIO3 << MXC_F_SPIXFC_GEN_CTRL_SDATAIN_POS) /**< GEN_CTRL_SDATAIN_SDIO3 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_BBDAT_POS                12 /**< GEN_CTRL_BBDAT Position */
#define MXC_F_SPIXFC_GEN_CTRL_BBDAT                    ((uint32_t)(0xFUL << MXC_F_SPIXFC_GEN_CTRL_BBDAT_POS)) /**< GEN_CTRL_BBDAT Mask */
#define MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO0              ((uint32_t)0x1UL) /**< GEN_CTRL_BBDAT_SDIO0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDAT_SDIO0              (MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO0 << MXC_F_SPIXFC_GEN_CTRL_BBDAT_POS) /**< GEN_CTRL_BBDAT_SDIO0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO1              ((uint32_t)0x2UL) /**< GEN_CTRL_BBDAT_SDIO1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDAT_SDIO1              (MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO1 << MXC_F_SPIXFC_GEN_CTRL_BBDAT_POS) /**< GEN_CTRL_BBDAT_SDIO1 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO2              ((uint32_t)0x4UL) /**< GEN_CTRL_BBDAT_SDIO2 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDAT_SDIO2              (MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO2 << MXC_F_SPIXFC_GEN_CTRL_BBDAT_POS) /**< GEN_CTRL_BBDAT_SDIO2 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO3              ((uint32_t)0x8UL) /**< GEN_CTRL_BBDAT_SDIO3 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDAT_SDIO3              (MXC_V_SPIXFC_GEN_CTRL_BBDAT_SDIO3 << MXC_F_SPIXFC_GEN_CTRL_BBDAT_POS) /**< GEN_CTRL_BBDAT_SDIO3 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_BBDATOEN_POS             16 /**< GEN_CTRL_BBDATOEN Position */
#define MXC_F_SPIXFC_GEN_CTRL_BBDATOEN                 ((uint32_t)(0xFUL << MXC_F_SPIXFC_GEN_CTRL_BBDATOEN_POS)) /**< GEN_CTRL_BBDATOEN Mask */
#define MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO0           ((uint32_t)0x1UL) /**< GEN_CTRL_BBDATOEN_SDIO0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDATOEN_SDIO0           (MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO0 << MXC_F_SPIXFC_GEN_CTRL_BBDATOEN_POS) /**< GEN_CTRL_BBDATOEN_SDIO0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO1           ((uint32_t)0x2UL) /**< GEN_CTRL_BBDATOEN_SDIO1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDATOEN_SDIO1           (MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO1 << MXC_F_SPIXFC_GEN_CTRL_BBDATOEN_POS) /**< GEN_CTRL_BBDATOEN_SDIO1 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO2           ((uint32_t)0x4UL) /**< GEN_CTRL_BBDATOEN_SDIO2 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDATOEN_SDIO2           (MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO2 << MXC_F_SPIXFC_GEN_CTRL_BBDATOEN_POS) /**< GEN_CTRL_BBDATOEN_SDIO2 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO3           ((uint32_t)0x8UL) /**< GEN_CTRL_BBDATOEN_SDIO3 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BBDATOEN_SDIO3           (MXC_V_SPIXFC_GEN_CTRL_BBDATOEN_SDIO3 << MXC_F_SPIXFC_GEN_CTRL_BBDATOEN_POS) /**< GEN_CTRL_BBDATOEN_SDIO3 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SIMPLE_POS               20 /**< GEN_CTRL_SIMPLE Position */
#define MXC_F_SPIXFC_GEN_CTRL_SIMPLE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SIMPLE_POS)) /**< GEN_CTRL_SIMPLE Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SIMPLE_DIS               ((uint32_t)0x0UL) /**< GEN_CTRL_SIMPLE_DIS Value */
#define MXC_S_SPIXFC_GEN_CTRL_SIMPLE_DIS               (MXC_V_SPIXFC_GEN_CTRL_SIMPLE_DIS << MXC_F_SPIXFC_GEN_CTRL_SIMPLE_POS) /**< GEN_CTRL_SIMPLE_DIS Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SIMPLE_EN                ((uint32_t)0x1UL) /**< GEN_CTRL_SIMPLE_EN Value */
#define MXC_S_SPIXFC_GEN_CTRL_SIMPLE_EN                (MXC_V_SPIXFC_GEN_CTRL_SIMPLE_EN << MXC_F_SPIXFC_GEN_CTRL_SIMPLE_POS) /**< GEN_CTRL_SIMPLE_EN Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SIMPLERX_POS             21 /**< GEN_CTRL_SIMPLERX Position */
#define MXC_F_SPIXFC_GEN_CTRL_SIMPLERX                 ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SIMPLERX_POS)) /**< GEN_CTRL_SIMPLERX Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SIMPLERX_INITSPI         ((uint32_t)0x1UL) /**< GEN_CTRL_SIMPLERX_INITSPI Value */
#define MXC_S_SPIXFC_GEN_CTRL_SIMPLERX_INITSPI         (MXC_V_SPIXFC_GEN_CTRL_SIMPLERX_INITSPI << MXC_F_SPIXFC_GEN_CTRL_SIMPLERX_POS) /**< GEN_CTRL_SIMPLERX_INITSPI Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SMPLSS_POS               22 /**< GEN_CTRL_SMPLSS Position */
#define MXC_F_SPIXFC_GEN_CTRL_SMPLSS                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SMPLSS_POS)) /**< GEN_CTRL_SMPLSS Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SMPLSS_DEASSERTSS        ((uint32_t)0x1UL) /**< GEN_CTRL_SMPLSS_DEASSERTSS Value */
#define MXC_S_SPIXFC_GEN_CTRL_SMPLSS_DEASSERTSS        (MXC_V_SPIXFC_GEN_CTRL_SMPLSS_DEASSERTSS << MXC_F_SPIXFC_GEN_CTRL_SMPLSS_POS) /**< GEN_CTRL_SMPLSS_DEASSERTSS Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SCKFB_POS                24 /**< GEN_CTRL_SCKFB Position */
#define MXC_F_SPIXFC_GEN_CTRL_SCKFB                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SCKFB_POS)) /**< GEN_CTRL_SCKFB Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SCKFB_DIS                ((uint32_t)0x0UL) /**< GEN_CTRL_SCKFB_DIS Value */
#define MXC_S_SPIXFC_GEN_CTRL_SCKFB_DIS                (MXC_V_SPIXFC_GEN_CTRL_SCKFB_DIS << MXC_F_SPIXFC_GEN_CTRL_SCKFB_POS) /**< GEN_CTRL_SCKFB_DIS Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SCKFB_EN                 ((uint32_t)0x1UL) /**< GEN_CTRL_SCKFB_EN Value */
#define MXC_S_SPIXFC_GEN_CTRL_SCKFB_EN                 (MXC_V_SPIXFC_GEN_CTRL_SCKFB_EN << MXC_F_SPIXFC_GEN_CTRL_SCKFB_POS) /**< GEN_CTRL_SCKFB_EN Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SCKFBINV_POS             25 /**< GEN_CTRL_SCKFBINV Position */
#define MXC_F_SPIXFC_GEN_CTRL_SCKFBINV                 ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SCKFBINV_POS)) /**< GEN_CTRL_SCKFBINV Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SCKFBINV_NORMAL          ((uint32_t)0x0UL) /**< GEN_CTRL_SCKFBINV_NORMAL Value */
#define MXC_S_SPIXFC_GEN_CTRL_SCKFBINV_NORMAL          (MXC_V_SPIXFC_GEN_CTRL_SCKFBINV_NORMAL << MXC_F_SPIXFC_GEN_CTRL_SCKFBINV_POS) /**< GEN_CTRL_SCKFBINV_NORMAL Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SCKFBINV_INVERT          ((uint32_t)0x1UL) /**< GEN_CTRL_SCKFBINV_INVERT Value */
#define MXC_S_SPIXFC_GEN_CTRL_SCKFBINV_INVERT          (MXC_V_SPIXFC_GEN_CTRL_SCKFBINV_INVERT << MXC_F_SPIXFC_GEN_CTRL_SCKFBINV_POS) /**< GEN_CTRL_SCKFBINV_INVERT Setting */

/**@} end of group SPIXFC_GEN_CTRL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_FIFO_CTRL SPIXFC_FIFO_CTRL
 * @brief    SPIX Controller FIFO Control and Status Register.
 * @{
 */
#define MXC_F_SPIXFC_FIFO_CTRL_TFIFOLVL_POS            0 /**< FIFO_CTRL_TFIFOLVL Position */
#define MXC_F_SPIXFC_FIFO_CTRL_TFIFOLVL                ((uint32_t)(0xFUL << MXC_F_SPIXFC_FIFO_CTRL_TFIFOLVL_POS)) /**< FIFO_CTRL_TFIFOLVL Mask */

#define MXC_F_SPIXFC_FIFO_CTRL_TFIFOCNT_POS            8 /**< FIFO_CTRL_TFIFOCNT Position */
#define MXC_F_SPIXFC_FIFO_CTRL_TFIFOCNT                ((uint32_t)(0x1FUL << MXC_F_SPIXFC_FIFO_CTRL_TFIFOCNT_POS)) /**< FIFO_CTRL_TFIFOCNT Mask */

#define MXC_F_SPIXFC_FIFO_CTRL_RFIFOLVL_POS            16 /**< FIFO_CTRL_RFIFOLVL Position */
#define MXC_F_SPIXFC_FIFO_CTRL_RFIFOLVL                ((uint32_t)(0x1FUL << MXC_F_SPIXFC_FIFO_CTRL_RFIFOLVL_POS)) /**< FIFO_CTRL_RFIFOLVL Mask */

#define MXC_F_SPIXFC_FIFO_CTRL_RFIFOCNT_POS            24 /**< FIFO_CTRL_RFIFOCNT Position */
#define MXC_F_SPIXFC_FIFO_CTRL_RFIFOCNT                ((uint32_t)(0x3FUL << MXC_F_SPIXFC_FIFO_CTRL_RFIFOCNT_POS)) /**< FIFO_CTRL_RFIFOCNT Mask */

/**@} end of group SPIXFC_FIFO_CTRL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_SP_CTRL SPIXFC_SP_CTRL
 * @brief    SPIX Controller Special Control Register.
 * @{
 */
#define MXC_F_SPIXFC_SP_CTRL_SAMPL_POS                 0 /**< SP_CTRL_SAMPL Position */
#define MXC_F_SPIXFC_SP_CTRL_SAMPL                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_SP_CTRL_SAMPL_POS)) /**< SP_CTRL_SAMPL Mask */
#define MXC_V_SPIXFC_SP_CTRL_SAMPL_DIS                 ((uint32_t)0x0UL) /**< SP_CTRL_SAMPL_DIS Value */
#define MXC_S_SPIXFC_SP_CTRL_SAMPL_DIS                 (MXC_V_SPIXFC_SP_CTRL_SAMPL_DIS << MXC_F_SPIXFC_SP_CTRL_SAMPL_POS) /**< SP_CTRL_SAMPL_DIS Setting */
#define MXC_V_SPIXFC_SP_CTRL_SAMPL_EN                  ((uint32_t)0x1UL) /**< SP_CTRL_SAMPL_EN Value */
#define MXC_S_SPIXFC_SP_CTRL_SAMPL_EN                  (MXC_V_SPIXFC_SP_CTRL_SAMPL_EN << MXC_F_SPIXFC_SP_CTRL_SAMPL_POS) /**< SP_CTRL_SAMPL_EN Setting */

#define MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_POS              4 /**< SP_CTRL_SDIO_OUT Position */
#define MXC_F_SPIXFC_SP_CTRL_SDIO_OUT                  ((uint32_t)(0xFUL << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_POS)) /**< SP_CTRL_SDIO_OUT Mask */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO0            ((uint32_t)0x1UL) /**< SP_CTRL_SDIO_OUT_SDIO0 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_SDIO0            (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO0 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_POS) /**< SP_CTRL_SDIO_OUT_SDIO0 Setting */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO1            ((uint32_t)0x2UL) /**< SP_CTRL_SDIO_OUT_SDIO1 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_SDIO1            (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO1 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_POS) /**< SP_CTRL_SDIO_OUT_SDIO1 Setting */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO2            ((uint32_t)0x4UL) /**< SP_CTRL_SDIO_OUT_SDIO2 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_SDIO2            (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO2 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_POS) /**< SP_CTRL_SDIO_OUT_SDIO2 Setting */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO3            ((uint32_t)0x8UL) /**< SP_CTRL_SDIO_OUT_SDIO3 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_SDIO3            (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_SDIO3 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_POS) /**< SP_CTRL_SDIO_OUT_SDIO3 Setting */

#define MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN_POS           8 /**< SP_CTRL_SDIO_OUT_EN Position */
#define MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN               ((uint32_t)(0xFUL << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN_POS)) /**< SP_CTRL_SDIO_OUT_EN Mask */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO0         ((uint32_t)0x1UL) /**< SP_CTRL_SDIO_OUT_EN_SDIO0 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO0         (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO0 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN_POS) /**< SP_CTRL_SDIO_OUT_EN_SDIO0 Setting */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO1         ((uint32_t)0x2UL) /**< SP_CTRL_SDIO_OUT_EN_SDIO1 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO1         (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO1 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN_POS) /**< SP_CTRL_SDIO_OUT_EN_SDIO1 Setting */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO2         ((uint32_t)0x4UL) /**< SP_CTRL_SDIO_OUT_EN_SDIO2 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO2         (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO2 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN_POS) /**< SP_CTRL_SDIO_OUT_EN_SDIO2 Setting */
#define MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO3         ((uint32_t)0x8UL) /**< SP_CTRL_SDIO_OUT_EN_SDIO3 Value */
#define MXC_S_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO3         (MXC_V_SPIXFC_SP_CTRL_SDIO_OUT_EN_SDIO3 << MXC_F_SPIXFC_SP_CTRL_SDIO_OUT_EN_POS) /**< SP_CTRL_SDIO_OUT_EN_SDIO3 Setting */

#define MXC_F_SPIXFC_SP_CTRL_SCKINH3_POS               16 /**< SP_CTRL_SCKINH3 Position */
#define MXC_F_SPIXFC_SP_CTRL_SCKINH3                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_SP_CTRL_SCKINH3_POS)) /**< SP_CTRL_SCKINH3 Mask */
#define MXC_V_SPIXFC_SP_CTRL_SCKINH3_EN                ((uint32_t)0x0UL) /**< SP_CTRL_SCKINH3_EN Value */
#define MXC_S_SPIXFC_SP_CTRL_SCKINH3_EN                (MXC_V_SPIXFC_SP_CTRL_SCKINH3_EN << MXC_F_SPIXFC_SP_CTRL_SCKINH3_POS) /**< SP_CTRL_SCKINH3_EN Setting */
#define MXC_V_SPIXFC_SP_CTRL_SCKINH3_DIS               ((uint32_t)0x1UL) /**< SP_CTRL_SCKINH3_DIS Value */
#define MXC_S_SPIXFC_SP_CTRL_SCKINH3_DIS               (MXC_V_SPIXFC_SP_CTRL_SCKINH3_DIS << MXC_F_SPIXFC_SP_CTRL_SCKINH3_POS) /**< SP_CTRL_SCKINH3_DIS Setting */

/**@} end of group SPIXFC_SP_CTRL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_INT_FL SPIXFC_INT_FL
 * @brief    SPIX Controller Interrupt Status Register.
 * @{
 */
#define MXC_F_SPIXFC_INT_FL_TSTALL_POS                 0 /**< INT_FL_TSTALL Position */
#define MXC_F_SPIXFC_INT_FL_TSTALL                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_FL_TSTALL_POS)) /**< INT_FL_TSTALL Mask */
#define MXC_V_SPIXFC_INT_FL_TSTALL_CLEAR               ((uint32_t)0x1UL) /**< INT_FL_TSTALL_CLEAR Value */
#define MXC_S_SPIXFC_INT_FL_TSTALL_CLEAR               (MXC_V_SPIXFC_INT_FL_TSTALL_CLEAR << MXC_F_SPIXFC_INT_FL_TSTALL_POS) /**< INT_FL_TSTALL_CLEAR Setting */

#define MXC_F_SPIXFC_INT_FL_RSTALL_POS                 1 /**< INT_FL_RSTALL Position */
#define MXC_F_SPIXFC_INT_FL_RSTALL                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_FL_RSTALL_POS)) /**< INT_FL_RSTALL Mask */
#define MXC_V_SPIXFC_INT_FL_RSTALL_CLEAR               ((uint32_t)0x1UL) /**< INT_FL_RSTALL_CLEAR Value */
#define MXC_S_SPIXFC_INT_FL_RSTALL_CLEAR               (MXC_V_SPIXFC_INT_FL_RSTALL_CLEAR << MXC_F_SPIXFC_INT_FL_RSTALL_POS) /**< INT_FL_RSTALL_CLEAR Setting */

#define MXC_F_SPIXFC_INT_FL_TRDY_POS                   2 /**< INT_FL_TRDY Position */
#define MXC_F_SPIXFC_INT_FL_TRDY                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_FL_TRDY_POS)) /**< INT_FL_TRDY Mask */
#define MXC_V_SPIXFC_INT_FL_TRDY_CLEAR                 ((uint32_t)0x1UL) /**< INT_FL_TRDY_CLEAR Value */
#define MXC_S_SPIXFC_INT_FL_TRDY_CLEAR                 (MXC_V_SPIXFC_INT_FL_TRDY_CLEAR << MXC_F_SPIXFC_INT_FL_TRDY_POS) /**< INT_FL_TRDY_CLEAR Setting */

#define MXC_F_SPIXFC_INT_FL_RDONE_POS                  3 /**< INT_FL_RDONE Position */
#define MXC_F_SPIXFC_INT_FL_RDONE                      ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_FL_RDONE_POS)) /**< INT_FL_RDONE Mask */
#define MXC_V_SPIXFC_INT_FL_RDONE_CLEAR                ((uint32_t)0x1UL) /**< INT_FL_RDONE_CLEAR Value */
#define MXC_S_SPIXFC_INT_FL_RDONE_CLEAR                (MXC_V_SPIXFC_INT_FL_RDONE_CLEAR << MXC_F_SPIXFC_INT_FL_RDONE_POS) /**< INT_FL_RDONE_CLEAR Setting */

#define MXC_F_SPIXFC_INT_FL_TFIFOAE_POS                4 /**< INT_FL_TFIFOAE Position */
#define MXC_F_SPIXFC_INT_FL_TFIFOAE                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_FL_TFIFOAE_POS)) /**< INT_FL_TFIFOAE Mask */
#define MXC_V_SPIXFC_INT_FL_TFIFOAE_CLEAR              ((uint32_t)0x1UL) /**< INT_FL_TFIFOAE_CLEAR Value */
#define MXC_S_SPIXFC_INT_FL_TFIFOAE_CLEAR              (MXC_V_SPIXFC_INT_FL_TFIFOAE_CLEAR << MXC_F_SPIXFC_INT_FL_TFIFOAE_POS) /**< INT_FL_TFIFOAE_CLEAR Setting */

#define MXC_F_SPIXFC_INT_FL_RFIFOAF_POS                5 /**< INT_FL_RFIFOAF Position */
#define MXC_F_SPIXFC_INT_FL_RFIFOAF                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_FL_RFIFOAF_POS)) /**< INT_FL_RFIFOAF Mask */
#define MXC_V_SPIXFC_INT_FL_RFIFOAF_CLEAR              ((uint32_t)0x1UL) /**< INT_FL_RFIFOAF_CLEAR Value */
#define MXC_S_SPIXFC_INT_FL_RFIFOAF_CLEAR              (MXC_V_SPIXFC_INT_FL_RFIFOAF_CLEAR << MXC_F_SPIXFC_INT_FL_RFIFOAF_POS) /**< INT_FL_RFIFOAF_CLEAR Setting */

/**@} end of group SPIXFC_INT_FL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_INT_EN SPIXFC_INT_EN
 * @brief    SPIX Controller Interrupt Enable Register.
 * @{
 */
#define MXC_F_SPIXFC_INT_EN_TSTALLIE_POS               0 /**< INT_EN_TSTALLIE Position */
#define MXC_F_SPIXFC_INT_EN_TSTALLIE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_EN_TSTALLIE_POS)) /**< INT_EN_TSTALLIE Mask */
#define MXC_V_SPIXFC_INT_EN_TSTALLIE_DIS               ((uint32_t)0x0UL) /**< INT_EN_TSTALLIE_DIS Value */
#define MXC_S_SPIXFC_INT_EN_TSTALLIE_DIS               (MXC_V_SPIXFC_INT_EN_TSTALLIE_DIS << MXC_F_SPIXFC_INT_EN_TSTALLIE_POS) /**< INT_EN_TSTALLIE_DIS Setting */
#define MXC_V_SPIXFC_INT_EN_TSTALLIE_EN                ((uint32_t)0x1UL) /**< INT_EN_TSTALLIE_EN Value */
#define MXC_S_SPIXFC_INT_EN_TSTALLIE_EN                (MXC_V_SPIXFC_INT_EN_TSTALLIE_EN << MXC_F_SPIXFC_INT_EN_TSTALLIE_POS) /**< INT_EN_TSTALLIE_EN Setting */

#define MXC_F_SPIXFC_INT_EN_RSTALLIE_POS               1 /**< INT_EN_RSTALLIE Position */
#define MXC_F_SPIXFC_INT_EN_RSTALLIE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_EN_RSTALLIE_POS)) /**< INT_EN_RSTALLIE Mask */
#define MXC_V_SPIXFC_INT_EN_RSTALLIE_DIS               ((uint32_t)0x0UL) /**< INT_EN_RSTALLIE_DIS Value */
#define MXC_S_SPIXFC_INT_EN_RSTALLIE_DIS               (MXC_V_SPIXFC_INT_EN_RSTALLIE_DIS << MXC_F_SPIXFC_INT_EN_RSTALLIE_POS) /**< INT_EN_RSTALLIE_DIS Setting */
#define MXC_V_SPIXFC_INT_EN_RSTALLIE_EN                ((uint32_t)0x1UL) /**< INT_EN_RSTALLIE_EN Value */
#define MXC_S_SPIXFC_INT_EN_RSTALLIE_EN                (MXC_V_SPIXFC_INT_EN_RSTALLIE_EN << MXC_F_SPIXFC_INT_EN_RSTALLIE_POS) /**< INT_EN_RSTALLIE_EN Setting */

#define MXC_F_SPIXFC_INT_EN_TRDYIE_POS                 2 /**< INT_EN_TRDYIE Position */
#define MXC_F_SPIXFC_INT_EN_TRDYIE                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_EN_TRDYIE_POS)) /**< INT_EN_TRDYIE Mask */
#define MXC_V_SPIXFC_INT_EN_TRDYIE_DIS                 ((uint32_t)0x0UL) /**< INT_EN_TRDYIE_DIS Value */
#define MXC_S_SPIXFC_INT_EN_TRDYIE_DIS                 (MXC_V_SPIXFC_INT_EN_TRDYIE_DIS << MXC_F_SPIXFC_INT_EN_TRDYIE_POS) /**< INT_EN_TRDYIE_DIS Setting */
#define MXC_V_SPIXFC_INT_EN_TRDYIE_EN                  ((uint32_t)0x1UL) /**< INT_EN_TRDYIE_EN Value */
#define MXC_S_SPIXFC_INT_EN_TRDYIE_EN                  (MXC_V_SPIXFC_INT_EN_TRDYIE_EN << MXC_F_SPIXFC_INT_EN_TRDYIE_POS) /**< INT_EN_TRDYIE_EN Setting */

#define MXC_F_SPIXFC_INT_EN_RDONEIE_POS                3 /**< INT_EN_RDONEIE Position */
#define MXC_F_SPIXFC_INT_EN_RDONEIE                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_EN_RDONEIE_POS)) /**< INT_EN_RDONEIE Mask */
#define MXC_V_SPIXFC_INT_EN_RDONEIE_DIS                ((uint32_t)0x0UL) /**< INT_EN_RDONEIE_DIS Value */
#define MXC_S_SPIXFC_INT_EN_RDONEIE_DIS                (MXC_V_SPIXFC_INT_EN_RDONEIE_DIS << MXC_F_SPIXFC_INT_EN_RDONEIE_POS) /**< INT_EN_RDONEIE_DIS Setting */
#define MXC_V_SPIXFC_INT_EN_RDONEIE_EN                 ((uint32_t)0x1UL) /**< INT_EN_RDONEIE_EN Value */
#define MXC_S_SPIXFC_INT_EN_RDONEIE_EN                 (MXC_V_SPIXFC_INT_EN_RDONEIE_EN << MXC_F_SPIXFC_INT_EN_RDONEIE_POS) /**< INT_EN_RDONEIE_EN Setting */

#define MXC_F_SPIXFC_INT_EN_TFIFOAEIE_POS              4 /**< INT_EN_TFIFOAEIE Position */
#define MXC_F_SPIXFC_INT_EN_TFIFOAEIE                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_EN_TFIFOAEIE_POS)) /**< INT_EN_TFIFOAEIE Mask */
#define MXC_V_SPIXFC_INT_EN_TFIFOAEIE_DIS              ((uint32_t)0x0UL) /**< INT_EN_TFIFOAEIE_DIS Value */
#define MXC_S_SPIXFC_INT_EN_TFIFOAEIE_DIS              (MXC_V_SPIXFC_INT_EN_TFIFOAEIE_DIS << MXC_F_SPIXFC_INT_EN_TFIFOAEIE_POS) /**< INT_EN_TFIFOAEIE_DIS Setting */
#define MXC_V_SPIXFC_INT_EN_TFIFOAEIE_EN               ((uint32_t)0x1UL) /**< INT_EN_TFIFOAEIE_EN Value */
#define MXC_S_SPIXFC_INT_EN_TFIFOAEIE_EN               (MXC_V_SPIXFC_INT_EN_TFIFOAEIE_EN << MXC_F_SPIXFC_INT_EN_TFIFOAEIE_POS) /**< INT_EN_TFIFOAEIE_EN Setting */

#define MXC_F_SPIXFC_INT_EN_RFIFOAFIE_POS              5 /**< INT_EN_RFIFOAFIE Position */
#define MXC_F_SPIXFC_INT_EN_RFIFOAFIE                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INT_EN_RFIFOAFIE_POS)) /**< INT_EN_RFIFOAFIE Mask */
#define MXC_V_SPIXFC_INT_EN_RFIFOAFIE_DIS              ((uint32_t)0x0UL) /**< INT_EN_RFIFOAFIE_DIS Value */
#define MXC_S_SPIXFC_INT_EN_RFIFOAFIE_DIS              (MXC_V_SPIXFC_INT_EN_RFIFOAFIE_DIS << MXC_F_SPIXFC_INT_EN_RFIFOAFIE_POS) /**< INT_EN_RFIFOAFIE_DIS Setting */
#define MXC_V_SPIXFC_INT_EN_RFIFOAFIE_EN               ((uint32_t)0x1UL) /**< INT_EN_RFIFOAFIE_EN Value */
#define MXC_S_SPIXFC_INT_EN_RFIFOAFIE_EN               (MXC_V_SPIXFC_INT_EN_RFIFOAFIE_EN << MXC_F_SPIXFC_INT_EN_RFIFOAFIE_POS) /**< INT_EN_RFIFOAFIE_EN Setting */

/**@} end of group SPIXFC_INT_EN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXFC_REGS_H_
