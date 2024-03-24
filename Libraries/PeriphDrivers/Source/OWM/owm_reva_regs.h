/**
 * @file    owm_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the OWM_REVA Peripheral Module.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_REGS_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_REGS_H_

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
 * @ingroup     owm_reva
 * @defgroup    owm_reva_registers OWM_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the OWM_REVA Peripheral Module.
 * @details 1-Wire Master Interface.
 */

/**
 * @ingroup owm_reva_registers
 * Structure type to access the OWM_REVA Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x0000:</tt> OWM_REVA CFG Register */
    __IO uint32_t clk_div_1us;          /**< <tt>\b 0x0004:</tt> OWM_REVA CLK_DIV_1US Register */
    __IO uint32_t ctrl_stat;            /**< <tt>\b 0x0008:</tt> OWM_REVA CTRL_STAT Register */
    __IO uint32_t data;                 /**< <tt>\b 0x000C:</tt> OWM_REVA DATA Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0010:</tt> OWM_REVA INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0014:</tt> OWM_REVA INTEN Register */
} mxc_owm_reva_regs_t;

/* Register offsets for module OWM_REVA */
/**
 * @ingroup    owm_reva_registers
 * @defgroup   OWM_REVA_Register_Offsets Register Offsets
 * @brief      OWM_REVA Peripheral Register Offsets from the OWM_REVA Base Peripheral Address. 
 * @{
 */
#define MXC_R_OWM_REVA_CFG                 ((uint32_t)0x00000000UL) /**< Offset from OWM_REVA Base Address: <tt> 0x0000</tt> */
#define MXC_R_OWM_REVA_CLK_DIV_1US         ((uint32_t)0x00000004UL) /**< Offset from OWM_REVA Base Address: <tt> 0x0004</tt> */
#define MXC_R_OWM_REVA_CTRL_STAT           ((uint32_t)0x00000008UL) /**< Offset from OWM_REVA Base Address: <tt> 0x0008</tt> */
#define MXC_R_OWM_REVA_DATA                ((uint32_t)0x0000000CUL) /**< Offset from OWM_REVA Base Address: <tt> 0x000C</tt> */
#define MXC_R_OWM_REVA_INTFL               ((uint32_t)0x00000010UL) /**< Offset from OWM_REVA Base Address: <tt> 0x0010</tt> */
#define MXC_R_OWM_REVA_INTEN               ((uint32_t)0x00000014UL) /**< Offset from OWM_REVA Base Address: <tt> 0x0014</tt> */
/**@} end of group owm_reva_registers */

/**
 * @ingroup  owm_reva_registers
 * @defgroup OWM_REVA_CFG OWM_REVA_CFG
 * @brief    1-Wire Master Configuration.
 * @{
 */
#define MXC_F_OWM_REVA_CFG_LONG_LINE_MODE_POS          0 /**< CFG_LONG_LINE_MODE Position */
#define MXC_F_OWM_REVA_CFG_LONG_LINE_MODE              ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_LONG_LINE_MODE_POS)) /**< CFG_LONG_LINE_MODE Mask */

#define MXC_F_OWM_REVA_CFG_FORCE_PRES_DET_POS          1 /**< CFG_FORCE_PRES_DET Position */
#define MXC_F_OWM_REVA_CFG_FORCE_PRES_DET              ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_FORCE_PRES_DET_POS)) /**< CFG_FORCE_PRES_DET Mask */

#define MXC_F_OWM_REVA_CFG_BIT_BANG_EN_POS             2 /**< CFG_BIT_BANG_EN Position */
#define MXC_F_OWM_REVA_CFG_BIT_BANG_EN                 ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_BIT_BANG_EN_POS)) /**< CFG_BIT_BANG_EN Mask */

#define MXC_F_OWM_REVA_CFG_EXT_PULLUP_MODE_POS         3 /**< CFG_EXT_PULLUP_MODE Position */
#define MXC_F_OWM_REVA_CFG_EXT_PULLUP_MODE             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_EXT_PULLUP_MODE_POS)) /**< CFG_EXT_PULLUP_MODE Mask */

#define MXC_F_OWM_REVA_CFG_EXT_PULLUP_ENABLE_POS       4 /**< CFG_EXT_PULLUP_ENABLE Position */
#define MXC_F_OWM_REVA_CFG_EXT_PULLUP_ENABLE           ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_EXT_PULLUP_ENABLE_POS)) /**< CFG_EXT_PULLUP_ENABLE Mask */

#define MXC_F_OWM_REVA_CFG_SINGLE_BIT_MODE_POS         5 /**< CFG_SINGLE_BIT_MODE Position */
#define MXC_F_OWM_REVA_CFG_SINGLE_BIT_MODE             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_SINGLE_BIT_MODE_POS)) /**< CFG_SINGLE_BIT_MODE Mask */

#define MXC_F_OWM_REVA_CFG_OVERDRIVE_POS               6 /**< CFG_OVERDRIVE Position */
#define MXC_F_OWM_REVA_CFG_OVERDRIVE                   ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_OVERDRIVE_POS)) /**< CFG_OVERDRIVE Mask */

#define MXC_F_OWM_REVA_CFG_INT_PULLUP_ENABLE_POS       7 /**< CFG_INT_PULLUP_ENABLE Position */
#define MXC_F_OWM_REVA_CFG_INT_PULLUP_ENABLE           ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CFG_INT_PULLUP_ENABLE_POS)) /**< CFG_INT_PULLUP_ENABLE Mask */

/**@} end of group OWM_REVA_CFG_Register */

/**
 * @ingroup  owm_reva_registers
 * @defgroup OWM_REVA_CLK_DIV_1US OWM_REVA_CLK_DIV_1US
 * @brief    1-Wire Master Clock Divisor.
 * @{
 */
#define MXC_F_OWM_REVA_CLK_DIV_1US_DIVISOR_POS         0 /**< CLK_DIV_1US_DIVISOR Position */
#define MXC_F_OWM_REVA_CLK_DIV_1US_DIVISOR             ((uint32_t)(0xFFUL << MXC_F_OWM_REVA_CLK_DIV_1US_DIVISOR_POS)) /**< CLK_DIV_1US_DIVISOR Mask */

/**@} end of group OWM_REVA_CLK_DIV_1US_Register */

/**
 * @ingroup  owm_reva_registers
 * @defgroup OWM_REVA_CTRL_STAT OWM_REVA_CTRL_STAT
 * @brief    1-Wire Master Control/Status.
 * @{
 */
#define MXC_F_OWM_REVA_CTRL_STAT_START_OW_RESET_POS    0 /**< CTRL_STAT_START_OW_RESET Position */
#define MXC_F_OWM_REVA_CTRL_STAT_START_OW_RESET        ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CTRL_STAT_START_OW_RESET_POS)) /**< CTRL_STAT_START_OW_RESET Mask */

#define MXC_F_OWM_REVA_CTRL_STAT_SRA_MODE_POS          1 /**< CTRL_STAT_SRA_MODE Position */
#define MXC_F_OWM_REVA_CTRL_STAT_SRA_MODE              ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CTRL_STAT_SRA_MODE_POS)) /**< CTRL_STAT_SRA_MODE Mask */

#define MXC_F_OWM_REVA_CTRL_STAT_BIT_BANG_OE_POS       2 /**< CTRL_STAT_BIT_BANG_OE Position */
#define MXC_F_OWM_REVA_CTRL_STAT_BIT_BANG_OE           ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CTRL_STAT_BIT_BANG_OE_POS)) /**< CTRL_STAT_BIT_BANG_OE Mask */

#define MXC_F_OWM_REVA_CTRL_STAT_OW_INPUT_POS          3 /**< CTRL_STAT_OW_INPUT Position */
#define MXC_F_OWM_REVA_CTRL_STAT_OW_INPUT              ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CTRL_STAT_OW_INPUT_POS)) /**< CTRL_STAT_OW_INPUT Mask */

#define MXC_F_OWM_REVA_CTRL_STAT_OD_SPEC_MODE_POS      4 /**< CTRL_STAT_OD_SPEC_MODE Position */
#define MXC_F_OWM_REVA_CTRL_STAT_OD_SPEC_MODE          ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CTRL_STAT_OD_SPEC_MODE_POS)) /**< CTRL_STAT_OD_SPEC_MODE Mask */

#define MXC_F_OWM_REVA_CTRL_STAT_PRESENCE_DETECT_POS   5 /**< CTRL_STAT_PRESENCE_DETECT Position */
#define MXC_F_OWM_REVA_CTRL_STAT_PRESENCE_DETECT       ((uint32_t)(0x1UL << MXC_F_OWM_REVA_CTRL_STAT_PRESENCE_DETECT_POS)) /**< CTRL_STAT_PRESENCE_DETECT Mask */

/**@} end of group OWM_REVA_CTRL_STAT_Register */

/**
 * @ingroup  owm_reva_registers
 * @defgroup OWM_REVA_DATA OWM_REVA_DATA
 * @brief    1-Wire Master Data Buffer.
 * @{
 */
#define MXC_F_OWM_REVA_DATA_TX_RX_POS                  0 /**< DATA_TX_RX Position */
#define MXC_F_OWM_REVA_DATA_TX_RX                      ((uint32_t)(0xFFUL << MXC_F_OWM_REVA_DATA_TX_RX_POS)) /**< DATA_TX_RX Mask */

/**@} end of group OWM_REVA_DATA_Register */

/**
 * @ingroup  owm_reva_registers
 * @defgroup OWM_REVA_INTFL OWM_REVA_INTFL
 * @brief    1-Wire Master Interrupt Flags.
 * @{
 */
#define MXC_F_OWM_REVA_INTFL_OW_RESET_DONE_POS         0 /**< INTFL_OW_RESET_DONE Position */
#define MXC_F_OWM_REVA_INTFL_OW_RESET_DONE             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTFL_OW_RESET_DONE_POS)) /**< INTFL_OW_RESET_DONE Mask */

#define MXC_F_OWM_REVA_INTFL_TX_DATA_EMPTY_POS         1 /**< INTFL_TX_DATA_EMPTY Position */
#define MXC_F_OWM_REVA_INTFL_TX_DATA_EMPTY             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTFL_TX_DATA_EMPTY_POS)) /**< INTFL_TX_DATA_EMPTY Mask */

#define MXC_F_OWM_REVA_INTFL_RX_DATA_READY_POS         2 /**< INTFL_RX_DATA_READY Position */
#define MXC_F_OWM_REVA_INTFL_RX_DATA_READY             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTFL_RX_DATA_READY_POS)) /**< INTFL_RX_DATA_READY Mask */

#define MXC_F_OWM_REVA_INTFL_LINE_SHORT_POS            3 /**< INTFL_LINE_SHORT Position */
#define MXC_F_OWM_REVA_INTFL_LINE_SHORT                ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTFL_LINE_SHORT_POS)) /**< INTFL_LINE_SHORT Mask */

#define MXC_F_OWM_REVA_INTFL_LINE_LOW_POS              4 /**< INTFL_LINE_LOW Position */
#define MXC_F_OWM_REVA_INTFL_LINE_LOW                  ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTFL_LINE_LOW_POS)) /**< INTFL_LINE_LOW Mask */

/**@} end of group OWM_REVA_INTFL_Register */

/**
 * @ingroup  owm_reva_registers
 * @defgroup OWM_REVA_INTEN OWM_REVA_INTEN
 * @brief    1-Wire Master Interrupt Enables.
 * @{
 */
#define MXC_F_OWM_REVA_INTEN_OW_RESET_DONE_POS         0 /**< INTEN_OW_RESET_DONE Position */
#define MXC_F_OWM_REVA_INTEN_OW_RESET_DONE             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTEN_OW_RESET_DONE_POS)) /**< INTEN_OW_RESET_DONE Mask */

#define MXC_F_OWM_REVA_INTEN_TX_DATA_EMPTY_POS         1 /**< INTEN_TX_DATA_EMPTY Position */
#define MXC_F_OWM_REVA_INTEN_TX_DATA_EMPTY             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTEN_TX_DATA_EMPTY_POS)) /**< INTEN_TX_DATA_EMPTY Mask */

#define MXC_F_OWM_REVA_INTEN_RX_DATA_READY_POS         2 /**< INTEN_RX_DATA_READY Position */
#define MXC_F_OWM_REVA_INTEN_RX_DATA_READY             ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTEN_RX_DATA_READY_POS)) /**< INTEN_RX_DATA_READY Mask */

#define MXC_F_OWM_REVA_INTEN_LINE_SHORT_POS            3 /**< INTEN_LINE_SHORT Position */
#define MXC_F_OWM_REVA_INTEN_LINE_SHORT                ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTEN_LINE_SHORT_POS)) /**< INTEN_LINE_SHORT Mask */

#define MXC_F_OWM_REVA_INTEN_LINE_LOW_POS              4 /**< INTEN_LINE_LOW Position */
#define MXC_F_OWM_REVA_INTEN_LINE_LOW                  ((uint32_t)(0x1UL << MXC_F_OWM_REVA_INTEN_LINE_LOW_POS)) /**< INTEN_LINE_LOW Mask */

/**@} end of group OWM_REVA_INTEN_Register */

#ifdef __cplusplus
}
#endif

#endif  // LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_REGS_H_

