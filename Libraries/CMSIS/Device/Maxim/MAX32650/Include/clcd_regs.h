/**
 * @file    clcd_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the CLCD Peripheral Module.
 * @note    This file is @generated.
 * @ingroup clcd_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_CLCD_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_CLCD_REGS_H_

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
 * @ingroup     clcd
 * @defgroup    clcd_registers CLCD_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the CLCD Peripheral Module.
 * @details     Color LCD Controller
 */

/**
 * @ingroup clcd_registers
 * Structure type to access the CLCD Registers.
 */
typedef struct {
    __IO uint32_t clk_ctrl;             /**< <tt>\b 0x000:</tt> CLCD CLK_CTRL Register */
    __IO uint32_t vtim_0;               /**< <tt>\b 0x004:</tt> CLCD VTIM_0 Register */
    __IO uint32_t vtim_1;               /**< <tt>\b 0x008:</tt> CLCD VTIM_1 Register */
    __IO uint32_t htim;                 /**< <tt>\b 0x00C:</tt> CLCD HTIM Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x010:</tt> CLCD CTRL Register */
    __R  uint32_t rsv_0x14;
    __IO uint32_t frbuf;                /**< <tt>\b 0x18:</tt> CLCD FRBUF Register */
    __R  uint32_t rsv_0x1c;
    __IO uint32_t int_en;               /**< <tt>\b 0x020:</tt> CLCD INT_EN Register */
    __IO uint32_t int_stat;             /**< <tt>\b 0x024:</tt> CLCD INT_STAT Register */
    __R  uint32_t rsv_0x28_0x3ff[246];
    __IO uint32_t palette_ram[256];     /**< <tt>\b 0x400:</tt> CLCD PALETTE_RAM Register */
} mxc_clcd_regs_t;

/* Register offsets for module CLCD */
/**
 * @ingroup    clcd_registers
 * @defgroup   CLCD_Register_Offsets Register Offsets
 * @brief      CLCD Peripheral Register Offsets from the CLCD Base Peripheral Address.
 * @{
 */
#define MXC_R_CLCD_CLK_CTRL                ((uint32_t)0x00000000UL) /**< Offset from CLCD Base Address: <tt> 0x0000</tt> */
#define MXC_R_CLCD_VTIM_0                  ((uint32_t)0x00000004UL) /**< Offset from CLCD Base Address: <tt> 0x0004</tt> */
#define MXC_R_CLCD_VTIM_1                  ((uint32_t)0x00000008UL) /**< Offset from CLCD Base Address: <tt> 0x0008</tt> */
#define MXC_R_CLCD_HTIM                    ((uint32_t)0x0000000CUL) /**< Offset from CLCD Base Address: <tt> 0x000C</tt> */
#define MXC_R_CLCD_CTRL                    ((uint32_t)0x00000010UL) /**< Offset from CLCD Base Address: <tt> 0x0010</tt> */
#define MXC_R_CLCD_FRBUF                   ((uint32_t)0x00000018UL) /**< Offset from CLCD Base Address: <tt> 0x0018</tt> */
#define MXC_R_CLCD_INT_EN                  ((uint32_t)0x00000020UL) /**< Offset from CLCD Base Address: <tt> 0x0020</tt> */
#define MXC_R_CLCD_INT_STAT                ((uint32_t)0x00000024UL) /**< Offset from CLCD Base Address: <tt> 0x0024</tt> */
#define MXC_R_CLCD_PALETTE_RAM             ((uint32_t)0x00000400UL) /**< Offset from CLCD Base Address: <tt> 0x0400</tt> */
/**@} end of group clcd_registers */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_CLK_CTRL CLCD_CLK_CTRL
 * @brief    LCD Clock Control Register
 * @{
 */
#define MXC_F_CLCD_CLK_CTRL_LCD_CLKDIV_POS             0 /**< CLK_CTRL_LCD_CLKDIV Position */
#define MXC_F_CLCD_CLK_CTRL_LCD_CLKDIV                 ((uint32_t)(0xFFUL << MXC_F_CLCD_CLK_CTRL_LCD_CLKDIV_POS)) /**< CLK_CTRL_LCD_CLKDIV Mask */

#define MXC_F_CLCD_CLK_CTRL_STN_AC_BIAS_POS            8 /**< CLK_CTRL_STN_AC_BIAS Position */
#define MXC_F_CLCD_CLK_CTRL_STN_AC_BIAS                ((uint32_t)(0xFFUL << MXC_F_CLCD_CLK_CTRL_STN_AC_BIAS_POS)) /**< CLK_CTRL_STN_AC_BIAS Mask */

#define MXC_F_CLCD_CLK_CTRL_VDEN_POL_POS               16 /**< CLK_CTRL_VDEN_POL Position */
#define MXC_F_CLCD_CLK_CTRL_VDEN_POL                   ((uint32_t)(0x1UL << MXC_F_CLCD_CLK_CTRL_VDEN_POL_POS)) /**< CLK_CTRL_VDEN_POL Mask */
#define MXC_V_CLCD_CLK_CTRL_VDEN_POL_ACTIVELO          ((uint32_t)0x0UL) /**< CLK_CTRL_VDEN_POL_ACTIVELO Value */
#define MXC_S_CLCD_CLK_CTRL_VDEN_POL_ACTIVELO          (MXC_V_CLCD_CLK_CTRL_VDEN_POL_ACTIVELO << MXC_F_CLCD_CLK_CTRL_VDEN_POL_POS) /**< CLK_CTRL_VDEN_POL_ACTIVELO Setting */
#define MXC_V_CLCD_CLK_CTRL_VDEN_POL_ACTIVEHI          ((uint32_t)0x1UL) /**< CLK_CTRL_VDEN_POL_ACTIVEHI Value */
#define MXC_S_CLCD_CLK_CTRL_VDEN_POL_ACTIVEHI          (MXC_V_CLCD_CLK_CTRL_VDEN_POL_ACTIVEHI << MXC_F_CLCD_CLK_CTRL_VDEN_POL_POS) /**< CLK_CTRL_VDEN_POL_ACTIVEHI Setting */

#define MXC_F_CLCD_CLK_CTRL_VSYNC_POL_POS              17 /**< CLK_CTRL_VSYNC_POL Position */
#define MXC_F_CLCD_CLK_CTRL_VSYNC_POL                  ((uint32_t)(0x1UL << MXC_F_CLCD_CLK_CTRL_VSYNC_POL_POS)) /**< CLK_CTRL_VSYNC_POL Mask */
#define MXC_V_CLCD_CLK_CTRL_VSYNC_POL_ACTIVELO         ((uint32_t)0x0UL) /**< CLK_CTRL_VSYNC_POL_ACTIVELO Value */
#define MXC_S_CLCD_CLK_CTRL_VSYNC_POL_ACTIVELO         (MXC_V_CLCD_CLK_CTRL_VSYNC_POL_ACTIVELO << MXC_F_CLCD_CLK_CTRL_VSYNC_POL_POS) /**< CLK_CTRL_VSYNC_POL_ACTIVELO Setting */
#define MXC_V_CLCD_CLK_CTRL_VSYNC_POL_ACTIVEHI         ((uint32_t)0x1UL) /**< CLK_CTRL_VSYNC_POL_ACTIVEHI Value */
#define MXC_S_CLCD_CLK_CTRL_VSYNC_POL_ACTIVEHI         (MXC_V_CLCD_CLK_CTRL_VSYNC_POL_ACTIVEHI << MXC_F_CLCD_CLK_CTRL_VSYNC_POL_POS) /**< CLK_CTRL_VSYNC_POL_ACTIVEHI Setting */

#define MXC_F_CLCD_CLK_CTRL_HSYNC_POL_POS              18 /**< CLK_CTRL_HSYNC_POL Position */
#define MXC_F_CLCD_CLK_CTRL_HSYNC_POL                  ((uint32_t)(0x1UL << MXC_F_CLCD_CLK_CTRL_HSYNC_POL_POS)) /**< CLK_CTRL_HSYNC_POL Mask */
#define MXC_V_CLCD_CLK_CTRL_HSYNC_POL_ACTIVELO         ((uint32_t)0x0UL) /**< CLK_CTRL_HSYNC_POL_ACTIVELO Value */
#define MXC_S_CLCD_CLK_CTRL_HSYNC_POL_ACTIVELO         (MXC_V_CLCD_CLK_CTRL_HSYNC_POL_ACTIVELO << MXC_F_CLCD_CLK_CTRL_HSYNC_POL_POS) /**< CLK_CTRL_HSYNC_POL_ACTIVELO Setting */
#define MXC_V_CLCD_CLK_CTRL_HSYNC_POL_ACTIVEHI         ((uint32_t)0x1UL) /**< CLK_CTRL_HSYNC_POL_ACTIVEHI Value */
#define MXC_S_CLCD_CLK_CTRL_HSYNC_POL_ACTIVEHI         (MXC_V_CLCD_CLK_CTRL_HSYNC_POL_ACTIVEHI << MXC_F_CLCD_CLK_CTRL_HSYNC_POL_POS) /**< CLK_CTRL_HSYNC_POL_ACTIVEHI Setting */

#define MXC_F_CLCD_CLK_CTRL_CLK_EDGE_SEL_POS           19 /**< CLK_CTRL_CLK_EDGE_SEL Position */
#define MXC_F_CLCD_CLK_CTRL_CLK_EDGE_SEL               ((uint32_t)(0x1UL << MXC_F_CLCD_CLK_CTRL_CLK_EDGE_SEL_POS)) /**< CLK_CTRL_CLK_EDGE_SEL Mask */
#define MXC_V_CLCD_CLK_CTRL_CLK_EDGE_SEL_RISING        ((uint32_t)0x0UL) /**< CLK_CTRL_CLK_EDGE_SEL_RISING Value */
#define MXC_S_CLCD_CLK_CTRL_CLK_EDGE_SEL_RISING        (MXC_V_CLCD_CLK_CTRL_CLK_EDGE_SEL_RISING << MXC_F_CLCD_CLK_CTRL_CLK_EDGE_SEL_POS) /**< CLK_CTRL_CLK_EDGE_SEL_RISING Setting */
#define MXC_V_CLCD_CLK_CTRL_CLK_EDGE_SEL_FALLING       ((uint32_t)0x1UL) /**< CLK_CTRL_CLK_EDGE_SEL_FALLING Value */
#define MXC_S_CLCD_CLK_CTRL_CLK_EDGE_SEL_FALLING       (MXC_V_CLCD_CLK_CTRL_CLK_EDGE_SEL_FALLING << MXC_F_CLCD_CLK_CTRL_CLK_EDGE_SEL_POS) /**< CLK_CTRL_CLK_EDGE_SEL_FALLING Setting */

#define MXC_F_CLCD_CLK_CTRL_CLK_ACTIVE_POS             20 /**< CLK_CTRL_CLK_ACTIVE Position */
#define MXC_F_CLCD_CLK_CTRL_CLK_ACTIVE                 ((uint32_t)(0x1UL << MXC_F_CLCD_CLK_CTRL_CLK_ACTIVE_POS)) /**< CLK_CTRL_CLK_ACTIVE Mask */
#define MXC_V_CLCD_CLK_CTRL_CLK_ACTIVE_ALWAYS          ((uint32_t)0x0UL) /**< CLK_CTRL_CLK_ACTIVE_ALWAYS Value */
#define MXC_S_CLCD_CLK_CTRL_CLK_ACTIVE_ALWAYS          (MXC_V_CLCD_CLK_CTRL_CLK_ACTIVE_ALWAYS << MXC_F_CLCD_CLK_CTRL_CLK_ACTIVE_POS) /**< CLK_CTRL_CLK_ACTIVE_ALWAYS Setting */
#define MXC_V_CLCD_CLK_CTRL_CLK_ACTIVE_ONDATA          ((uint32_t)0x1UL) /**< CLK_CTRL_CLK_ACTIVE_ONDATA Value */
#define MXC_S_CLCD_CLK_CTRL_CLK_ACTIVE_ONDATA          (MXC_V_CLCD_CLK_CTRL_CLK_ACTIVE_ONDATA << MXC_F_CLCD_CLK_CTRL_CLK_ACTIVE_POS) /**< CLK_CTRL_CLK_ACTIVE_ONDATA Setting */

/**@} end of group CLCD_CLK_CTRL_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_VTIM_0 CLCD_VTIM_0
 * @brief    LCD Vertical Timing 0 Register
 * @{
 */
#define MXC_F_CLCD_VTIM_0_VLINES_POS                   0 /**< VTIM_0_VLINES Position */
#define MXC_F_CLCD_VTIM_0_VLINES                       ((uint32_t)(0xFFUL << MXC_F_CLCD_VTIM_0_VLINES_POS)) /**< VTIM_0_VLINES Mask */

#define MXC_F_CLCD_VTIM_0_VBP_WIDTH_POS                16 /**< VTIM_0_VBP_WIDTH Position */
#define MXC_F_CLCD_VTIM_0_VBP_WIDTH                    ((uint32_t)(0xFFUL << MXC_F_CLCD_VTIM_0_VBP_WIDTH_POS)) /**< VTIM_0_VBP_WIDTH Mask */

/**@} end of group CLCD_VTIM_0_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_VTIM_1 CLCD_VTIM_1
 * @brief    LCD Vertical Timing 1 Register
 * @{
 */
#define MXC_F_CLCD_VTIM_1_VSYNC_WIDTH_POS              0 /**< VTIM_1_VSYNC_WIDTH Position */
#define MXC_F_CLCD_VTIM_1_VSYNC_WIDTH                  ((uint32_t)(0xFFUL << MXC_F_CLCD_VTIM_1_VSYNC_WIDTH_POS)) /**< VTIM_1_VSYNC_WIDTH Mask */

#define MXC_F_CLCD_VTIM_1_VFP_WIDTH_POS                16 /**< VTIM_1_VFP_WIDTH Position */
#define MXC_F_CLCD_VTIM_1_VFP_WIDTH                    ((uint32_t)(0xFFUL << MXC_F_CLCD_VTIM_1_VFP_WIDTH_POS)) /**< VTIM_1_VFP_WIDTH Mask */

/**@} end of group CLCD_VTIM_1_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_HTIM CLCD_HTIM
 * @brief    LCD Horizontal Timing Register.
 * @{
 */
#define MXC_F_CLCD_HTIM_HSYNC_WIDTH_POS                0 /**< HTIM_HSYNC_WIDTH Position */
#define MXC_F_CLCD_HTIM_HSYNC_WIDTH                    ((uint32_t)(0xFFUL << MXC_F_CLCD_HTIM_HSYNC_WIDTH_POS)) /**< HTIM_HSYNC_WIDTH Mask */

#define MXC_F_CLCD_HTIM_HFP_WIDTH_POS                  8 /**< HTIM_HFP_WIDTH Position */
#define MXC_F_CLCD_HTIM_HFP_WIDTH                      ((uint32_t)(0xFFUL << MXC_F_CLCD_HTIM_HFP_WIDTH_POS)) /**< HTIM_HFP_WIDTH Mask */

#define MXC_F_CLCD_HTIM_HSIZE_INDEX_POS                16 /**< HTIM_HSIZE_INDEX Position */
#define MXC_F_CLCD_HTIM_HSIZE_INDEX                    ((uint32_t)(0xFFUL << MXC_F_CLCD_HTIM_HSIZE_INDEX_POS)) /**< HTIM_HSIZE_INDEX Mask */

#define MXC_F_CLCD_HTIM_HBP_WIDTH_POS                  24 /**< HTIM_HBP_WIDTH Position */
#define MXC_F_CLCD_HTIM_HBP_WIDTH                      ((uint32_t)(0xFFUL << MXC_F_CLCD_HTIM_HBP_WIDTH_POS)) /**< HTIM_HBP_WIDTH Mask */

/**@} end of group CLCD_HTIM_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_CTRL CLCD_CTRL
 * @brief    LCD Control Register
 * @{
 */
#define MXC_F_CLCD_CTRL_CLCD_ENABLE_POS                0 /**< CTRL_CLCD_ENABLE Position */
#define MXC_F_CLCD_CTRL_CLCD_ENABLE                    ((uint32_t)(0x1UL << MXC_F_CLCD_CTRL_CLCD_ENABLE_POS)) /**< CTRL_CLCD_ENABLE Mask */
#define MXC_V_CLCD_CTRL_CLCD_ENABLE_DIS                ((uint32_t)0x0UL) /**< CTRL_CLCD_ENABLE_DIS Value */
#define MXC_S_CLCD_CTRL_CLCD_ENABLE_DIS                (MXC_V_CLCD_CTRL_CLCD_ENABLE_DIS << MXC_F_CLCD_CTRL_CLCD_ENABLE_POS) /**< CTRL_CLCD_ENABLE_DIS Setting */
#define MXC_V_CLCD_CTRL_CLCD_ENABLE_EN                 ((uint32_t)0x1UL) /**< CTRL_CLCD_ENABLE_EN Value */
#define MXC_S_CLCD_CTRL_CLCD_ENABLE_EN                 (MXC_V_CLCD_CTRL_CLCD_ENABLE_EN << MXC_F_CLCD_CTRL_CLCD_ENABLE_POS) /**< CTRL_CLCD_ENABLE_EN Setting */

#define MXC_F_CLCD_CTRL_VCI_SEL_POS                    1 /**< CTRL_VCI_SEL Position */
#define MXC_F_CLCD_CTRL_VCI_SEL                        ((uint32_t)(0x3UL << MXC_F_CLCD_CTRL_VCI_SEL_POS)) /**< CTRL_VCI_SEL Mask */
#define MXC_V_CLCD_CTRL_VCI_SEL_ON_VSYNC               ((uint32_t)0x0UL) /**< CTRL_VCI_SEL_ON_VSYNC Value */
#define MXC_S_CLCD_CTRL_VCI_SEL_ON_VSYNC               (MXC_V_CLCD_CTRL_VCI_SEL_ON_VSYNC << MXC_F_CLCD_CTRL_VCI_SEL_POS) /**< CTRL_VCI_SEL_ON_VSYNC Setting */
#define MXC_V_CLCD_CTRL_VCI_SEL_ON_VBP                 ((uint32_t)0x1UL) /**< CTRL_VCI_SEL_ON_VBP Value */
#define MXC_S_CLCD_CTRL_VCI_SEL_ON_VBP                 (MXC_V_CLCD_CTRL_VCI_SEL_ON_VBP << MXC_F_CLCD_CTRL_VCI_SEL_POS) /**< CTRL_VCI_SEL_ON_VBP Setting */
#define MXC_V_CLCD_CTRL_VCI_SEL_ON_VDEN                ((uint32_t)0x2UL) /**< CTRL_VCI_SEL_ON_VDEN Value */
#define MXC_S_CLCD_CTRL_VCI_SEL_ON_VDEN                (MXC_V_CLCD_CTRL_VCI_SEL_ON_VDEN << MXC_F_CLCD_CTRL_VCI_SEL_POS) /**< CTRL_VCI_SEL_ON_VDEN Setting */
#define MXC_V_CLCD_CTRL_VCI_SEL_ON_VFP                 ((uint32_t)0x3UL) /**< CTRL_VCI_SEL_ON_VFP Value */
#define MXC_S_CLCD_CTRL_VCI_SEL_ON_VFP                 (MXC_V_CLCD_CTRL_VCI_SEL_ON_VFP << MXC_F_CLCD_CTRL_VCI_SEL_POS) /**< CTRL_VCI_SEL_ON_VFP Setting */

#define MXC_F_CLCD_CTRL_DISPTYPE_POS                   4 /**< CTRL_DISPTYPE Position */
#define MXC_F_CLCD_CTRL_DISPTYPE                       ((uint32_t)(0xFUL << MXC_F_CLCD_CTRL_DISPTYPE_POS)) /**< CTRL_DISPTYPE Mask */
#define MXC_V_CLCD_CTRL_DISPTYPE_8BITCOLORSTN          ((uint32_t)0x4UL) /**< CTRL_DISPTYPE_8BITCOLORSTN Value */
#define MXC_S_CLCD_CTRL_DISPTYPE_8BITCOLORSTN          (MXC_V_CLCD_CTRL_DISPTYPE_8BITCOLORSTN << MXC_F_CLCD_CTRL_DISPTYPE_POS) /**< CTRL_DISPTYPE_8BITCOLORSTN Setting */
#define MXC_V_CLCD_CTRL_DISPTYPE_TFT                   ((uint32_t)0x8UL) /**< CTRL_DISPTYPE_TFT Value */
#define MXC_S_CLCD_CTRL_DISPTYPE_TFT                   (MXC_V_CLCD_CTRL_DISPTYPE_TFT << MXC_F_CLCD_CTRL_DISPTYPE_POS) /**< CTRL_DISPTYPE_TFT Setting */

#define MXC_F_CLCD_CTRL_BPP_POS                        8 /**< CTRL_BPP Position */
#define MXC_F_CLCD_CTRL_BPP                            ((uint32_t)(0x7UL << MXC_F_CLCD_CTRL_BPP_POS)) /**< CTRL_BPP Mask */
#define MXC_V_CLCD_CTRL_BPP_BPP1                       ((uint32_t)0x0UL) /**< CTRL_BPP_BPP1 Value */
#define MXC_S_CLCD_CTRL_BPP_BPP1                       (MXC_V_CLCD_CTRL_BPP_BPP1 << MXC_F_CLCD_CTRL_BPP_POS) /**< CTRL_BPP_BPP1 Setting */
#define MXC_V_CLCD_CTRL_BPP_BPP2                       ((uint32_t)0x1UL) /**< CTRL_BPP_BPP2 Value */
#define MXC_S_CLCD_CTRL_BPP_BPP2                       (MXC_V_CLCD_CTRL_BPP_BPP2 << MXC_F_CLCD_CTRL_BPP_POS) /**< CTRL_BPP_BPP2 Setting */
#define MXC_V_CLCD_CTRL_BPP_BPP4                       ((uint32_t)0x2UL) /**< CTRL_BPP_BPP4 Value */
#define MXC_S_CLCD_CTRL_BPP_BPP4                       (MXC_V_CLCD_CTRL_BPP_BPP4 << MXC_F_CLCD_CTRL_BPP_POS) /**< CTRL_BPP_BPP4 Setting */
#define MXC_V_CLCD_CTRL_BPP_BPP8                       ((uint32_t)0x3UL) /**< CTRL_BPP_BPP8 Value */
#define MXC_S_CLCD_CTRL_BPP_BPP8                       (MXC_V_CLCD_CTRL_BPP_BPP8 << MXC_F_CLCD_CTRL_BPP_POS) /**< CTRL_BPP_BPP8 Setting */
#define MXC_V_CLCD_CTRL_BPP_BPP16                      ((uint32_t)0x4UL) /**< CTRL_BPP_BPP16 Value */
#define MXC_S_CLCD_CTRL_BPP_BPP16                      (MXC_V_CLCD_CTRL_BPP_BPP16 << MXC_F_CLCD_CTRL_BPP_POS) /**< CTRL_BPP_BPP16 Setting */
#define MXC_V_CLCD_CTRL_BPP_BPP24                      ((uint32_t)0x5UL) /**< CTRL_BPP_BPP24 Value */
#define MXC_S_CLCD_CTRL_BPP_BPP24                      (MXC_V_CLCD_CTRL_BPP_BPP24 << MXC_F_CLCD_CTRL_BPP_POS) /**< CTRL_BPP_BPP24 Setting */

#define MXC_F_CLCD_CTRL_MODE565_POS                    11 /**< CTRL_MODE565 Position */
#define MXC_F_CLCD_CTRL_MODE565                        ((uint32_t)(0x1UL << MXC_F_CLCD_CTRL_MODE565_POS)) /**< CTRL_MODE565 Mask */
#define MXC_V_CLCD_CTRL_MODE565_BGR556                 ((uint32_t)0x0UL) /**< CTRL_MODE565_BGR556 Value */
#define MXC_S_CLCD_CTRL_MODE565_BGR556                 (MXC_V_CLCD_CTRL_MODE565_BGR556 << MXC_F_CLCD_CTRL_MODE565_POS) /**< CTRL_MODE565_BGR556 Setting */
#define MXC_V_CLCD_CTRL_MODE565_RGB565                 ((uint32_t)0x1UL) /**< CTRL_MODE565_RGB565 Value */
#define MXC_S_CLCD_CTRL_MODE565_RGB565                 (MXC_V_CLCD_CTRL_MODE565_RGB565 << MXC_F_CLCD_CTRL_MODE565_POS) /**< CTRL_MODE565_RGB565 Setting */

#define MXC_F_CLCD_CTRL_ENDIAN_POS                     12 /**< CTRL_ENDIAN Position */
#define MXC_F_CLCD_CTRL_ENDIAN                         ((uint32_t)(0x3UL << MXC_F_CLCD_CTRL_ENDIAN_POS)) /**< CTRL_ENDIAN Mask */
#define MXC_V_CLCD_CTRL_ENDIAN_LBLP                    ((uint32_t)0x0UL) /**< CTRL_ENDIAN_LBLP Value */
#define MXC_S_CLCD_CTRL_ENDIAN_LBLP                    (MXC_V_CLCD_CTRL_ENDIAN_LBLP << MXC_F_CLCD_CTRL_ENDIAN_POS) /**< CTRL_ENDIAN_LBLP Setting */
#define MXC_V_CLCD_CTRL_ENDIAN_BBBP                    ((uint32_t)0x1UL) /**< CTRL_ENDIAN_BBBP Value */
#define MXC_S_CLCD_CTRL_ENDIAN_BBBP                    (MXC_V_CLCD_CTRL_ENDIAN_BBBP << MXC_F_CLCD_CTRL_ENDIAN_POS) /**< CTRL_ENDIAN_BBBP Setting */
#define MXC_V_CLCD_CTRL_ENDIAN_LBBP                    ((uint32_t)0x2UL) /**< CTRL_ENDIAN_LBBP Value */
#define MXC_S_CLCD_CTRL_ENDIAN_LBBP                    (MXC_V_CLCD_CTRL_ENDIAN_LBBP << MXC_F_CLCD_CTRL_ENDIAN_POS) /**< CTRL_ENDIAN_LBBP Setting */
#define MXC_V_CLCD_CTRL_ENDIAN_RFU                     ((uint32_t)0x3UL) /**< CTRL_ENDIAN_RFU Value */
#define MXC_S_CLCD_CTRL_ENDIAN_RFU                     (MXC_V_CLCD_CTRL_ENDIAN_RFU << MXC_F_CLCD_CTRL_ENDIAN_POS) /**< CTRL_ENDIAN_RFU Setting */

#define MXC_F_CLCD_CTRL_COMPACT_24B_POS                15 /**< CTRL_COMPACT_24B Position */
#define MXC_F_CLCD_CTRL_COMPACT_24B                    ((uint32_t)(0x1UL << MXC_F_CLCD_CTRL_COMPACT_24B_POS)) /**< CTRL_COMPACT_24B Mask */
#define MXC_V_CLCD_CTRL_COMPACT_24B_1_PFR              ((uint32_t)0x0UL) /**< CTRL_COMPACT_24B_1_PFR Value */
#define MXC_S_CLCD_CTRL_COMPACT_24B_1_PFR              (MXC_V_CLCD_CTRL_COMPACT_24B_1_PFR << MXC_F_CLCD_CTRL_COMPACT_24B_POS) /**< CTRL_COMPACT_24B_1_PFR Setting */
#define MXC_V_CLCD_CTRL_COMPACT_24B_1ANDA3RD_PFR       ((uint32_t)0x1UL) /**< CTRL_COMPACT_24B_1ANDA3RD_PFR Value */
#define MXC_S_CLCD_CTRL_COMPACT_24B_1ANDA3RD_PFR       (MXC_V_CLCD_CTRL_COMPACT_24B_1ANDA3RD_PFR << MXC_F_CLCD_CTRL_COMPACT_24B_POS) /**< CTRL_COMPACT_24B_1ANDA3RD_PFR Setting */

#define MXC_F_CLCD_CTRL_BURST_SIZE_POS                 19 /**< CTRL_BURST_SIZE Position */
#define MXC_F_CLCD_CTRL_BURST_SIZE                     ((uint32_t)(0x3UL << MXC_F_CLCD_CTRL_BURST_SIZE_POS)) /**< CTRL_BURST_SIZE Mask */
#define MXC_V_CLCD_CTRL_BURST_SIZE_4WORDS              ((uint32_t)0x0UL) /**< CTRL_BURST_SIZE_4WORDS Value */
#define MXC_S_CLCD_CTRL_BURST_SIZE_4WORDS              (MXC_V_CLCD_CTRL_BURST_SIZE_4WORDS << MXC_F_CLCD_CTRL_BURST_SIZE_POS) /**< CTRL_BURST_SIZE_4WORDS Setting */
#define MXC_V_CLCD_CTRL_BURST_SIZE_8WORDS              ((uint32_t)0x1UL) /**< CTRL_BURST_SIZE_8WORDS Value */
#define MXC_S_CLCD_CTRL_BURST_SIZE_8WORDS              (MXC_V_CLCD_CTRL_BURST_SIZE_8WORDS << MXC_F_CLCD_CTRL_BURST_SIZE_POS) /**< CTRL_BURST_SIZE_8WORDS Setting */
#define MXC_V_CLCD_CTRL_BURST_SIZE_16WORDS             ((uint32_t)0x2UL) /**< CTRL_BURST_SIZE_16WORDS Value */
#define MXC_S_CLCD_CTRL_BURST_SIZE_16WORDS             (MXC_V_CLCD_CTRL_BURST_SIZE_16WORDS << MXC_F_CLCD_CTRL_BURST_SIZE_POS) /**< CTRL_BURST_SIZE_16WORDS Setting */

#define MXC_F_CLCD_CTRL_LEND_POL_POS                   21 /**< CTRL_LEND_POL Position */
#define MXC_F_CLCD_CTRL_LEND_POL                       ((uint32_t)(0x1UL << MXC_F_CLCD_CTRL_LEND_POL_POS)) /**< CTRL_LEND_POL Mask */
#define MXC_V_CLCD_CTRL_LEND_POL_ACTIVELO              ((uint32_t)0x0UL) /**< CTRL_LEND_POL_ACTIVELO Value */
#define MXC_S_CLCD_CTRL_LEND_POL_ACTIVELO              (MXC_V_CLCD_CTRL_LEND_POL_ACTIVELO << MXC_F_CLCD_CTRL_LEND_POL_POS) /**< CTRL_LEND_POL_ACTIVELO Setting */
#define MXC_V_CLCD_CTRL_LEND_POL_ACTIVEHI              ((uint32_t)0x1UL) /**< CTRL_LEND_POL_ACTIVEHI Value */
#define MXC_S_CLCD_CTRL_LEND_POL_ACTIVEHI              (MXC_V_CLCD_CTRL_LEND_POL_ACTIVEHI << MXC_F_CLCD_CTRL_LEND_POL_POS) /**< CTRL_LEND_POL_ACTIVEHI Setting */

#define MXC_F_CLCD_CTRL_PWR_ENABLE_POS                 22 /**< CTRL_PWR_ENABLE Position */
#define MXC_F_CLCD_CTRL_PWR_ENABLE                     ((uint32_t)(0x1UL << MXC_F_CLCD_CTRL_PWR_ENABLE_POS)) /**< CTRL_PWR_ENABLE Mask */
#define MXC_V_CLCD_CTRL_PWR_ENABLE_LO                  ((uint32_t)0x0UL) /**< CTRL_PWR_ENABLE_LO Value */
#define MXC_S_CLCD_CTRL_PWR_ENABLE_LO                  (MXC_V_CLCD_CTRL_PWR_ENABLE_LO << MXC_F_CLCD_CTRL_PWR_ENABLE_POS) /**< CTRL_PWR_ENABLE_LO Setting */
#define MXC_V_CLCD_CTRL_PWR_ENABLE_HI                  ((uint32_t)0x1UL) /**< CTRL_PWR_ENABLE_HI Value */
#define MXC_S_CLCD_CTRL_PWR_ENABLE_HI                  (MXC_V_CLCD_CTRL_PWR_ENABLE_HI << MXC_F_CLCD_CTRL_PWR_ENABLE_POS) /**< CTRL_PWR_ENABLE_HI Setting */

/**@} end of group CLCD_CTRL_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_FRBUF CLCD_FRBUF
 * @brief    Frame buffer.
 * @{
 */
#define MXC_F_CLCD_FRBUF_FRAME_ADDR_POS                0 /**< FRBUF_FRAME_ADDR Position */
#define MXC_F_CLCD_FRBUF_FRAME_ADDR                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_CLCD_FRBUF_FRAME_ADDR_POS)) /**< FRBUF_FRAME_ADDR Mask */

/**@} end of group CLCD_FRBUF_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_INT_EN CLCD_INT_EN
 * @brief    LCD Interrupt Enable Register.
 * @{
 */
#define MXC_F_CLCD_INT_EN_UNDERFLOW_IE_POS             0 /**< INT_EN_UNDERFLOW_IE Position */
#define MXC_F_CLCD_INT_EN_UNDERFLOW_IE                 ((uint32_t)(0x1UL << MXC_F_CLCD_INT_EN_UNDERFLOW_IE_POS)) /**< INT_EN_UNDERFLOW_IE Mask */
#define MXC_V_CLCD_INT_EN_UNDERFLOW_IE_DIS             ((uint32_t)0x0UL) /**< INT_EN_UNDERFLOW_IE_DIS Value */
#define MXC_S_CLCD_INT_EN_UNDERFLOW_IE_DIS             (MXC_V_CLCD_INT_EN_UNDERFLOW_IE_DIS << MXC_F_CLCD_INT_EN_UNDERFLOW_IE_POS) /**< INT_EN_UNDERFLOW_IE_DIS Setting */
#define MXC_V_CLCD_INT_EN_UNDERFLOW_IE_EN              ((uint32_t)0x1UL) /**< INT_EN_UNDERFLOW_IE_EN Value */
#define MXC_S_CLCD_INT_EN_UNDERFLOW_IE_EN              (MXC_V_CLCD_INT_EN_UNDERFLOW_IE_EN << MXC_F_CLCD_INT_EN_UNDERFLOW_IE_POS) /**< INT_EN_UNDERFLOW_IE_EN Setting */

#define MXC_F_CLCD_INT_EN_ADDR_RDY_IE_POS              1 /**< INT_EN_ADDR_RDY_IE Position */
#define MXC_F_CLCD_INT_EN_ADDR_RDY_IE                  ((uint32_t)(0x1UL << MXC_F_CLCD_INT_EN_ADDR_RDY_IE_POS)) /**< INT_EN_ADDR_RDY_IE Mask */
#define MXC_V_CLCD_INT_EN_ADDR_RDY_IE_DIS              ((uint32_t)0x0UL) /**< INT_EN_ADDR_RDY_IE_DIS Value */
#define MXC_S_CLCD_INT_EN_ADDR_RDY_IE_DIS              (MXC_V_CLCD_INT_EN_ADDR_RDY_IE_DIS << MXC_F_CLCD_INT_EN_ADDR_RDY_IE_POS) /**< INT_EN_ADDR_RDY_IE_DIS Setting */
#define MXC_V_CLCD_INT_EN_ADDR_RDY_IE_EN               ((uint32_t)0x1UL) /**< INT_EN_ADDR_RDY_IE_EN Value */
#define MXC_S_CLCD_INT_EN_ADDR_RDY_IE_EN               (MXC_V_CLCD_INT_EN_ADDR_RDY_IE_EN << MXC_F_CLCD_INT_EN_ADDR_RDY_IE_POS) /**< INT_EN_ADDR_RDY_IE_EN Setting */

#define MXC_F_CLCD_INT_EN_VCI_IE_POS                   2 /**< INT_EN_VCI_IE Position */
#define MXC_F_CLCD_INT_EN_VCI_IE                       ((uint32_t)(0x1UL << MXC_F_CLCD_INT_EN_VCI_IE_POS)) /**< INT_EN_VCI_IE Mask */
#define MXC_V_CLCD_INT_EN_VCI_IE_DIS                   ((uint32_t)0x0UL) /**< INT_EN_VCI_IE_DIS Value */
#define MXC_S_CLCD_INT_EN_VCI_IE_DIS                   (MXC_V_CLCD_INT_EN_VCI_IE_DIS << MXC_F_CLCD_INT_EN_VCI_IE_POS) /**< INT_EN_VCI_IE_DIS Setting */
#define MXC_V_CLCD_INT_EN_VCI_IE_EN                    ((uint32_t)0x1UL) /**< INT_EN_VCI_IE_EN Value */
#define MXC_S_CLCD_INT_EN_VCI_IE_EN                    (MXC_V_CLCD_INT_EN_VCI_IE_EN << MXC_F_CLCD_INT_EN_VCI_IE_POS) /**< INT_EN_VCI_IE_EN Setting */

#define MXC_F_CLCD_INT_EN_BUS_ERROR_IE_POS             3 /**< INT_EN_BUS_ERROR_IE Position */
#define MXC_F_CLCD_INT_EN_BUS_ERROR_IE                 ((uint32_t)(0x1UL << MXC_F_CLCD_INT_EN_BUS_ERROR_IE_POS)) /**< INT_EN_BUS_ERROR_IE Mask */
#define MXC_V_CLCD_INT_EN_BUS_ERROR_IE_DIS             ((uint32_t)0x0UL) /**< INT_EN_BUS_ERROR_IE_DIS Value */
#define MXC_S_CLCD_INT_EN_BUS_ERROR_IE_DIS             (MXC_V_CLCD_INT_EN_BUS_ERROR_IE_DIS << MXC_F_CLCD_INT_EN_BUS_ERROR_IE_POS) /**< INT_EN_BUS_ERROR_IE_DIS Setting */
#define MXC_V_CLCD_INT_EN_BUS_ERROR_IE_EN              ((uint32_t)0x1UL) /**< INT_EN_BUS_ERROR_IE_EN Value */
#define MXC_S_CLCD_INT_EN_BUS_ERROR_IE_EN              (MXC_V_CLCD_INT_EN_BUS_ERROR_IE_EN << MXC_F_CLCD_INT_EN_BUS_ERROR_IE_POS) /**< INT_EN_BUS_ERROR_IE_EN Setting */

/**@} end of group CLCD_INT_EN_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_INT_STAT CLCD_INT_STAT
 * @brief    LCD Status Register.
 * @{
 */
#define MXC_F_CLCD_INT_STAT_UNDERFLOW_POS              0 /**< INT_STAT_UNDERFLOW Position */
#define MXC_F_CLCD_INT_STAT_UNDERFLOW                  ((uint32_t)(0x1UL << MXC_F_CLCD_INT_STAT_UNDERFLOW_POS)) /**< INT_STAT_UNDERFLOW Mask */
#define MXC_V_CLCD_INT_STAT_UNDERFLOW_INACTIVE         ((uint32_t)0x0UL) /**< INT_STAT_UNDERFLOW_INACTIVE Value */
#define MXC_S_CLCD_INT_STAT_UNDERFLOW_INACTIVE         (MXC_V_CLCD_INT_STAT_UNDERFLOW_INACTIVE << MXC_F_CLCD_INT_STAT_UNDERFLOW_POS) /**< INT_STAT_UNDERFLOW_INACTIVE Setting */
#define MXC_V_CLCD_INT_STAT_UNDERFLOW_PEND             ((uint32_t)0x1UL) /**< INT_STAT_UNDERFLOW_PEND Value */
#define MXC_S_CLCD_INT_STAT_UNDERFLOW_PEND             (MXC_V_CLCD_INT_STAT_UNDERFLOW_PEND << MXC_F_CLCD_INT_STAT_UNDERFLOW_POS) /**< INT_STAT_UNDERFLOW_PEND Setting */
#define MXC_V_CLCD_INT_STAT_UNDERFLOW_CLEAR            ((uint32_t)0x1UL) /**< INT_STAT_UNDERFLOW_CLEAR Value */
#define MXC_S_CLCD_INT_STAT_UNDERFLOW_CLEAR            (MXC_V_CLCD_INT_STAT_UNDERFLOW_CLEAR << MXC_F_CLCD_INT_STAT_UNDERFLOW_POS) /**< INT_STAT_UNDERFLOW_CLEAR Setting */

#define MXC_F_CLCD_INT_STAT_ADDR_RDY_POS               1 /**< INT_STAT_ADDR_RDY Position */
#define MXC_F_CLCD_INT_STAT_ADDR_RDY                   ((uint32_t)(0x1UL << MXC_F_CLCD_INT_STAT_ADDR_RDY_POS)) /**< INT_STAT_ADDR_RDY Mask */
#define MXC_V_CLCD_INT_STAT_ADDR_RDY_INACTIVE          ((uint32_t)0x0UL) /**< INT_STAT_ADDR_RDY_INACTIVE Value */
#define MXC_S_CLCD_INT_STAT_ADDR_RDY_INACTIVE          (MXC_V_CLCD_INT_STAT_ADDR_RDY_INACTIVE << MXC_F_CLCD_INT_STAT_ADDR_RDY_POS) /**< INT_STAT_ADDR_RDY_INACTIVE Setting */
#define MXC_V_CLCD_INT_STAT_ADDR_RDY_PEND              ((uint32_t)0x1UL) /**< INT_STAT_ADDR_RDY_PEND Value */
#define MXC_S_CLCD_INT_STAT_ADDR_RDY_PEND              (MXC_V_CLCD_INT_STAT_ADDR_RDY_PEND << MXC_F_CLCD_INT_STAT_ADDR_RDY_POS) /**< INT_STAT_ADDR_RDY_PEND Setting */
#define MXC_V_CLCD_INT_STAT_ADDR_RDY_CLEAR             ((uint32_t)0x1UL) /**< INT_STAT_ADDR_RDY_CLEAR Value */
#define MXC_S_CLCD_INT_STAT_ADDR_RDY_CLEAR             (MXC_V_CLCD_INT_STAT_ADDR_RDY_CLEAR << MXC_F_CLCD_INT_STAT_ADDR_RDY_POS) /**< INT_STAT_ADDR_RDY_CLEAR Setting */

#define MXC_F_CLCD_INT_STAT_VCI_POS                    2 /**< INT_STAT_VCI Position */
#define MXC_F_CLCD_INT_STAT_VCI                        ((uint32_t)(0x1UL << MXC_F_CLCD_INT_STAT_VCI_POS)) /**< INT_STAT_VCI Mask */
#define MXC_V_CLCD_INT_STAT_VCI_INACTIVE               ((uint32_t)0x0UL) /**< INT_STAT_VCI_INACTIVE Value */
#define MXC_S_CLCD_INT_STAT_VCI_INACTIVE               (MXC_V_CLCD_INT_STAT_VCI_INACTIVE << MXC_F_CLCD_INT_STAT_VCI_POS) /**< INT_STAT_VCI_INACTIVE Setting */
#define MXC_V_CLCD_INT_STAT_VCI_PEND                   ((uint32_t)0x1UL) /**< INT_STAT_VCI_PEND Value */
#define MXC_S_CLCD_INT_STAT_VCI_PEND                   (MXC_V_CLCD_INT_STAT_VCI_PEND << MXC_F_CLCD_INT_STAT_VCI_POS) /**< INT_STAT_VCI_PEND Setting */
#define MXC_V_CLCD_INT_STAT_VCI_CLEAR                  ((uint32_t)0x1UL) /**< INT_STAT_VCI_CLEAR Value */
#define MXC_S_CLCD_INT_STAT_VCI_CLEAR                  (MXC_V_CLCD_INT_STAT_VCI_CLEAR << MXC_F_CLCD_INT_STAT_VCI_POS) /**< INT_STAT_VCI_CLEAR Setting */

#define MXC_F_CLCD_INT_STAT_BUS_ERROR_POS              3 /**< INT_STAT_BUS_ERROR Position */
#define MXC_F_CLCD_INT_STAT_BUS_ERROR                  ((uint32_t)(0x1UL << MXC_F_CLCD_INT_STAT_BUS_ERROR_POS)) /**< INT_STAT_BUS_ERROR Mask */
#define MXC_V_CLCD_INT_STAT_BUS_ERROR_INACTIVE         ((uint32_t)0x0UL) /**< INT_STAT_BUS_ERROR_INACTIVE Value */
#define MXC_S_CLCD_INT_STAT_BUS_ERROR_INACTIVE         (MXC_V_CLCD_INT_STAT_BUS_ERROR_INACTIVE << MXC_F_CLCD_INT_STAT_BUS_ERROR_POS) /**< INT_STAT_BUS_ERROR_INACTIVE Setting */
#define MXC_V_CLCD_INT_STAT_BUS_ERROR_PEND             ((uint32_t)0x1UL) /**< INT_STAT_BUS_ERROR_PEND Value */
#define MXC_S_CLCD_INT_STAT_BUS_ERROR_PEND             (MXC_V_CLCD_INT_STAT_BUS_ERROR_PEND << MXC_F_CLCD_INT_STAT_BUS_ERROR_POS) /**< INT_STAT_BUS_ERROR_PEND Setting */
#define MXC_V_CLCD_INT_STAT_BUS_ERROR_CLEAR            ((uint32_t)0x1UL) /**< INT_STAT_BUS_ERROR_CLEAR Value */
#define MXC_S_CLCD_INT_STAT_BUS_ERROR_CLEAR            (MXC_V_CLCD_INT_STAT_BUS_ERROR_CLEAR << MXC_F_CLCD_INT_STAT_BUS_ERROR_POS) /**< INT_STAT_BUS_ERROR_CLEAR Setting */

#define MXC_F_CLCD_INT_STAT_CLCD_IDLE_POS              8 /**< INT_STAT_CLCD_IDLE Position */
#define MXC_F_CLCD_INT_STAT_CLCD_IDLE                  ((uint32_t)(0x1UL << MXC_F_CLCD_INT_STAT_CLCD_IDLE_POS)) /**< INT_STAT_CLCD_IDLE Mask */
#define MXC_V_CLCD_INT_STAT_CLCD_IDLE_IDLE             ((uint32_t)0x0UL) /**< INT_STAT_CLCD_IDLE_IDLE Value */
#define MXC_S_CLCD_INT_STAT_CLCD_IDLE_IDLE             (MXC_V_CLCD_INT_STAT_CLCD_IDLE_IDLE << MXC_F_CLCD_INT_STAT_CLCD_IDLE_POS) /**< INT_STAT_CLCD_IDLE_IDLE Setting */
#define MXC_V_CLCD_INT_STAT_CLCD_IDLE_BUSY             ((uint32_t)0x1UL) /**< INT_STAT_CLCD_IDLE_BUSY Value */
#define MXC_S_CLCD_INT_STAT_CLCD_IDLE_BUSY             (MXC_V_CLCD_INT_STAT_CLCD_IDLE_BUSY << MXC_F_CLCD_INT_STAT_CLCD_IDLE_POS) /**< INT_STAT_CLCD_IDLE_BUSY Setting */

/**@} end of group CLCD_INT_STAT_Register */

/**
 * @ingroup  clcd_registers
 * @defgroup CLCD_PALETTE_RAM CLCD_PALETTE_RAM
 * @brief    Palette
 * @{
 */
#define MXC_F_CLCD_PALETTE_RAM_RED_POS                 0 /**< PALETTE_RAM_RED Position */
#define MXC_F_CLCD_PALETTE_RAM_RED                     ((uint32_t)(0xFFUL << MXC_F_CLCD_PALETTE_RAM_RED_POS)) /**< PALETTE_RAM_RED Mask */

#define MXC_F_CLCD_PALETTE_RAM_GREEN_POS               8 /**< PALETTE_RAM_GREEN Position */
#define MXC_F_CLCD_PALETTE_RAM_GREEN                   ((uint32_t)(0xFFUL << MXC_F_CLCD_PALETTE_RAM_GREEN_POS)) /**< PALETTE_RAM_GREEN Mask */

#define MXC_F_CLCD_PALETTE_RAM_BLUE_POS                16 /**< PALETTE_RAM_BLUE Position */
#define MXC_F_CLCD_PALETTE_RAM_BLUE                    ((uint32_t)(0xFFUL << MXC_F_CLCD_PALETTE_RAM_BLUE_POS)) /**< PALETTE_RAM_BLUE Mask */

/**@} end of group CLCD_PALETTE_RAM_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_CLCD_REGS_H_
