/**
 * @file    clcd_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the CLCD_REVA Peripheral Module.
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

#ifndef _CLCD_REVA_REGS_H_
#define _CLCD_REVA_REGS_H_

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
 * @ingroup     clcd_reva
 * @defgroup    clcd_reva_registers CLCD_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the CLCD_REVA Peripheral Module.
 * @details Color LCD Controller
 */

/**
 * @ingroup clcd_reva_registers
 * Structure type to access the CLCD_REVA Registers.
 */
typedef struct {
    __IO uint32_t clk;                  /**< <tt>\b 0x000:</tt> CLCD_REVA CLK Register */
    __IO uint32_t vtim_0;               /**< <tt>\b 0x004:</tt> CLCD_REVA VTIM_0 Register */
    __IO uint32_t vtim_1;               /**< <tt>\b 0x008:</tt> CLCD_REVA VTIM_1 Register */
    __IO uint32_t htim;                 /**< <tt>\b 0x00C:</tt> CLCD_REVA HTIM Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x010:</tt> CLCD_REVA CTRL Register */
    __R  uint32_t rsv_0x14;
    __IO uint32_t fr;                   /**< <tt>\b 0x18:</tt> CLCD_REVA FR Register */
    __R  uint32_t rsv_0x1c;
    __IO uint32_t int_en;               /**< <tt>\b 0x020:</tt> CLCD_REVA INT_EN Register */
    __IO uint32_t stat;                 /**< <tt>\b 0x024:</tt> CLCD_REVA STAT Register */
    __R  uint32_t rsv_0x28_0x3ff[246];
    __IO uint32_t palette[256];         /**< <tt>\b 0x400:</tt> CLCD_REVA PALETTE Register */
} mxc_clcd_reva_regs_t;

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_CLK CLCD_REVA_CLK
 * @brief    LCD Clock Control Register
 * @{
 */
 #define MXC_F_CLCD_REVA_CLK_CLKDIV_POS                      0 /**< CLK_CLKDIV Position */
 #define MXC_F_CLCD_REVA_CLK_CLKDIV                          ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_CLK_CLKDIV_POS)) /**< CLK_CLKDIV Mask */

 #define MXC_F_CLCD_REVA_CLK_ACB_POS                         8 /**< CLK_ACB Position */
 #define MXC_F_CLCD_REVA_CLK_ACB                             ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_CLK_ACB_POS)) /**< CLK_ACB Mask */

 #define MXC_F_CLCD_REVA_CLK_DPOL_POS                        16 /**< CLK_DPOL Position */
 #define MXC_F_CLCD_REVA_CLK_DPOL                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CLK_DPOL_POS)) /**< CLK_DPOL Mask */
 #define MXC_V_CLCD_REVA_CLK_DPOL_ACTIVEHI                   ((uint32_t)0x0UL) /**< CLK_DPOL_ACTIVEHI Value */
 #define MXC_S_CLCD_REVA_CLK_DPOL_ACTIVEHI                   (MXC_V_CLCD_REVA_CLK_DPOL_ACTIVEHI << MXC_F_CLCD_REVA_CLK_DPOL_POS) /**< CLK_DPOL_ACTIVEHI Setting */
 #define MXC_V_CLCD_REVA_CLK_DPOL_ACTIVELO                   ((uint32_t)0x1UL) /**< CLK_DPOL_ACTIVELO Value */
 #define MXC_S_CLCD_REVA_CLK_DPOL_ACTIVELO                   (MXC_V_CLCD_REVA_CLK_DPOL_ACTIVELO << MXC_F_CLCD_REVA_CLK_DPOL_POS) /**< CLK_DPOL_ACTIVELO Setting */

 #define MXC_F_CLCD_REVA_CLK_VPOL_POS                        17 /**< CLK_VPOL Position */
 #define MXC_F_CLCD_REVA_CLK_VPOL                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CLK_VPOL_POS)) /**< CLK_VPOL Mask */
 #define MXC_V_CLCD_REVA_CLK_VPOL_ACTIVEHI                   ((uint32_t)0x1UL) /**< CLK_VPOL_ACTIVEHI Value */
 #define MXC_S_CLCD_REVA_CLK_VPOL_ACTIVEHI                   (MXC_V_CLCD_REVA_CLK_VPOL_ACTIVEHI << MXC_F_CLCD_REVA_CLK_VPOL_POS) /**< CLK_VPOL_ACTIVEHI Setting */
 #define MXC_V_CLCD_REVA_CLK_VPOL_ACTIVELO                   ((uint32_t)0x0UL) /**< CLK_VPOL_ACTIVELO Value */
 #define MXC_S_CLCD_REVA_CLK_VPOL_ACTIVELO                   (MXC_V_CLCD_REVA_CLK_VPOL_ACTIVELO << MXC_F_CLCD_REVA_CLK_VPOL_POS) /**< CLK_VPOL_ACTIVELO Setting */

 #define MXC_F_CLCD_REVA_CLK_HPOL_POS                        18 /**< CLK_HPOL Position */
 #define MXC_F_CLCD_REVA_CLK_HPOL                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CLK_HPOL_POS)) /**< CLK_HPOL Mask */
 #define MXC_V_CLCD_REVA_CLK_HPOL_ACTIVEHI                   ((uint32_t)0x1UL) /**< CLK_HPOL_ACTIVEHI Value */
 #define MXC_S_CLCD_REVA_CLK_HPOL_ACTIVEHI                   (MXC_V_CLCD_REVA_CLK_HPOL_ACTIVEHI << MXC_F_CLCD_REVA_CLK_HPOL_POS) /**< CLK_HPOL_ACTIVEHI Setting */
 #define MXC_V_CLCD_REVA_CLK_HPOL_ACTIVELO                   ((uint32_t)0x0UL) /**< CLK_HPOL_ACTIVELO Value */
 #define MXC_S_CLCD_REVA_CLK_HPOL_ACTIVELO                   (MXC_V_CLCD_REVA_CLK_HPOL_ACTIVELO << MXC_F_CLCD_REVA_CLK_HPOL_POS) /**< CLK_HPOL_ACTIVELO Setting */

 #define MXC_F_CLCD_REVA_CLK_EDGE_POS                        19 /**< CLK_EDGE Position */
 #define MXC_F_CLCD_REVA_CLK_EDGE                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CLK_EDGE_POS)) /**< CLK_EDGE Mask */
 #define MXC_V_CLCD_REVA_CLK_EDGE_RISEEDGE                   ((uint32_t)0x0UL) /**< CLK_EDGE_RISEEDGE Value */
 #define MXC_S_CLCD_REVA_CLK_EDGE_RISEEDGE                   (MXC_V_CLCD_REVA_CLK_EDGE_RISEEDGE << MXC_F_CLCD_REVA_CLK_EDGE_POS) /**< CLK_EDGE_RISEEDGE Setting */
 #define MXC_V_CLCD_REVA_CLK_EDGE_FALLEDGE                   ((uint32_t)0x1UL) /**< CLK_EDGE_FALLEDGE Value */
 #define MXC_S_CLCD_REVA_CLK_EDGE_FALLEDGE                   (MXC_V_CLCD_REVA_CLK_EDGE_FALLEDGE << MXC_F_CLCD_REVA_CLK_EDGE_POS) /**< CLK_EDGE_FALLEDGE Setting */

 #define MXC_F_CLCD_REVA_CLK_PASCLK_POS                      20 /**< CLK_PASCLK Position */
 #define MXC_F_CLCD_REVA_CLK_PASCLK                          ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CLK_PASCLK_POS)) /**< CLK_PASCLK Mask */
 #define MXC_V_CLCD_REVA_CLK_PASCLK_ALWAYSACTIVE             ((uint32_t)0x0UL) /**< CLK_PASCLK_ALWAYSACTIVE Value */
 #define MXC_S_CLCD_REVA_CLK_PASCLK_ALWAYSACTIVE             (MXC_V_CLCD_REVA_CLK_PASCLK_ALWAYSACTIVE << MXC_F_CLCD_REVA_CLK_PASCLK_POS) /**< CLK_PASCLK_ALWAYSACTIVE Setting */
 #define MXC_V_CLCD_REVA_CLK_PASCLK_ACTIVEONDATA             ((uint32_t)0x1UL) /**< CLK_PASCLK_ACTIVEONDATA Value */
 #define MXC_S_CLCD_REVA_CLK_PASCLK_ACTIVEONDATA             (MXC_V_CLCD_REVA_CLK_PASCLK_ACTIVEONDATA << MXC_F_CLCD_REVA_CLK_PASCLK_POS) /**< CLK_PASCLK_ACTIVEONDATA Setting */

/**@} end of group CLCD_REVA_CLK_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_VTIM_0 CLCD_REVA_VTIM_0
 * @brief    LCD Vertical Timing 0 Register
 * @{
 */
 #define MXC_F_CLCD_REVA_VTIM_0_VLINES_POS                   0 /**< VTIM_0_VLINES Position */
 #define MXC_F_CLCD_REVA_VTIM_0_VLINES                       ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_VTIM_0_VLINES_POS)) /**< VTIM_0_VLINES Mask */

 #define MXC_F_CLCD_REVA_VTIM_0_VBACKPORCH_POS               16 /**< VTIM_0_VBACKPORCH Position */
 #define MXC_F_CLCD_REVA_VTIM_0_VBACKPORCH                   ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_VTIM_0_VBACKPORCH_POS)) /**< VTIM_0_VBACKPORCH Mask */

/**@} end of group CLCD_REVA_VTIM_0_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_VTIM_1 CLCD_REVA_VTIM_1
 * @brief    LCD Vertical Timing 1 Register
 * @{
 */
 #define MXC_F_CLCD_REVA_VTIM_1_VSYNCWIDTH_POS               0 /**< VTIM_1_VSYNCWIDTH Position */
 #define MXC_F_CLCD_REVA_VTIM_1_VSYNCWIDTH                   ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_VTIM_1_VSYNCWIDTH_POS)) /**< VTIM_1_VSYNCWIDTH Mask */

 #define MXC_F_CLCD_REVA_VTIM_1_VFRONTPORCH_POS              16 /**< VTIM_1_VFRONTPORCH Position */
 #define MXC_F_CLCD_REVA_VTIM_1_VFRONTPORCH                  ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_VTIM_1_VFRONTPORCH_POS)) /**< VTIM_1_VFRONTPORCH Mask */

/**@} end of group CLCD_REVA_VTIM_1_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_HTIM CLCD_REVA_HTIM
 * @brief    LCD Horizontal Timing Register.
 * @{
 */
 #define MXC_F_CLCD_REVA_HTIM_HSYNCWIDTH_POS                 0 /**< HTIM_HSYNCWIDTH Position */
 #define MXC_F_CLCD_REVA_HTIM_HSYNCWIDTH                     ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_HTIM_HSYNCWIDTH_POS)) /**< HTIM_HSYNCWIDTH Mask */

 #define MXC_F_CLCD_REVA_HTIM_HFRONTPORCH_POS                8 /**< HTIM_HFRONTPORCH Position */
 #define MXC_F_CLCD_REVA_HTIM_HFRONTPORCH                    ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_HTIM_HFRONTPORCH_POS)) /**< HTIM_HFRONTPORCH Mask */

 #define MXC_F_CLCD_REVA_HTIM_HSIZE_POS                      16 /**< HTIM_HSIZE Position */
 #define MXC_F_CLCD_REVA_HTIM_HSIZE                          ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_HTIM_HSIZE_POS)) /**< HTIM_HSIZE Mask */

 #define MXC_F_CLCD_REVA_HTIM_HBACKPORCH_POS                 24 /**< HTIM_HBACKPORCH Position */
 #define MXC_F_CLCD_REVA_HTIM_HBACKPORCH                     ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_HTIM_HBACKPORCH_POS)) /**< HTIM_HBACKPORCH Mask */

/**@} end of group CLCD_REVA_HTIM_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_CTRL CLCD_REVA_CTRL
 * @brief    LCD Control Register
 * @{
 */
 #define MXC_F_CLCD_REVA_CTRL_LCDEN_POS                      0 /**< CTRL_LCDEN Position */
 #define MXC_F_CLCD_REVA_CTRL_LCDEN                          ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CTRL_LCDEN_POS)) /**< CTRL_LCDEN Mask */
 #define MXC_V_CLCD_REVA_CTRL_LCDEN_DISABLE                  ((uint32_t)0x0UL) /**< CTRL_LCDEN_DISABLE Value */
 #define MXC_S_CLCD_REVA_CTRL_LCDEN_DISABLE                  (MXC_V_CLCD_REVA_CTRL_LCDEN_DISABLE << MXC_F_CLCD_REVA_CTRL_LCDEN_POS) /**< CTRL_LCDEN_DISABLE Setting */
 #define MXC_V_CLCD_REVA_CTRL_LCDEN_ENABLE                   ((uint32_t)0x1UL) /**< CTRL_LCDEN_ENABLE Value */
 #define MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE                   (MXC_V_CLCD_REVA_CTRL_LCDEN_ENABLE << MXC_F_CLCD_REVA_CTRL_LCDEN_POS) /**< CTRL_LCDEN_ENABLE Setting */

 #define MXC_F_CLCD_REVA_CTRL_VISEL_POS                      1 /**< CTRL_VISEL Position */
 #define MXC_F_CLCD_REVA_CTRL_VISEL                          ((uint32_t)(0x3UL << MXC_F_CLCD_REVA_CTRL_VISEL_POS)) /**< CTRL_VISEL Mask */
 #define MXC_V_CLCD_REVA_CTRL_VISEL_ONVERTSYNC               ((uint32_t)0x0UL) /**< CTRL_VISEL_ONVERTSYNC Value */
 #define MXC_S_CLCD_REVA_CTRL_VISEL_ONVERTSYNC               (MXC_V_CLCD_REVA_CTRL_VISEL_ONVERTSYNC << MXC_F_CLCD_REVA_CTRL_VISEL_POS) /**< CTRL_VISEL_ONVERTSYNC Setting */
 #define MXC_V_CLCD_REVA_CTRL_VISEL_ONVERTBACKPORCH          ((uint32_t)0x1UL) /**< CTRL_VISEL_ONVERTBACKPORCH Value */
 #define MXC_S_CLCD_REVA_CTRL_VISEL_ONVERTBACKPORCH          (MXC_V_CLCD_REVA_CTRL_VISEL_ONVERTBACKPORCH << MXC_F_CLCD_REVA_CTRL_VISEL_POS) /**< CTRL_VISEL_ONVERTBACKPORCH Setting */
 #define MXC_V_CLCD_REVA_CTRL_VISEL_ONACTIVEVIDEO            ((uint32_t)0x2UL) /**< CTRL_VISEL_ONACTIVEVIDEO Value */
 #define MXC_S_CLCD_REVA_CTRL_VISEL_ONACTIVEVIDEO            (MXC_V_CLCD_REVA_CTRL_VISEL_ONACTIVEVIDEO << MXC_F_CLCD_REVA_CTRL_VISEL_POS) /**< CTRL_VISEL_ONACTIVEVIDEO Setting */
 #define MXC_V_CLCD_REVA_CTRL_VISEL_ONVERTFRONTPORCH         ((uint32_t)0x3UL) /**< CTRL_VISEL_ONVERTFRONTPORCH Value */
 #define MXC_S_CLCD_REVA_CTRL_VISEL_ONVERTFRONTPORCH         (MXC_V_CLCD_REVA_CTRL_VISEL_ONVERTFRONTPORCH << MXC_F_CLCD_REVA_CTRL_VISEL_POS) /**< CTRL_VISEL_ONVERTFRONTPORCH Setting */

 #define MXC_F_CLCD_REVA_CTRL_DISPTYPE_POS                   4 /**< CTRL_DISPTYPE Position */
 #define MXC_F_CLCD_REVA_CTRL_DISPTYPE                       ((uint32_t)(0xFUL << MXC_F_CLCD_REVA_CTRL_DISPTYPE_POS)) /**< CTRL_DISPTYPE Mask */
 #define MXC_V_CLCD_REVA_CTRL_DISPTYPE_STNCOLOR8BIT          ((uint32_t)0x4UL) /**< CTRL_DISPTYPE_STNCOLOR8BIT Value */
 #define MXC_S_CLCD_REVA_CTRL_DISPTYPE_STNCOLOR8BIT          (MXC_V_CLCD_REVA_CTRL_DISPTYPE_STNCOLOR8BIT << MXC_F_CLCD_REVA_CTRL_DISPTYPE_POS) /**< CTRL_DISPTYPE_STNCOLOR8BIT Setting */
 #define MXC_V_CLCD_REVA_CTRL_DISPTYPE_CLCD                  ((uint32_t)0x8UL) /**< CTRL_DISPTYPE_CLCD_REVA Value */
 #define MXC_S_CLCD_REVA_CTRL_DISPTYPE_CLCD                  (MXC_V_CLCD_REVA_CTRL_DISPTYPE_CLCD << MXC_F_CLCD_REVA_CTRL_DISPTYPE_POS) /**< CTRL_DISPTYPE_CLCD_REVA Setting */

 #define MXC_F_CLCD_REVA_CTRL_BPP_POS                        8 /**< CTRL_BPP Position */
 #define MXC_F_CLCD_REVA_CTRL_BPP                            ((uint32_t)(0x7UL << MXC_F_CLCD_REVA_CTRL_BPP_POS)) /**< CTRL_BPP Mask */
 #define MXC_V_CLCD_REVA_CTRL_BPP_BPP1                       ((uint32_t)0x0UL) /**< CTRL_BPP_BPP1 Value */
 #define MXC_S_CLCD_REVA_CTRL_BPP_BPP1                       (MXC_V_CLCD_REVA_CTRL_BPP_BPP1 << MXC_F_CLCD_REVA_CTRL_BPP_POS) /**< CTRL_BPP_BPP1 Setting */
 #define MXC_V_CLCD_REVA_CTRL_BPP_BPP2                       ((uint32_t)0x1UL) /**< CTRL_BPP_BPP2 Value */
 #define MXC_S_CLCD_REVA_CTRL_BPP_BPP2                       (MXC_V_CLCD_REVA_CTRL_BPP_BPP2 << MXC_F_CLCD_REVA_CTRL_BPP_POS) /**< CTRL_BPP_BPP2 Setting */
 #define MXC_V_CLCD_REVA_CTRL_BPP_BPP4                       ((uint32_t)0x2UL) /**< CTRL_BPP_BPP4 Value */
 #define MXC_S_CLCD_REVA_CTRL_BPP_BPP4                       (MXC_V_CLCD_REVA_CTRL_BPP_BPP4 << MXC_F_CLCD_REVA_CTRL_BPP_POS) /**< CTRL_BPP_BPP4 Setting */
 #define MXC_V_CLCD_REVA_CTRL_BPP_BPP8                       ((uint32_t)0x3UL) /**< CTRL_BPP_BPP8 Value */
 #define MXC_S_CLCD_REVA_CTRL_BPP_BPP8                       (MXC_V_CLCD_REVA_CTRL_BPP_BPP8 << MXC_F_CLCD_REVA_CTRL_BPP_POS) /**< CTRL_BPP_BPP8 Setting */
 #define MXC_V_CLCD_REVA_CTRL_BPP_BPP16                      ((uint32_t)0x4UL) /**< CTRL_BPP_BPP16 Value */
 #define MXC_S_CLCD_REVA_CTRL_BPP_BPP16                      (MXC_V_CLCD_REVA_CTRL_BPP_BPP16 << MXC_F_CLCD_REVA_CTRL_BPP_POS) /**< CTRL_BPP_BPP16 Setting */
 #define MXC_V_CLCD_REVA_CTRL_BPP_BPP24                      ((uint32_t)0x5UL) /**< CTRL_BPP_BPP24 Value */
 #define MXC_S_CLCD_REVA_CTRL_BPP_BPP24                      (MXC_V_CLCD_REVA_CTRL_BPP_BPP24 << MXC_F_CLCD_REVA_CTRL_BPP_POS) /**< CTRL_BPP_BPP24 Setting */

 #define MXC_F_CLCD_REVA_CTRL_MODE565_POS                    11 /**< CTRL_MODE565 Position */
 #define MXC_F_CLCD_REVA_CTRL_MODE565                        ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CTRL_MODE565_POS)) /**< CTRL_MODE565 Mask */
 #define MXC_V_CLCD_REVA_CTRL_MODE565_BGR556                 ((uint32_t)0x0UL) /**< CTRL_MODE565_BGR556 Value */
 #define MXC_S_CLCD_REVA_CTRL_MODE565_BGR556                 (MXC_V_CLCD_REVA_CTRL_MODE565_BGR556 << MXC_F_CLCD_REVA_CTRL_MODE565_POS) /**< CTRL_MODE565_BGR556 Setting */
 #define MXC_V_CLCD_REVA_CTRL_MODE565_RGB565                 ((uint32_t)0x1UL) /**< CTRL_MODE565_RGB565 Value */
 #define MXC_S_CLCD_REVA_CTRL_MODE565_RGB565                 (MXC_V_CLCD_REVA_CTRL_MODE565_RGB565 << MXC_F_CLCD_REVA_CTRL_MODE565_POS) /**< CTRL_MODE565_RGB565 Setting */

 #define MXC_F_CLCD_REVA_CTRL_EMODE_POS                      12 /**< CTRL_EMODE Position */
 #define MXC_F_CLCD_REVA_CTRL_EMODE                          ((uint32_t)(0x3UL << MXC_F_CLCD_REVA_CTRL_EMODE_POS)) /**< CTRL_EMODE Mask */
 #define MXC_V_CLCD_REVA_CTRL_EMODE_LLBP                     ((uint32_t)0x0UL) /**< CTRL_EMODE_LLBP Value */
 #define MXC_S_CLCD_REVA_CTRL_EMODE_LLBP                     (MXC_V_CLCD_REVA_CTRL_EMODE_LLBP << MXC_F_CLCD_REVA_CTRL_EMODE_POS) /**< CTRL_EMODE_LLBP Setting */
 #define MXC_V_CLCD_REVA_CTRL_EMODE_BBBP                     ((uint32_t)0x1UL) /**< CTRL_EMODE_BBBP Value */
 #define MXC_S_CLCD_REVA_CTRL_EMODE_BBBP                     (MXC_V_CLCD_REVA_CTRL_EMODE_BBBP << MXC_F_CLCD_REVA_CTRL_EMODE_POS) /**< CTRL_EMODE_BBBP Setting */
 #define MXC_V_CLCD_REVA_CTRL_EMODE_LBBP                     ((uint32_t)0x2UL) /**< CTRL_EMODE_LBBP Value */
 #define MXC_S_CLCD_REVA_CTRL_EMODE_LBBP                     (MXC_V_CLCD_REVA_CTRL_EMODE_LBBP << MXC_F_CLCD_REVA_CTRL_EMODE_POS) /**< CTRL_EMODE_LBBP Setting */
 #define MXC_V_CLCD_REVA_CTRL_EMODE_RFU                      ((uint32_t)0x3UL) /**< CTRL_EMODE_RFU Value */
 #define MXC_S_CLCD_REVA_CTRL_EMODE_RFU                      (MXC_V_CLCD_REVA_CTRL_EMODE_RFU << MXC_F_CLCD_REVA_CTRL_EMODE_POS) /**< CTRL_EMODE_RFU Setting */

 #define MXC_F_CLCD_REVA_CTRL_C24_POS                        15 /**< CTRL_C24 Position */
 #define MXC_F_CLCD_REVA_CTRL_C24                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CTRL_C24_POS)) /**< CTRL_C24 Mask */

 #define MXC_F_CLCD_REVA_CTRL_BURST_POS                      19 /**< CTRL_BURST Position */
 #define MXC_F_CLCD_REVA_CTRL_BURST                          ((uint32_t)(0x3UL << MXC_F_CLCD_REVA_CTRL_BURST_POS)) /**< CTRL_BURST Mask */
 #define MXC_V_CLCD_REVA_CTRL_BURST_WORDS4                   ((uint32_t)0x0UL) /**< CTRL_BURST_WORDS4 Value */
 #define MXC_S_CLCD_REVA_CTRL_BURST_WORDS4                   (MXC_V_CLCD_REVA_CTRL_BURST_WORDS4 << MXC_F_CLCD_REVA_CTRL_BURST_POS) /**< CTRL_BURST_WORDS4 Setting */
 #define MXC_V_CLCD_REVA_CTRL_BURST_WORDS8                   ((uint32_t)0x1UL) /**< CTRL_BURST_WORDS8 Value */
 #define MXC_S_CLCD_REVA_CTRL_BURST_WORDS8                   (MXC_V_CLCD_REVA_CTRL_BURST_WORDS8 << MXC_F_CLCD_REVA_CTRL_BURST_POS) /**< CTRL_BURST_WORDS8 Setting */

 #define MXC_F_CLCD_REVA_CTRL_LPOL_POS                       21 /**< CTRL_LPOL Position */
 #define MXC_F_CLCD_REVA_CTRL_LPOL                           ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CTRL_LPOL_POS)) /**< CTRL_LPOL Mask */
 #define MXC_V_CLCD_REVA_CTRL_LPOL_ACTIVEHI                  ((uint32_t)0x0UL) /**< CTRL_LPOL_ACTIVEHI Value */
 #define MXC_S_CLCD_REVA_CTRL_LPOL_ACTIVEHI                  (MXC_V_CLCD_REVA_CTRL_LPOL_ACTIVEHI << MXC_F_CLCD_REVA_CTRL_LPOL_POS) /**< CTRL_LPOL_ACTIVEHI Setting */
 #define MXC_V_CLCD_REVA_CTRL_LPOL_ACTIVELO                  ((uint32_t)0x1UL) /**< CTRL_LPOL_ACTIVELO Value */
 #define MXC_S_CLCD_REVA_CTRL_LPOL_ACTIVELO                  (MXC_V_CLCD_REVA_CTRL_LPOL_ACTIVELO << MXC_F_CLCD_REVA_CTRL_LPOL_POS) /**< CTRL_LPOL_ACTIVELO Setting */

 #define MXC_F_CLCD_REVA_CTRL_PEN_POS                        22 /**< CTRL_PEN Position */
 #define MXC_F_CLCD_REVA_CTRL_PEN                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_CTRL_PEN_POS)) /**< CTRL_PEN Mask */

/**@} end of group CLCD_REVA_CTRL_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_INT_EN CLCD_REVA_INT_EN
 * @brief    LCD Interrupt Enable Register.
 * @{
 */
 #define MXC_F_CLCD_REVA_INT_EN_UFLO_POS                     0 /**< INT_EN_UFLO Position */
 #define MXC_F_CLCD_REVA_INT_EN_UFLO                         ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_INT_EN_UFLO_POS)) /**< INT_EN_UFLO Mask */

 #define MXC_F_CLCD_REVA_INT_EN_ADRRDY_POS                   1 /**< INT_EN_ADRRDY Position */
 #define MXC_F_CLCD_REVA_INT_EN_ADRRDY                       ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_INT_EN_ADRRDY_POS)) /**< INT_EN_ADRRDY Mask */

 #define MXC_F_CLCD_REVA_INT_EN_VCI_POS                      2 /**< INT_EN_VCI Position */
 #define MXC_F_CLCD_REVA_INT_EN_VCI                          ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_INT_EN_VCI_POS)) /**< INT_EN_VCI Mask */

 #define MXC_F_CLCD_REVA_INT_EN_BERR_POS                     3 /**< INT_EN_BERR Position */
 #define MXC_F_CLCD_REVA_INT_EN_BERR                         ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_INT_EN_BERR_POS)) /**< INT_EN_BERR Mask */

/**@} end of group CLCD_REVA_INT_EN_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_STAT CLCD_REVA_STAT
 * @brief    LCD Status Register.
 * @{
 */
 #define MXC_F_CLCD_REVA_STAT_UFLO_POS                       0 /**< STAT_UFLO Position */
 #define MXC_F_CLCD_REVA_STAT_UFLO                           ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_STAT_UFLO_POS)) /**< STAT_UFLO Mask */
 #define MXC_V_CLCD_REVA_STAT_UFLO_INACTIVE                  ((uint32_t)0x0UL) /**< STAT_UFLO_INACTIVE Value */
 #define MXC_S_CLCD_REVA_STAT_UFLO_INACTIVE                  (MXC_V_CLCD_REVA_STAT_UFLO_INACTIVE << MXC_F_CLCD_REVA_STAT_UFLO_POS) /**< STAT_UFLO_INACTIVE Setting */
 #define MXC_V_CLCD_REVA_STAT_UFLO_PENDING                   ((uint32_t)0x1UL) /**< STAT_UFLO_PENDING Value */
 #define MXC_S_CLCD_REVA_STAT_UFLO_PENDING                   (MXC_V_CLCD_REVA_STAT_UFLO_PENDING << MXC_F_CLCD_REVA_STAT_UFLO_POS) /**< STAT_UFLO_PENDING Setting */
 #define MXC_V_CLCD_REVA_STAT_UFLO_CLEAR                     ((uint32_t)0x1UL) /**< STAT_UFLO_CLEAR Value */
 #define MXC_S_CLCD_REVA_STAT_UFLO_CLEAR                     (MXC_V_CLCD_REVA_STAT_UFLO_CLEAR << MXC_F_CLCD_REVA_STAT_UFLO_POS) /**< STAT_UFLO_CLEAR Setting */

 #define MXC_F_CLCD_REVA_STAT_ADRRDY_POS                     1 /**< STAT_ADRRDY Position */
 #define MXC_F_CLCD_REVA_STAT_ADRRDY                         ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_STAT_ADRRDY_POS)) /**< STAT_ADRRDY Mask */
 #define MXC_V_CLCD_REVA_STAT_ADRRDY_INACTIVE                ((uint32_t)0x0UL) /**< STAT_ADRRDY_INACTIVE Value */
 #define MXC_S_CLCD_REVA_STAT_ADRRDY_INACTIVE                (MXC_V_CLCD_REVA_STAT_ADRRDY_INACTIVE << MXC_F_CLCD_REVA_STAT_ADRRDY_POS) /**< STAT_ADRRDY_INACTIVE Setting */
 #define MXC_V_CLCD_REVA_STAT_ADRRDY_PENDING                 ((uint32_t)0x1UL) /**< STAT_ADRRDY_PENDING Value */
 #define MXC_S_CLCD_REVA_STAT_ADRRDY_PENDING                 (MXC_V_CLCD_REVA_STAT_ADRRDY_PENDING << MXC_F_CLCD_REVA_STAT_ADRRDY_POS) /**< STAT_ADRRDY_PENDING Setting */
 #define MXC_V_CLCD_REVA_STAT_ADRRDY_CLEAR                   ((uint32_t)0x1UL) /**< STAT_ADRRDY_CLEAR Value */
 #define MXC_S_CLCD_REVA_STAT_ADRRDY_CLEAR                   (MXC_V_CLCD_REVA_STAT_ADRRDY_CLEAR << MXC_F_CLCD_REVA_STAT_ADRRDY_POS) /**< STAT_ADRRDY_CLEAR Setting */

 #define MXC_F_CLCD_REVA_STAT_VCI_POS                        2 /**< STAT_VCI Position */
 #define MXC_F_CLCD_REVA_STAT_VCI                            ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_STAT_VCI_POS)) /**< STAT_VCI Mask */
 #define MXC_V_CLCD_REVA_STAT_VCI_INACTIVE                   ((uint32_t)0x0UL) /**< STAT_VCI_INACTIVE Value */
 #define MXC_S_CLCD_REVA_STAT_VCI_INACTIVE                   (MXC_V_CLCD_REVA_STAT_VCI_INACTIVE << MXC_F_CLCD_REVA_STAT_VCI_POS) /**< STAT_VCI_INACTIVE Setting */
 #define MXC_V_CLCD_REVA_STAT_VCI_PENDING                    ((uint32_t)0x1UL) /**< STAT_VCI_PENDING Value */
 #define MXC_S_CLCD_REVA_STAT_VCI_PENDING                    (MXC_V_CLCD_REVA_STAT_VCI_PENDING << MXC_F_CLCD_REVA_STAT_VCI_POS) /**< STAT_VCI_PENDING Setting */
 #define MXC_V_CLCD_REVA_STAT_VCI_CLEAR                      ((uint32_t)0x1UL) /**< STAT_VCI_CLEAR Value */
 #define MXC_S_CLCD_REVA_STAT_VCI_CLEAR                      (MXC_V_CLCD_REVA_STAT_VCI_CLEAR << MXC_F_CLCD_REVA_STAT_VCI_POS) /**< STAT_VCI_CLEAR Setting */

 #define MXC_F_CLCD_REVA_STAT_BERR_POS                       3 /**< STAT_BERR Position */
 #define MXC_F_CLCD_REVA_STAT_BERR                           ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_STAT_BERR_POS)) /**< STAT_BERR Mask */
 #define MXC_V_CLCD_REVA_STAT_BERR_INACTIVE                  ((uint32_t)0x0UL) /**< STAT_BERR_INACTIVE Value */
 #define MXC_S_CLCD_REVA_STAT_BERR_INACTIVE                  (MXC_V_CLCD_REVA_STAT_BERR_INACTIVE << MXC_F_CLCD_REVA_STAT_BERR_POS) /**< STAT_BERR_INACTIVE Setting */
 #define MXC_V_CLCD_REVA_STAT_BERR_PENDING                   ((uint32_t)0x1UL) /**< STAT_BERR_PENDING Value */
 #define MXC_S_CLCD_REVA_STAT_BERR_PENDING                   (MXC_V_CLCD_REVA_STAT_BERR_PENDING << MXC_F_CLCD_REVA_STAT_BERR_POS) /**< STAT_BERR_PENDING Setting */
 #define MXC_V_CLCD_REVA_STAT_BERR_CLEAR                     ((uint32_t)0x1UL) /**< STAT_BERR_CLEAR Value */
 #define MXC_S_CLCD_REVA_STAT_BERR_CLEAR                     (MXC_V_CLCD_REVA_STAT_BERR_CLEAR << MXC_F_CLCD_REVA_STAT_BERR_POS) /**< STAT_BERR_CLEAR Setting */

 #define MXC_F_CLCD_REVA_STAT_LCDIDLE_POS                    8 /**< STAT_LCDIDLE Position */
 #define MXC_F_CLCD_REVA_STAT_LCDIDLE                        ((uint32_t)(0x1UL << MXC_F_CLCD_REVA_STAT_LCDIDLE_POS)) /**< STAT_LCDIDLE Mask */
 #define MXC_V_CLCD_REVA_STAT_LCDIDLE_BUSY                   ((uint32_t)0x0UL) /**< STAT_LCDIDLE_BUSY Value */
 #define MXC_S_CLCD_REVA_STAT_LCDIDLE_BUSY                   (MXC_V_CLCD_REVA_STAT_LCDIDLE_BUSY << MXC_F_CLCD_REVA_STAT_LCDIDLE_POS) /**< STAT_LCDIDLE_BUSY Setting */
 #define MXC_V_CLCD_REVA_STAT_LCDIDLE_READY                  ((uint32_t)0x1UL) /**< STAT_LCDIDLE_READY Value */
 #define MXC_S_CLCD_REVA_STAT_LCDIDLE_READY                  (MXC_V_CLCD_REVA_STAT_LCDIDLE_READY << MXC_F_CLCD_REVA_STAT_LCDIDLE_POS) /**< STAT_LCDIDLE_READY Setting */

/**@} end of group CLCD_REVA_STAT_Register */

/**
 * @ingroup  clcd_reva_registers
 * @defgroup CLCD_REVA_PALETTE CLCD_REVA_PALETTE
 * @brief    Palette
 * @{
 */
 #define MXC_F_CLCD_REVA_PALETTE_RED_POS                     0 /**< PALETTE_RED Position */
 #define MXC_F_CLCD_REVA_PALETTE_RED                         ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_PALETTE_RED_POS)) /**< PALETTE_RED Mask */

 #define MXC_F_CLCD_REVA_PALETTE_GREEN_POS                   8 /**< PALETTE_GREEN Position */
 #define MXC_F_CLCD_REVA_PALETTE_GREEN                       ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_PALETTE_GREEN_POS)) /**< PALETTE_GREEN Mask */

 #define MXC_F_CLCD_REVA_PALETTE_BLUE_POS                    16 /**< PALETTE_BLUE Position */
 #define MXC_F_CLCD_REVA_PALETTE_BLUE                        ((uint32_t)(0xFFUL << MXC_F_CLCD_REVA_PALETTE_BLUE_POS)) /**< PALETTE_BLUE Mask */

/**@} end of group CLCD_REVA_PALETTE_Register */

#ifdef __cplusplus
}
#endif

#endif /* _CLCD_REVA_REGS_H_ */
