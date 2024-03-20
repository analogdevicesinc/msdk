/**
 * @file    htmr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the HTMR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup htmr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_HTMR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_HTMR_REGS_H_

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
 * @ingroup     htmr
 * @defgroup    htmr_registers HTMR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the HTMR Peripheral Module.
 * @details     High Speed Timer Module.
 */

/**
 * @ingroup htmr_registers
 * Structure type to access the HTMR Registers.
 */
typedef struct {
    __IO uint32_t lnicnt;               /**< <tt>\b 0x00:</tt> HTMR LNICNT Register */
    __IO uint32_t shicnt;               /**< <tt>\b 0x04:</tt> HTMR SHICNT Register */
    __IO uint32_t lnialm;               /**< <tt>\b 0x08:</tt> HTMR LNIALM Register */
    __IO uint32_t shialm;               /**< <tt>\b 0x0C:</tt> HTMR SHIALM Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> HTMR CTRL Register */
    __IO uint32_t trim;                 /**< <tt>\b 0x14:</tt> HTMR TRIM Register */
    __IO uint32_t oscctrl;              /**< <tt>\b 0x18:</tt> HTMR OSCCTRL Register */
} mxc_htmr_regs_t;

/* Register offsets for module HTMR */
/**
 * @ingroup    htmr_registers
 * @defgroup   HTMR_Register_Offsets Register Offsets
 * @brief      HTMR Peripheral Register Offsets from the HTMR Base Peripheral Address.
 * @{
 */
#define MXC_R_HTMR_LNICNT                  ((uint32_t)0x00000000UL) /**< Offset from HTMR Base Address: <tt> 0x0000</tt> */
#define MXC_R_HTMR_SHICNT                  ((uint32_t)0x00000004UL) /**< Offset from HTMR Base Address: <tt> 0x0004</tt> */
#define MXC_R_HTMR_LNIALM                  ((uint32_t)0x00000008UL) /**< Offset from HTMR Base Address: <tt> 0x0008</tt> */
#define MXC_R_HTMR_SHIALM                  ((uint32_t)0x0000000CUL) /**< Offset from HTMR Base Address: <tt> 0x000C</tt> */
#define MXC_R_HTMR_CTRL                    ((uint32_t)0x00000010UL) /**< Offset from HTMR Base Address: <tt> 0x0010</tt> */
#define MXC_R_HTMR_TRIM                    ((uint32_t)0x00000014UL) /**< Offset from HTMR Base Address: <tt> 0x0014</tt> */
#define MXC_R_HTMR_OSCCTRL                 ((uint32_t)0x00000018UL) /**< Offset from HTMR Base Address: <tt> 0x0018</tt> */
/**@} end of group htmr_registers */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_LNICNT HTMR_LNICNT
 * @brief    HTimer Long-Interval Counter. This register contains the 32 most significant
 *           bits of the counter.
 * @{
 */
#define MXC_F_HTMR_LNICNT_CNT_POS                      0 /**< LNICNT_CNT Position */
#define MXC_F_HTMR_LNICNT_CNT                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_LNICNT_CNT_POS)) /**< LNICNT_CNT Mask */

/**@} end of group HTMR_LNICNT_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SHICNT HTMR_SHICNT
 * @brief    HTimer Short Interval Counter. This counter ticks every t_htclk (16.48uS).
 *           HTIMER_SEC is incremented when this register rolls over from 0xFF to 0x00.
 * @{
 */
#define MXC_F_HTMR_SHICNT_CNT_POS                      0 /**< SHICNT_CNT Position */
#define MXC_F_HTMR_SHICNT_CNT                          ((uint32_t)(0xFFUL << MXC_F_HTMR_SHICNT_CNT_POS)) /**< SHICNT_CNT Mask */

/**@} end of group HTMR_SHICNT_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_LNIALM HTMR_LNIALM
 * @brief    HTimer Long Interval Alarm Value Register.
 * @{
 */
#define MXC_F_HTMR_LNIALM_ALM_POS                      0 /**< LNIALM_ALM Position */
#define MXC_F_HTMR_LNIALM_ALM                          ((uint32_t)(0xFFFFFUL << MXC_F_HTMR_LNIALM_ALM_POS)) /**< LNIALM_ALM Mask */

/**@} end of group HTMR_LNIALM_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SHIALM HTMR_SHIALM
 * @brief    HTimer Short Interval Alarm Value Register.
 * @{
 */
#define MXC_F_HTMR_SHIALM_ALM_POS                      0 /**< SHIALM_ALM Position */
#define MXC_F_HTMR_SHIALM_ALM                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_SHIALM_ALM_POS)) /**< SHIALM_ALM Mask */

/**@} end of group HTMR_SHIALM_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_CTRL HTMR_CTRL
 * @brief    HTimer Control Register.
 * @{
 */
#define MXC_F_HTMR_CTRL_EN_POS                         0 /**< CTRL_EN Position */
#define MXC_F_HTMR_CTRL_EN                             ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_HTMR_CTRL_LONG_ALM_IE_POS                1 /**< CTRL_LONG_ALM_IE Position */
#define MXC_F_HTMR_CTRL_LONG_ALM_IE                    ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_LONG_ALM_IE_POS)) /**< CTRL_LONG_ALM_IE Mask */

#define MXC_F_HTMR_CTRL_SHORT_ALM_IE_POS               2 /**< CTRL_SHORT_ALM_IE Position */
#define MXC_F_HTMR_CTRL_SHORT_ALM_IE                   ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_SHORT_ALM_IE_POS)) /**< CTRL_SHORT_ALM_IE Mask */

#define MXC_F_HTMR_CTRL_BUSY_POS                       3 /**< CTRL_BUSY Position */
#define MXC_F_HTMR_CTRL_BUSY                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_BUSY_POS)) /**< CTRL_BUSY Mask */

#define MXC_F_HTMR_CTRL_RDY_POS                        4 /**< CTRL_RDY Position */
#define MXC_F_HTMR_CTRL_RDY                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

#define MXC_F_HTMR_CTRL_RDY_IE_POS                     5 /**< CTRL_RDY_IE Position */
#define MXC_F_HTMR_CTRL_RDY_IE                         ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDY_IE_POS)) /**< CTRL_RDY_IE Mask */

#define MXC_F_HTMR_CTRL_LONG_ALM_IF_POS                6 /**< CTRL_LONG_ALM_IF Position */
#define MXC_F_HTMR_CTRL_LONG_ALM_IF                    ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_LONG_ALM_IF_POS)) /**< CTRL_LONG_ALM_IF Mask */

#define MXC_F_HTMR_CTRL_SHORT_ALM_IF_POS               7 /**< CTRL_SHORT_ALM_IF Position */
#define MXC_F_HTMR_CTRL_SHORT_ALM_IF                   ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_SHORT_ALM_IF_POS)) /**< CTRL_SHORT_ALM_IF Mask */

#define MXC_F_HTMR_CTRL_WR_EN_POS                      15 /**< CTRL_WR_EN Position */
#define MXC_F_HTMR_CTRL_WR_EN                          ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_WR_EN_POS)) /**< CTRL_WR_EN Mask */

/**@} end of group HTMR_CTRL_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_TRIM HTMR_TRIM
 * @brief    HTimer Trim Register.
 * @{
 */
#define MXC_F_HTMR_TRIM_TRIM_POS                       0 /**< TRIM_TRIM Position */
#define MXC_F_HTMR_TRIM_TRIM                           ((uint32_t)(0xFFUL << MXC_F_HTMR_TRIM_TRIM_POS)) /**< TRIM_TRIM Mask */

#define MXC_F_HTMR_TRIM_VBAT_TMR_POS                   8 /**< TRIM_VBAT_TMR Position */
#define MXC_F_HTMR_TRIM_VBAT_TMR                       ((uint32_t)(0xFFFFFFUL << MXC_F_HTMR_TRIM_VBAT_TMR_POS)) /**< TRIM_VBAT_TMR Mask */

/**@} end of group HTMR_TRIM_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_OSCCTRL HTMR_OSCCTRL
 * @brief    HTimer Oscillator Control Register.
 * @{
 */
#define MXC_F_HTMR_OSCCTRL_FILTER_EN_POS               0 /**< OSCCTRL_FILTER_EN Position */
#define MXC_F_HTMR_OSCCTRL_FILTER_EN                   ((uint32_t)(0x1UL << MXC_F_HTMR_OSCCTRL_FILTER_EN_POS)) /**< OSCCTRL_FILTER_EN Mask */

#define MXC_F_HTMR_OSCCTRL_IBIAS_SEL_POS               1 /**< OSCCTRL_IBIAS_SEL Position */
#define MXC_F_HTMR_OSCCTRL_IBIAS_SEL                   ((uint32_t)(0x1UL << MXC_F_HTMR_OSCCTRL_IBIAS_SEL_POS)) /**< OSCCTRL_IBIAS_SEL Mask */

#define MXC_F_HTMR_OSCCTRL_HYST_EN_POS                 2 /**< OSCCTRL_HYST_EN Position */
#define MXC_F_HTMR_OSCCTRL_HYST_EN                     ((uint32_t)(0x1UL << MXC_F_HTMR_OSCCTRL_HYST_EN_POS)) /**< OSCCTRL_HYST_EN Mask */

#define MXC_F_HTMR_OSCCTRL_IBIAS_EN_POS                3 /**< OSCCTRL_IBIAS_EN Position */
#define MXC_F_HTMR_OSCCTRL_IBIAS_EN                    ((uint32_t)(0x1UL << MXC_F_HTMR_OSCCTRL_IBIAS_EN_POS)) /**< OSCCTRL_IBIAS_EN Mask */

#define MXC_F_HTMR_OSCCTRL_BYPASS_POS                  4 /**< OSCCTRL_BYPASS Position */
#define MXC_F_HTMR_OSCCTRL_BYPASS                      ((uint32_t)(0x1UL << MXC_F_HTMR_OSCCTRL_BYPASS_POS)) /**< OSCCTRL_BYPASS Mask */

#define MXC_F_HTMR_OSCCTRL_SQW_32K_POS                 5 /**< OSCCTRL_SQW_32K Position */
#define MXC_F_HTMR_OSCCTRL_SQW_32K                     ((uint32_t)(0x1UL << MXC_F_HTMR_OSCCTRL_SQW_32K_POS)) /**< OSCCTRL_SQW_32K Mask */

/**@} end of group HTMR_OSCCTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_HTMR_REGS_H_
