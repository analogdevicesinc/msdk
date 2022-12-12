/**
 * @file    htmr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the HTMR Peripheral Module.
 * @note    This file is @generated.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
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
    __IO uint32_t long;                 /**< <tt>\b 0x00:</tt> HTMR LONG Register */
    __IO uint32_t short;                /**< <tt>\b 0x04:</tt> HTMR SHORT Register */
    __IO uint32_t longa;                /**< <tt>\b 0x08:</tt> HTMR LONGA Register */
    __IO uint32_t shorta;               /**< <tt>\b 0x0C:</tt> HTMR SHORTA Register */
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
#define MXC_R_HTMR_LONG                    ((uint32_t)0x00000000UL) /**< Offset from HTMR Base Address: <tt> 0x0000</tt> */
#define MXC_R_HTMR_SHORT                   ((uint32_t)0x00000004UL) /**< Offset from HTMR Base Address: <tt> 0x0004</tt> */
#define MXC_R_HTMR_LONGA                   ((uint32_t)0x00000008UL) /**< Offset from HTMR Base Address: <tt> 0x0008</tt> */
#define MXC_R_HTMR_SHORTA                  ((uint32_t)0x0000000CUL) /**< Offset from HTMR Base Address: <tt> 0x000C</tt> */
#define MXC_R_HTMR_CTRL                    ((uint32_t)0x00000010UL) /**< Offset from HTMR Base Address: <tt> 0x0010</tt> */
#define MXC_R_HTMR_TRIM                    ((uint32_t)0x00000014UL) /**< Offset from HTMR Base Address: <tt> 0x0014</tt> */
#define MXC_R_HTMR_OSCCTRL                 ((uint32_t)0x00000018UL) /**< Offset from HTMR Base Address: <tt> 0x0018</tt> */
/**@} end of group htmr_registers */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_LONG HTMR_LONG
 * @brief    HTimer Long-Interval Counter. This register contains the 32 most significant
 *           bits of the counter.
 * @{
 */
#define MXC_F_HTMR_LONG_CNT_POS                        0 /**< LONG_CNT Position */
#define MXC_F_HTMR_LONG_CNT                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_LONG_CNT_POS)) /**< LONG_CNT Mask */

/**@} end of group HTMR_LONG_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SHORT HTMR_SHORT
 * @brief    HTimer Short Interval Counter. This counter ticks every t_htclk (16.48uS).
 *           HTIMER_LONG is incremented when this register rolls over from 0xFF to 0x00.
 * @{
 */
#define MXC_F_HTMR_SHORT_CNT_POS                       0 /**< SHORT_CNT Position */
#define MXC_F_HTMR_SHORT_CNT                           ((uint32_t)(0xFFUL << MXC_F_HTMR_SHORT_CNT_POS)) /**< SHORT_CNT Mask */

/**@} end of group HTMR_SHORT_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_LONGA HTMR_LONGA
 * @brief    Long Interval Alarm.
 * @{
 */
#define MXC_F_HTMR_LONGA_ALARM_POS                     0 /**< LONGA_ALARM Position */
#define MXC_F_HTMR_LONGA_ALARM                         ((uint32_t)(0xFFFFFUL << MXC_F_HTMR_LONGA_ALARM_POS)) /**< LONGA_ALARM Mask */

/**@} end of group HTMR_LONGA_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SHORTA HTMR_SHORTA
 * @brief    HTimer Short Interval Alarm. This register contains the reload value for the
 *           short interval alarm, HTIMER_CTRL.alarm_ss_fl is raised on rollover.
 * @{
 */
#define MXC_F_HTMR_SHORTA_ALARM_POS                    0 /**< SHORTA_ALARM Position */
#define MXC_F_HTMR_SHORTA_ALARM                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_SHORTA_ALARM_POS)) /**< SHORTA_ALARM Mask */

/**@} end of group HTMR_SHORTA_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_CTRL HTMR_CTRL
 * @brief    HTimer Control Register.
 * @{
 */
#define MXC_F_HTMR_CTRL_EN_POS                         0 /**< CTRL_EN Position */
#define MXC_F_HTMR_CTRL_EN                             ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_HTMR_CTRL_LONGA_IE_POS                   1 /**< CTRL_LONGA_IE Position */
#define MXC_F_HTMR_CTRL_LONGA_IE                       ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_LONGA_IE_POS)) /**< CTRL_LONGA_IE Mask */

#define MXC_F_HTMR_CTRL_SHORTA_IE_POS                  2 /**< CTRL_SHORTA_IE Position */
#define MXC_F_HTMR_CTRL_SHORTA_IE                      ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_SHORTA_IE_POS)) /**< CTRL_SHORTA_IE Mask */

#define MXC_F_HTMR_CTRL_BUSY_POS                       3 /**< CTRL_BUSY Position */
#define MXC_F_HTMR_CTRL_BUSY                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_BUSY_POS)) /**< CTRL_BUSY Mask */

#define MXC_F_HTMR_CTRL_RDY_POS                        4 /**< CTRL_RDY Position */
#define MXC_F_HTMR_CTRL_RDY                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

#define MXC_F_HTMR_CTRL_RDY_IE_POS                     5 /**< CTRL_RDY_IE Position */
#define MXC_F_HTMR_CTRL_RDY_IE                         ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDY_IE_POS)) /**< CTRL_RDY_IE Mask */

#define MXC_F_HTMR_CTRL_LONGA_IF_POS                   6 /**< CTRL_LONGA_IF Position */
#define MXC_F_HTMR_CTRL_LONGA_IF                       ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_LONGA_IF_POS)) /**< CTRL_LONGA_IF Mask */

#define MXC_F_HTMR_CTRL_SHORTA_IF_POS                  7 /**< CTRL_SHORTA_IF Position */
#define MXC_F_HTMR_CTRL_SHORTA_IF                      ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_SHORTA_IF_POS)) /**< CTRL_SHORTA_IF Mask */

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

#define MXC_F_HTMR_TRIM_VBAT_HTMR_POS                  8 /**< TRIM_VBAT_HTMR Position */
#define MXC_F_HTMR_TRIM_VBAT_HTMR                      ((uint32_t)(0xFFFFFFUL << MXC_F_HTMR_TRIM_VBAT_HTMR_POS)) /**< TRIM_VBAT_HTMR Mask */

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
