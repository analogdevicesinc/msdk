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
    __IO uint32_t sec;                  /**< <tt>\b 0x00:</tt> HTMR SEC Register */
    __IO uint32_t ssec;                 /**< <tt>\b 0x04:</tt> HTMR SSEC Register */
    __IO uint32_t ras;                  /**< <tt>\b 0x08:</tt> HTMR RAS Register */
    __IO uint32_t rssa;                 /**< <tt>\b 0x0C:</tt> HTMR RSSA Register */
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
#define MXC_R_HTMR_SEC                     ((uint32_t)0x00000000UL) /**< Offset from HTMR Base Address: <tt> 0x0000</tt> */
#define MXC_R_HTMR_SSEC                    ((uint32_t)0x00000004UL) /**< Offset from HTMR Base Address: <tt> 0x0004</tt> */
#define MXC_R_HTMR_RAS                     ((uint32_t)0x00000008UL) /**< Offset from HTMR Base Address: <tt> 0x0008</tt> */
#define MXC_R_HTMR_RSSA                    ((uint32_t)0x0000000CUL) /**< Offset from HTMR Base Address: <tt> 0x000C</tt> */
#define MXC_R_HTMR_CTRL                    ((uint32_t)0x00000010UL) /**< Offset from HTMR Base Address: <tt> 0x0010</tt> */
#define MXC_R_HTMR_TRIM                    ((uint32_t)0x00000014UL) /**< Offset from HTMR Base Address: <tt> 0x0014</tt> */
#define MXC_R_HTMR_OSCCTRL                 ((uint32_t)0x00000018UL) /**< Offset from HTMR Base Address: <tt> 0x0018</tt> */
/**@} end of group htmr_registers */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SEC HTMR_SEC
 * @brief    HTimer Long-Interval Counter. This register contains the 32 most significant
 *           bits of the counter.
 * @{
 */
#define MXC_F_HTMR_SEC_RTS_POS                         0 /**< SEC_RTS Position */
#define MXC_F_HTMR_SEC_RTS                             ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_SEC_RTS_POS)) /**< SEC_RTS Mask */

/**@} end of group HTMR_SEC_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_SSEC HTMR_SSEC
 * @brief    HTimer Short Interval Counter. This counter ticks every t_htclk (16.48uS).
 *           HTIMER_SEC is incremented when this register rolls over from 0xFF to 0x00.
 * @{
 */
#define MXC_F_HTMR_SSEC_RTSS_POS                       0 /**< SSEC_RTSS Position */
#define MXC_F_HTMR_SSEC_RTSS                           ((uint32_t)(0xFFUL << MXC_F_HTMR_SSEC_RTSS_POS)) /**< SSEC_RTSS Mask */

/**@} end of group HTMR_SSEC_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_RAS HTMR_RAS
 * @brief    Long Interval Alarm.
 * @{
 */
#define MXC_F_HTMR_RAS_RAS_POS                         0 /**< RAS_RAS Position */
#define MXC_F_HTMR_RAS_RAS                             ((uint32_t)(0xFFFFFUL << MXC_F_HTMR_RAS_RAS_POS)) /**< RAS_RAS Mask */

/**@} end of group HTMR_RAS_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_RSSA HTMR_RSSA
 * @brief    HTimer Short Interval Alarm. This register contains the reload value for the
 *           short interval alarm, HTIMER_CTRL.alarm_ss_fl is raised on rollover.
 * @{
 */
#define MXC_F_HTMR_RSSA_RSSA_POS                       0 /**< RSSA_RSSA Position */
#define MXC_F_HTMR_RSSA_RSSA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_HTMR_RSSA_RSSA_POS)) /**< RSSA_RSSA Mask */

/**@} end of group HTMR_RSSA_Register */

/**
 * @ingroup  htmr_registers
 * @defgroup HTMR_CTRL HTMR_CTRL
 * @brief    HTimer Control Register.
 * @{
 */
#define MXC_F_HTMR_CTRL_HTEN_POS                       0 /**< CTRL_HTEN Position */
#define MXC_F_HTMR_CTRL_HTEN                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_HTEN_POS)) /**< CTRL_HTEN Mask */

#define MXC_F_HTMR_CTRL_ADE_POS                        1 /**< CTRL_ADE Position */
#define MXC_F_HTMR_CTRL_ADE                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ADE_POS)) /**< CTRL_ADE Mask */

#define MXC_F_HTMR_CTRL_ASE_POS                        2 /**< CTRL_ASE Position */
#define MXC_F_HTMR_CTRL_ASE                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ASE_POS)) /**< CTRL_ASE Mask */

#define MXC_F_HTMR_CTRL_BUSY_POS                       3 /**< CTRL_BUSY Position */
#define MXC_F_HTMR_CTRL_BUSY                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_BUSY_POS)) /**< CTRL_BUSY Mask */

#define MXC_F_HTMR_CTRL_RDY_POS                        4 /**< CTRL_RDY Position */
#define MXC_F_HTMR_CTRL_RDY                            ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

#define MXC_F_HTMR_CTRL_RDYE_POS                       5 /**< CTRL_RDYE Position */
#define MXC_F_HTMR_CTRL_RDYE                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_RDYE_POS)) /**< CTRL_RDYE Mask */

#define MXC_F_HTMR_CTRL_ALDF_POS                       6 /**< CTRL_ALDF Position */
#define MXC_F_HTMR_CTRL_ALDF                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ALDF_POS)) /**< CTRL_ALDF Mask */

#define MXC_F_HTMR_CTRL_ALSF_POS                       7 /**< CTRL_ALSF Position */
#define MXC_F_HTMR_CTRL_ALSF                           ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_ALSF_POS)) /**< CTRL_ALSF Mask */

#define MXC_F_HTMR_CTRL_WE_POS                         15 /**< CTRL_WE Position */
#define MXC_F_HTMR_CTRL_WE                             ((uint32_t)(0x1UL << MXC_F_HTMR_CTRL_WE_POS)) /**< CTRL_WE Mask */

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
