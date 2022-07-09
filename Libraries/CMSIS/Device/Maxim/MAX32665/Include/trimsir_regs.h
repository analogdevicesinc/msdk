/**
 * @file    trimsir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 */

/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 *
 *************************************************************************** */

#ifndef _TRIMSIR_REGS_H_
#define _TRIMSIR_REGS_H_

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
 * @ingroup     trimsir
 * @defgroup    trimsir_registers TRIMSIR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @details Trim System Initilazation Registers
 */

/**
 * @ingroup trimsir_registers
 * Structure type to access the TRIMSIR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t rtc;                  /**< <tt>\b 0x08:</tt> TRIMSIR RTC Register */
} mxc_trimsir_regs_t;

/* Register offsets for module TRIMSIR */
/**
 * @ingroup    trimsir_registers
 * @defgroup   TRIMSIR_Register_Offsets Register Offsets
 * @brief      TRIMSIR Peripheral Register Offsets from the TRIMSIR Base Peripheral Address. 
 * @{
 */
 #define MXC_R_TRIMSIR_RTC                  ((uint32_t)0x00000008UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0000</tt> */ 
/**@} end of group trimsir_registers */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_RTC TRIMSIR_RTC
 * @brief    RTC Trim System Initialization Register.
 * @{
 */
 #define MXC_F_TRIMSIR_RTC_RTCX1_POS                    16 /**< RTC_RTCX1 Position */
 #define MXC_F_TRIMSIR_RTC_RTCX1                        ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTC_RTCX1_POS)) /**< RTC_RTCX1 Mask */

 #define MXC_F_TRIMSIR_RTC_RTCX2_POS                    21 /**< RTC_RTCX2 Position */
 #define MXC_F_TRIMSIR_RTC_RTCX2                        ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTC_RTCX2_POS)) /**< RTC_RTCX2 Mask */

 #define MXC_F_TRIMSIR_RTC_LOCK_POS                     31 /**< RTC_LOCK Position */
 #define MXC_F_TRIMSIR_RTC_LOCK                         ((uint32_t)(0x1UL << MXC_F_TRIMSIR_RTC_LOCK_POS)) /**< RTC_LOCK Mask */

/**@} end of group TRIMSIR_RTC_Register */

#ifdef __cplusplus
}
#endif

#endif /* _TRIMSIR_REGS_H_ */
