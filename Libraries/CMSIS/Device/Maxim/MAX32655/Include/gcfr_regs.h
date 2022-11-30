/**
 * @file    gcfr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the GCFR Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_GCFR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_GCFR_REGS_H_

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
 * @ingroup     gcfr
 * @defgroup    gcfr_registers GCFR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the GCFR Peripheral Module.
 * @details     Global Control Function Register.
 */

/**
 * @ingroup gcfr_registers
 * Structure type to access the GCFR Registers.
 */
typedef struct {
    __IO uint32_t reg0;                 /**< <tt>\b 0x00:</tt> GCFR REG0 Register */
    __IO uint32_t reg1;                 /**< <tt>\b 0x04:</tt> GCFR REG1 Register */
} mxc_gcfr_regs_t;

/* Register offsets for module GCFR */
/**
 * @ingroup    gcfr_registers
 * @defgroup   GCFR_Register_Offsets Register Offsets
 * @brief      GCFR Peripheral Register Offsets from the GCFR Base Peripheral Address.
 * @{
 */
#define MXC_R_GCFR_REG0                    ((uint32_t)0x00000000UL) /**< Offset from GCFR Base Address: <tt> 0x0000</tt> */
#define MXC_R_GCFR_REG1                    ((uint32_t)0x00000004UL) /**< Offset from GCFR Base Address: <tt> 0x0004</tt> */
/**@} end of group gcfr_registers */

/**
 * @ingroup  gcfr_registers
 * @defgroup GCFR_REG0 GCFR_REG0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_GCFR_REG0_ISO_WUP_POS                    0 /**< REG0_ISO_WUP Position */
#define MXC_F_GCFR_REG0_ISO_WUP                        ((uint32_t)(0x1FFUL << MXC_F_GCFR_REG0_ISO_WUP_POS)) /**< REG0_ISO_WUP Mask */

#define MXC_F_GCFR_REG0_IPO_WUP_POS                    16 /**< REG0_IPO_WUP Position */
#define MXC_F_GCFR_REG0_IPO_WUP                        ((uint32_t)(0x7FFUL << MXC_F_GCFR_REG0_IPO_WUP_POS)) /**< REG0_IPO_WUP Mask */

/**@} end of group GCFR_REG0_Register */

/**
 * @ingroup  gcfr_registers
 * @defgroup GCFR_REG1 GCFR_REG1
 * @brief    Register 1.
 * @{
 */
#define MXC_F_GCFR_REG1_ERFO_WUP_POS                   0 /**< REG1_ERFO_WUP Position */
#define MXC_F_GCFR_REG1_ERFO_WUP                       ((uint32_t)(0x3FFFUL << MXC_F_GCFR_REG1_ERFO_WUP_POS)) /**< REG1_ERFO_WUP Mask */

#define MXC_F_GCFR_REG1_IBRO_WUP_POS                   16 /**< REG1_IBRO_WUP Position */
#define MXC_F_GCFR_REG1_IBRO_WUP                       ((uint32_t)(0x3FUL << MXC_F_GCFR_REG1_IBRO_WUP_POS)) /**< REG1_IBRO_WUP Mask */

/**@} end of group GCFR_REG1_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_GCFR_REGS_H_
