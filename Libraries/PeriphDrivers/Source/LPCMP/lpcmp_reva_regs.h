/**
 * @file    lpcmp_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the LPCMP_REVA Peripheral Module.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_REGS_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_REGS_H_

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
 * @ingroup     lpcmp_reva
 * @defgroup    lpcmp_reva_registers LPCMP_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the LPCMP_REVA Peripheral Module.
 * @details Low Power Comparator
 */

/**
 * @ingroup lpcmp_reva_registers
 * Structure type to access the LPCMP_REVA Registers.
 */
typedef struct {
    __IO uint32_t ctrl[3];              /**< <tt>\b 0x00:</tt> LPCMP_REVA CTRL Register */
} mxc_lpcmp_reva_regs_t;

/* Register offsets for module LPCMP_REVA */
/**
 * @ingroup    lpcmp_reva_registers
 * @defgroup   LPCMP_REVA_Register_Offsets Register Offsets
 * @brief      LPCMP_REVA Peripheral Register Offsets from the LPCMP_REVA Base Peripheral Address.
 * @{
 */
#define MXC_R_LPCMP_REVA_CTRL              ((uint32_t)0x00000000UL) /**< Offset from LPCMP_REVA Base Address: <tt> 0x0000</tt> */
/**@} end of group lpcmp_reva_registers */

/**
 * @ingroup  lpcmp_reva_registers
 * @defgroup LPCMP_REVA_CTRL LPCMP_REVA_CTRL
 * @brief    Comparator Control Register.
 * @{
 */
#define MXC_F_LPCMP_REVA_CTRL_EN_POS                   0 /**< CTRL_EN Position */
#define MXC_F_LPCMP_REVA_CTRL_EN                       ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_LPCMP_REVA_CTRL_POL_POS                  5 /**< CTRL_POL Position */
#define MXC_F_LPCMP_REVA_CTRL_POL                      ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_POL_POS)) /**< CTRL_POL Mask */

#define MXC_F_LPCMP_REVA_CTRL_INT_EN_POS               6 /**< CTRL_INT_EN Position */
#define MXC_F_LPCMP_REVA_CTRL_INT_EN                   ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_INT_EN_POS)) /**< CTRL_INT_EN Mask */

#define MXC_F_LPCMP_REVA_CTRL_OUT_POS                  14 /**< CTRL_OUT Position */
#define MXC_F_LPCMP_REVA_CTRL_OUT                      ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_OUT_POS)) /**< CTRL_OUT Mask */

#define MXC_F_LPCMP_REVA_CTRL_INT_FL_POS               15 /**< CTRL_INT_FL Position */
#define MXC_F_LPCMP_REVA_CTRL_INT_FL                   ((uint32_t)(0x1UL << MXC_F_LPCMP_REVA_CTRL_INT_FL_POS)) /**< CTRL_INT_FL Mask */

/**@} end of group LPCMP_REVA_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif  // LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_REGS_H_

