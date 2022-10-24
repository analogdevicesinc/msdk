/**
 * @file    ecc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the ECC Peripheral Module.
 *          This file is @deprecated.
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

#ifndef _ECC_REGS_H_
#define _ECC_REGS_H_

/* **** Includes **** */
#include <stdint.h>
#include <stdio.h>

#warning "DEPRECATED(10-24-2022): ecc_regs.h - Scheduled for removal. Please use trimsir_regs.h."

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
 * @ingroup     ecc
 * @defgroup    ecc_registers ECC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the ECC Peripheral Module.
 * @details Error Correction Code
 */

/**
 * @ingroup ecc_registers
 * Structure type to access the ECC Registers.
 */
typedef struct __attribute__((deprecated("struct and ecc_regs.h no longer supported. Use trimsir_regs.h and MXC_TRIMSIR (mxc_trimsir_regs_t) for ECC. 10-24-2022"))) {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t en;                   /**< <tt>\b 0x08:</tt> ECC EN Register */
} mxc_ecc_regs_t;

/* Register offsets for module ECC */
/**
 * @ingroup    ecc_registers
 * @defgroup   ECC_Register_Offsets Register Offsets
 * @brief      ECC Peripheral Register Offsets from the ECC Base Peripheral Address. 
 * @{
 */
 #define MXC_R_ECC_EN                       ((uint32_t)0x00000008UL) /**< Offset from ECC Base Address: <tt> 0x0008</tt> */ 
/**@} end of group ecc_registers */

/**
 * @ingroup  ecc_registers
 * @defgroup ECC_EN ECC_EN
 * @brief    ECC Enable Register
 * @{
 */
 #define MXC_F_ECC_EN_RAM0_1_POS                        8 /**< EN_RAM0_1 Position */
 #define MXC_F_ECC_EN_RAM0_1                            ((uint32_t)(0x1UL << MXC_F_ECC_EN_RAM0_1_POS)) /**< EN_RAM0_1 Mask */

 #define MXC_F_ECC_EN_RAM2_POS                          9 /**< EN_RAM2 Position */
 #define MXC_F_ECC_EN_RAM2                              ((uint32_t)(0x1UL << MXC_F_ECC_EN_RAM2_POS)) /**< EN_RAM2 Mask */

 #define MXC_F_ECC_EN_RAM3_POS                          10 /**< EN_RAM3 Position */
 #define MXC_F_ECC_EN_RAM3                              ((uint32_t)(0x1UL << MXC_F_ECC_EN_RAM3_POS)) /**< EN_RAM3 Mask */

 #define MXC_F_ECC_EN_ICC0_POS                          11 /**< EN_ICC0 Position */
 #define MXC_F_ECC_EN_ICC0                              ((uint32_t)(0x1UL << MXC_F_ECC_EN_ICC0_POS)) /**< EN_ICC0 Mask */

 #define MXC_F_ECC_EN_FL0_POS                           _Pragma ("This field is deprecated. Use MXC_R_TRIMSIR_BB_SIR2. 10-24-2022.")12 /**< EN_FL0 Position */
 #define MXC_F_ECC_EN_FL0                               ((uint32_t)(0x1UL << MXC_F_ECC_EN_FL0_POS)) /**< EN_FL0 Mask */

 #define MXC_F_ECC_EN_FL1_POS                           13 /**< EN_FL1 Position */
 #define MXC_F_ECC_EN_FL1                               ((uint32_t)(0x1UL << MXC_F_ECC_EN_FL1_POS)) /**< EN_FL1 Mask */

/**@} end of group ECC_EN_Register */

#ifdef __cplusplus
}
#endif

#endif /* _ECC_REGS_H_ */
