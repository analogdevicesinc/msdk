/**
 * @file    sir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_SIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_SIR_REGS_H_

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
 * @ingroup     sir
 * @defgroup    sir_registers SIR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @details     System Initialization Registers.
 */

/**
 * @ingroup sir_registers
 * Structure type to access the SIR Registers.
 */
typedef struct {
    __I  uint32_t sistat;               /**< <tt>\b 0x00:</tt> SIR SISTAT Register */
    __I  uint32_t siaddr;               /**< <tt>\b 0x04:</tt> SIR SIADDR Register */
} mxc_sir_regs_t;

/* Register offsets for module SIR */
/**
 * @ingroup    sir_registers
 * @defgroup   SIR_Register_Offsets Register Offsets
 * @brief      SIR Peripheral Register Offsets from the SIR Base Peripheral Address.
 * @{
 */
#define MXC_R_SIR_SISTAT                   ((uint32_t)0x00000000UL) /**< Offset from SIR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SIR_SIADDR                   ((uint32_t)0x00000004UL) /**< Offset from SIR Base Address: <tt> 0x0004</tt> */
/**@} end of group sir_registers */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SISTAT SIR_SISTAT
 * @brief    System Initialization Status Register.
 * @{
 */
#define MXC_F_SIR_SISTAT_MAGIC_POS                     0 /**< SISTAT_MAGIC Position */
#define MXC_F_SIR_SISTAT_MAGIC                         ((uint32_t)(0x1UL << MXC_F_SIR_SISTAT_MAGIC_POS)) /**< SISTAT_MAGIC Mask */

#define MXC_F_SIR_SISTAT_CRCERR_POS                    1 /**< SISTAT_CRCERR Position */
#define MXC_F_SIR_SISTAT_CRCERR                        ((uint32_t)(0x1UL << MXC_F_SIR_SISTAT_CRCERR_POS)) /**< SISTAT_CRCERR Mask */

#define MXC_F_SIR_SISTAT_USER_MAGIC_POS                2 /**< SISTAT_USER_MAGIC Position */
#define MXC_F_SIR_SISTAT_USER_MAGIC                    ((uint32_t)(0x1UL << MXC_F_SIR_SISTAT_USER_MAGIC_POS)) /**< SISTAT_USER_MAGIC Mask */

/**@} end of group SIR_SISTAT_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIADDR SIR_SIADDR
 * @brief    System Initialization Address Register.
 * @{
 */
#define MXC_F_SIR_SIADDR_ERRADDR_POS                   0 /**< SIADDR_ERRADDR Position */
#define MXC_F_SIR_SIADDR_ERRADDR                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIADDR_ERRADDR_POS)) /**< SIADDR_ERRADDR Mask */

/**@} end of group SIR_SIADDR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_SIR_REGS_H_
