/**
 * @file    usr_aeskeys_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the USR_AESKEYS Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_USR_AESKEYS_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_USR_AESKEYS_REGS_H_

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
 * @ingroup     usr_aeskeys
 * @defgroup    usr_aeskeys_registers USR_AESKEYS_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the USR_AESKEYS Peripheral Module.
 * @details     User AES Key Registers.
 */

/**
 * @ingroup usr_aeskeys_registers
 * Structure type to access the USR_AESKEYS Registers.
 */
typedef struct {
    __IO uint32_t sram_key;             /**< <tt>\b 0x00:</tt> USR_AESKEYS SRAM_KEY Register */
    __R  uint32_t rsv_0x4_0x1f[7];
    __IO uint32_t code_key;             /**< <tt>\b 0x20:</tt> USR_AESKEYS CODE_KEY Register */
    __R  uint32_t rsv_0x24_0x3f[7];
    __IO uint32_t data_key;             /**< <tt>\b 0x40:</tt> USR_AESKEYS DATA_KEY Register */
} mxc_usr_aeskeys_regs_t;

/* Register offsets for module USR_AESKEYS */
/**
 * @ingroup    usr_aeskeys_registers
 * @defgroup   USR_AESKEYS_Register_Offsets Register Offsets
 * @brief      USR_AESKEYS Peripheral Register Offsets from the USR_AESKEYS Base Peripheral Address.
 * @{
 */
#define MXC_R_USR_AESKEYS_SRAM_KEY         ((uint32_t)0x00000000UL) /**< Offset from USR_AESKEYS Base Address: <tt> 0x0000</tt> */
#define MXC_R_USR_AESKEYS_CODE_KEY         ((uint32_t)0x00000020UL) /**< Offset from USR_AESKEYS Base Address: <tt> 0x0020</tt> */
#define MXC_R_USR_AESKEYS_DATA_KEY         ((uint32_t)0x00000040UL) /**< Offset from USR_AESKEYS Base Address: <tt> 0x0040</tt> */
/**@} end of group usr_aeskeys_registers */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_USR_AESKEYS_REGS_H_
