/**
 * @file    aeskeys_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AES Keys Peripheral Module.
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

#ifndef _AESKEYS_REGS_H_
#define _AESKEYS_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
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
 * @ingroup     aeskeys
 * @defgroup    aeskeys_registers Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AEs Keys Peripheral Module.
 * @description AEs Keys Registers (For Simulation)
 */

/**
 * @ingroup aeskeys_registers
 * Structure type to access the AES Keys Registers.
 */
typedef struct {
    __IO uint32_t key0_tm;              /**< <tt>\b 0x00:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key1_tm;              /**< <tt>\b 0x04:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key2_tm;              /**< <tt>\b 0x08:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key3_tm;              /**< <tt>\b 0x0C:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key4_tm;              /**< <tt>\b 0x10:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key5_tm;              /**< <tt>\b 0x14:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key6_tm;              /**< <tt>\b 0x18:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key7_tm;              /**< <tt>\b 0x1C:<\tt> AES-256 SRAM Encryption Key (MEU) */
    __IO uint32_t key0_qspi_dec;        /**< <tt>\b 0x20:<\tt> AES-128 QSPI Decryption Key (MDIU) */
    __IO uint32_t key1_qspi_dec;        /**< <tt>\b 0x24:<\tt> AES-128 QSPI Decryption Key (MDIU) */
    __IO uint32_t key2_qspi_dec;        /**< <tt>\b 0x28:<\tt> AES-128 QSPI Decryption Key (MDIU) */
    __IO uint32_t key3_qspi_dec;        /**< <tt>\b 0x2C:<\tt> AES-128 QSPI Decryption Key (MDIU) */
    __IO uint32_t key0_qspi_auth;       /**< <tt>\b 0x30:<\tt> AES-128 QSPI Authentication Key (MDIU) */
    __IO uint32_t key1_qspi_auth;       /**< <tt>\b 0x34:<\tt> AES-128 QSPI Authentication Key (MDIU) */
    __IO uint32_t key2_qspi_auth;       /**< <tt>\b 0x38:<\tt> AES-128 QSPI Authentication Key (MDIU) */
    __IO uint32_t key3_qspi_auth;       /**< <tt>\b 0x3C:<\tt> AES-128 QSPI Authentication Key (MDIU) */
} mxc_aeskeys_regs_t;

/*******************************************************************************/
/*                                                                     AESKEYS */
#define MXC_BASE_AESKEYS                  ((uint32_t)0x40005000UL)
#define MXC_AESKEYS                       ((mxc_aeskeys_regs_t*)MXC_BASE_AESKEYS)
#define MXC_AESKEYS_INSTANCES             (1)

#ifdef __cplusplus
}
#endif

#endif /* _AESKEYS_REGS_H_ */

