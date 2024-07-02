/**
 * @file    aes_key_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AES_KEY Peripheral Module.
 * @note    This file is @deprecated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AES_KEY_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AES_KEY_REGS_H_

#warning "DEPRECATED(1-10-2023): aes_key_regs.h - Scheduled for removal. Please use aeskeys_regs.h."

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
 * @ingroup     aes_key
 * @defgroup    aes_key_registers AES_KEY_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AES_KEY Peripheral Module.
 * @details     AES Key Registers.
 */

/**
 * @ingroup aes_key_registers
 * Structure type to access the AES_KEY Registers.
 */
#if defined(__GNUC__)
__attribute__((deprecated("mxc_aes_key_regs_t struct and aes_key_regs.h no longer supported. Use aeskeys_regs.h and MXC_AESKEYS (mxc_aeskeys_regs_t) for AES Key Access. 1-10-2023")))
#else
#warning "mxc_aes_key_regs_t struct and aes_key_regs.h no longer supported. Use aeskeys_regs.h and MXC_AESKEYS (mxc_aeskeys_regs_t) for AES Key Access. 1-10-2023"
#endif
typedef struct {
    __IO uint32_t aes_key0;             /**< <tt>\b 0x00:</tt> AES_KEY AES_KEY0 Register */
    __IO uint32_t aes_key1;             /**< <tt>\b 0x04:</tt> AES_KEY AES_KEY1 Register */
    __IO uint32_t aes_key2;             /**< <tt>\b 0x08:</tt> AES_KEY AES_KEY2 Register */
    __IO uint32_t aes_key3;             /**< <tt>\b 0x0C:</tt> AES_KEY AES_KEY3 Register */
    __IO uint32_t aes_key4;             /**< <tt>\b 0x10:</tt> AES_KEY AES_KEY4 Register */
    __IO uint32_t aes_key5;             /**< <tt>\b 0x14:</tt> AES_KEY AES_KEY5 Register */
    __IO uint32_t aes_key6;             /**< <tt>\b 0x18:</tt> AES_KEY AES_KEY6 Register */
    __IO uint32_t aes_key7;             /**< <tt>\b 0x1C:</tt> AES_KEY AES_KEY7 Register */
} mxc_aes_key_regs_t;

/* Register offsets for module AES_KEY */
/**
 * @ingroup    aes_key_registers
 * @defgroup   AES_KEY_Register_Offsets Register Offsets
 * @brief      AES_KEY Peripheral Register Offsets from the AES_KEY Base Peripheral Address.
 * @{
 */
#define MXC_R_AES_KEY_AES_KEY0             ((uint32_t)0x00000000UL) /**< Offset from AES_KEY Base Address: <tt> 0x0000</tt> */
#define MXC_R_AES_KEY_AES_KEY1             ((uint32_t)0x00000004UL) /**< Offset from AES_KEY Base Address: <tt> 0x0004</tt> */
#define MXC_R_AES_KEY_AES_KEY2             ((uint32_t)0x00000008UL) /**< Offset from AES_KEY Base Address: <tt> 0x0008</tt> */
#define MXC_R_AES_KEY_AES_KEY3             ((uint32_t)0x0000000CUL) /**< Offset from AES_KEY Base Address: <tt> 0x000C</tt> */
#define MXC_R_AES_KEY_AES_KEY4             ((uint32_t)0x00000010UL) /**< Offset from AES_KEY Base Address: <tt> 0x0010</tt> */
#define MXC_R_AES_KEY_AES_KEY5             ((uint32_t)0x00000014UL) /**< Offset from AES_KEY Base Address: <tt> 0x0014</tt> */
#define MXC_R_AES_KEY_AES_KEY6             ((uint32_t)0x00000018UL) /**< Offset from AES_KEY Base Address: <tt> 0x0018</tt> */
#define MXC_R_AES_KEY_AES_KEY7             ((uint32_t)0x0000001CUL) /**< Offset from AES_KEY Base Address: <tt> 0x001C</tt> */
/**@} end of group aes_key_registers */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AES_KEY_REGS_H_
