/**
 * @file    usr_aeskeys_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the USR_AESKEYS Peripheral Module.
 * @note    This file is @generated.
 * @ingroup usr_aeskeys_registers
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
 * @ingroup     aes
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
