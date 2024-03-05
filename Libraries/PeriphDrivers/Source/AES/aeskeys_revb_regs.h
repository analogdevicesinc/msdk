/**
 * @file    aeskeys_revb_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AESKEYS_REVB Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_AESKEYS_AESKEYS_REVB_REGS_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_AESKEYS_AESKEYS_REVB_REGS_H_

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
 * @ingroup     aeskeys_revb
 * @defgroup    aeskeys_revb_registers AESKEYS_REVB_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AESKEYS_REVB Peripheral Module.
 * @details     AES Key Registers.
 */

/**
 * @ingroup aeskeys_revb_registers
 * Structure type to access the AESKEYS_REVB Registers.
 */
typedef struct {
    __IO uint32_t key0;                 /**< <tt>\b 0x00:</tt> AESKEYS_REVB KEY0 Register */
    __IO uint32_t key1;                 /**< <tt>\b 0x04:</tt> AESKEYS_REVB KEY1 Register */
    __IO uint32_t key2;                 /**< <tt>\b 0x08:</tt> AESKEYS_REVB KEY2 Register */
    __IO uint32_t key3;                 /**< <tt>\b 0x0C:</tt> AESKEYS_REVB KEY3 Register */
    __IO uint32_t key4;                 /**< <tt>\b 0x10:</tt> AESKEYS_REVB KEY4 Register */
    __IO uint32_t key5;                 /**< <tt>\b 0x14:</tt> AESKEYS_REVB KEY5 Register */
    __IO uint32_t key6;                 /**< <tt>\b 0x18:</tt> AESKEYS_REVB KEY6 Register */
    __IO uint32_t key7;                 /**< <tt>\b 0x1C:</tt> AESKEYS_REVB KEY7 Register */
} mxc_aeskeys_revb_regs_t;

/* Register offsets for module AESKEYS_REVB */
/**
 * @ingroup    aeskeys_revb_registers
 * @defgroup   AESKEYS_REVB_Register_Offsets Register Offsets
 * @brief      AESKEYS_REVB Peripheral Register Offsets from the AESKEYS_REVB Base Peripheral Address.
 * @{
 */
#define MXC_R_AESKEYS_REVB_KEY0            ((uint32_t)0x00000000UL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x0000</tt> */
#define MXC_R_AESKEYS_REVB_KEY1            ((uint32_t)0x00000004UL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x0004</tt> */
#define MXC_R_AESKEYS_REVB_KEY2            ((uint32_t)0x00000008UL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x0008</tt> */
#define MXC_R_AESKEYS_REVB_KEY3            ((uint32_t)0x0000000CUL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x000C</tt> */
#define MXC_R_AESKEYS_REVB_KEY4            ((uint32_t)0x00000010UL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x0010</tt> */
#define MXC_R_AESKEYS_REVB_KEY5            ((uint32_t)0x00000014UL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x0014</tt> */
#define MXC_R_AESKEYS_REVB_KEY6            ((uint32_t)0x00000018UL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x0018</tt> */
#define MXC_R_AESKEYS_REVB_KEY7            ((uint32_t)0x0000001CUL) /**< Offset from AESKEYS_REVB Base Address: <tt> 0x001C</tt> */
/**@} end of group aeskeys_revb_registers */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_AESKEYS_AESKEYS_REVB_REGS_H_
