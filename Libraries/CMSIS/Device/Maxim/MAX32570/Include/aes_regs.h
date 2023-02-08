/**
 * @file    aes_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AES Peripheral Module.
 * @note    This file is @generated.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_AES_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_AES_REGS_H_

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
 * @ingroup     aes
 * @defgroup    aes_registers AES_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AES Peripheral Module.
 * @details     AES Keys.
 */

/**
 * @ingroup aes_registers
 * Structure type to access the AES Registers.
 */
typedef struct {
    __IO uint32_t aes_sram_key;         /**< <tt>\b 0x000:</tt> AES AES_SRAM_KEY Register */
    __R  uint32_t rsv_0x4_0x7f[31];
    __IO uint32_t aes_code_key;         /**< <tt>\b 0x080:</tt> AES AES_CODE_KEY Register */
    __R  uint32_t rsv_0x84_0xff[31];
    __IO uint32_t aes_data_key;         /**< <tt>\b 0x100:</tt> AES AES_DATA_KEY Register */
} mxc_aes_regs_t;

/* Register offsets for module AES */
/**
 * @ingroup    aes_registers
 * @defgroup   AES_Register_Offsets Register Offsets
 * @brief      AES Peripheral Register Offsets from the AES Base Peripheral Address.
 * @{
 */
#define MXC_R_AES_AES_SRAM_KEY             ((uint32_t)0x00000000UL) /**< Offset from AES Base Address: <tt> 0x0000</tt> */
#define MXC_R_AES_AES_CODE_KEY             ((uint32_t)0x00000080UL) /**< Offset from AES Base Address: <tt> 0x0080</tt> */
#define MXC_R_AES_AES_DATA_KEY             ((uint32_t)0x00000100UL) /**< Offset from AES Base Address: <tt> 0x0100</tt> */
/**@} end of group aes_registers */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_AES_REGS_H_
