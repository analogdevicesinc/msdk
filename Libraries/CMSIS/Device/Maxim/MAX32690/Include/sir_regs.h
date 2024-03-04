/**
 * @file    sir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sir_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_SIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_SIR_REGS_H_

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
    __R  uint32_t rsv_0x8_0x27[8];
    __IO uint32_t btle_ldo_trim;        /**< <tt>\b 0x28:</tt> SIR BTLE_LDO_TRIM Register */
    __R  uint32_t rsv_0x2c_0xff[53];
    __I  uint32_t fstat;                /**< <tt>\b 0x100:</tt> SIR FSTAT Register */
    __I  uint32_t sfstat;               /**< <tt>\b 0x104:</tt> SIR SFSTAT Register */
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
#define MXC_R_SIR_BTLE_LDO_TRIM            ((uint32_t)0x00000028UL) /**< Offset from SIR Base Address: <tt> 0x0028</tt> */
#define MXC_R_SIR_FSTAT                    ((uint32_t)0x00000100UL) /**< Offset from SIR Base Address: <tt> 0x0100</tt> */
#define MXC_R_SIR_SFSTAT                   ((uint32_t)0x00000104UL) /**< Offset from SIR Base Address: <tt> 0x0104</tt> */
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

/**@} end of group SIR_SISTAT_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIADDR SIR_SIADDR
 * @brief    Read-only field set by the SIB block if a CRC error occurs during the read of
 *           the OTP memory. Contains the failing address in OTP memory (when CRCERR equals
 *           1).
 * @{
 */
#define MXC_F_SIR_SIADDR_ERRADDR_POS                   0 /**< SIADDR_ERRADDR Position */
#define MXC_F_SIR_SIADDR_ERRADDR                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIADDR_ERRADDR_POS)) /**< SIADDR_ERRADDR Mask */

/**@} end of group SIR_SIADDR_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_BTLE_LDO_TRIM SIR_BTLE_LDO_TRIM
 * @brief    BTLE LDO Trim register.
 * @{
 */
#define MXC_F_SIR_BTLE_LDO_TRIM_TX_POS                 0 /**< BTLE_LDO_TRIM_TX Position */
#define MXC_F_SIR_BTLE_LDO_TRIM_TX                     ((uint32_t)(0x1FUL << MXC_F_SIR_BTLE_LDO_TRIM_TX_POS)) /**< BTLE_LDO_TRIM_TX Mask */

#define MXC_F_SIR_BTLE_LDO_TRIM_RX_POS                 5 /**< BTLE_LDO_TRIM_RX Position */
#define MXC_F_SIR_BTLE_LDO_TRIM_RX                     ((uint32_t)(0x1FUL << MXC_F_SIR_BTLE_LDO_TRIM_RX_POS)) /**< BTLE_LDO_TRIM_RX Mask */

/**@} end of group SIR_BTLE_LDO_TRIM_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_FSTAT SIR_FSTAT
 * @brief    funcstat register.
 * @{
 */
#define MXC_F_SIR_FSTAT_FPU_POS                        0 /**< FSTAT_FPU Position */
#define MXC_F_SIR_FSTAT_FPU                            ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_FPU_POS)) /**< FSTAT_FPU Mask */

#define MXC_F_SIR_FSTAT_USB_POS                        1 /**< FSTAT_USB Position */
#define MXC_F_SIR_FSTAT_USB                            ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_USB_POS)) /**< FSTAT_USB Mask */

#define MXC_F_SIR_FSTAT_ADC_POS                        2 /**< FSTAT_ADC Position */
#define MXC_F_SIR_FSTAT_ADC                            ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_ADC_POS)) /**< FSTAT_ADC Mask */

#define MXC_F_SIR_FSTAT_SPIXIP_POS                     3 /**< FSTAT_SPIXIP Position */
#define MXC_F_SIR_FSTAT_SPIXIP                         ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_SPIXIP_POS)) /**< FSTAT_SPIXIP Mask */

#define MXC_F_SIR_FSTAT_HBC_POS                        4 /**< FSTAT_HBC Position */
#define MXC_F_SIR_FSTAT_HBC                            ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_HBC_POS)) /**< FSTAT_HBC Mask */

#define MXC_F_SIR_FSTAT_SMPHR_POS                      7 /**< FSTAT_SMPHR Position */
#define MXC_F_SIR_FSTAT_SMPHR                          ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_SMPHR_POS)) /**< FSTAT_SMPHR Mask */

#define MXC_F_SIR_FSTAT_BTLE_POS                       9 /**< FSTAT_BTLE Position */
#define MXC_F_SIR_FSTAT_BTLE                           ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_BTLE_POS)) /**< FSTAT_BTLE Mask */

/**@} end of group SIR_FSTAT_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SFSTAT SIR_SFSTAT
 * @brief    Security function status register.
 * @{
 */
#define MXC_F_SIR_SFSTAT_TRNG_POS                      2 /**< SFSTAT_TRNG Position */
#define MXC_F_SIR_SFSTAT_TRNG                          ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_TRNG_POS)) /**< SFSTAT_TRNG Mask */

#define MXC_F_SIR_SFSTAT_AES_POS                       3 /**< SFSTAT_AES Position */
#define MXC_F_SIR_SFSTAT_AES                           ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_AES_POS)) /**< SFSTAT_AES Mask */

#define MXC_F_SIR_SFSTAT_SHA_POS                       4 /**< SFSTAT_SHA Position */
#define MXC_F_SIR_SFSTAT_SHA                           ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_SHA_POS)) /**< SFSTAT_SHA Mask */

#define MXC_F_SIR_SFSTAT_MAA_POS                       5 /**< SFSTAT_MAA Position */
#define MXC_F_SIR_SFSTAT_MAA                           ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_MAA_POS)) /**< SFSTAT_MAA Mask */

/**@} end of group SIR_SFSTAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_SIR_REGS_H_
