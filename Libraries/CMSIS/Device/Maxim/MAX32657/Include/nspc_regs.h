/**
 * @file    nspc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the NSPC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup nspc_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_NSPC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_NSPC_REGS_H_

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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
 * @ingroup     nspc
 * @defgroup    nspc_registers NSPC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the NSPC Peripheral Module.
 * @details     Non-Secure Privilege Controller.
 */

/**
 * @ingroup nspc_registers
 * Structure type to access the NSPC Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x15f[88];
    __IO uint32_t apbpriv;              /**< <tt>\b 0x0160:</tt> NSPC APBPRIV Register */
    __R  uint32_t rsv_0x164_0x16f[3];
    __IO uint32_t ahbmpriv;             /**< <tt>\b 0x0170:</tt> NSPC AHBMPRIV Register */
} mxc_nspc_regs_t;

/* Register offsets for module NSPC */
/**
 * @ingroup    nspc_registers
 * @defgroup   NSPC_Register_Offsets Register Offsets
 * @brief      NSPC Peripheral Register Offsets from the NSPC Base Peripheral Address.
 * @{
 */
#define MXC_R_NSPC_APBPRIV                 ((uint32_t)0x00000160UL) /**< Offset from NSPC Base Address: <tt> 0x0160</tt> */
#define MXC_R_NSPC_AHBMPRIV                ((uint32_t)0x00000170UL) /**< Offset from NSPC Base Address: <tt> 0x0170</tt> */
/**@} end of group nspc_registers */

/**
 * @ingroup  nspc_registers
 * @defgroup NSPC_APBPRIV NSPC_APBPRIV
 * @brief    APB Tartet Privileged/Non-privileged PPC Access Register.
 * @{
 */
#define MXC_F_NSPC_APBPRIV_PERIPH_POS                  0 /**< APBPRIV_PERIPH Position */
#define MXC_F_NSPC_APBPRIV_PERIPH                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_NSPC_APBPRIV_PERIPH_POS)) /**< APBPRIV_PERIPH Mask */
#define MXC_V_NSPC_APBPRIV_PERIPH_GCR                  ((uint32_t)0x1UL) /**< APBPRIV_PERIPH_GCR Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_GCR                  (MXC_V_NSPC_APBPRIV_PERIPH_GCR << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_GCR Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_SIR                  ((uint32_t)0x2UL) /**< APBPRIV_PERIPH_SIR Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_SIR                  (MXC_V_NSPC_APBPRIV_PERIPH_SIR << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_SIR Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_FCR                  ((uint32_t)0x4UL) /**< APBPRIV_PERIPH_FCR Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_FCR                  (MXC_V_NSPC_APBPRIV_PERIPH_FCR << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_FCR Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_WDT                  ((uint32_t)0x8UL) /**< APBPRIV_PERIPH_WDT Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_WDT                  (MXC_V_NSPC_APBPRIV_PERIPH_WDT << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_WDT Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_AES                  ((uint32_t)0x10UL) /**< APBPRIV_PERIPH_AES Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_AES                  (MXC_V_NSPC_APBPRIV_PERIPH_AES << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_AES Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_AESKEYS              ((uint32_t)0x20UL) /**< APBPRIV_PERIPH_AESKEYS Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_AESKEYS              (MXC_V_NSPC_APBPRIV_PERIPH_AESKEYS << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_AESKEYS Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_CRC                  ((uint32_t)0x40UL) /**< APBPRIV_PERIPH_CRC Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_CRC                  (MXC_V_NSPC_APBPRIV_PERIPH_CRC << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_CRC Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_GPIO0                ((uint32_t)0x80UL) /**< APBPRIV_PERIPH_GPIO0 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_GPIO0                (MXC_V_NSPC_APBPRIV_PERIPH_GPIO0 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_GPIO0 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TMR0                 ((uint32_t)0x100UL) /**< APBPRIV_PERIPH_TMR0 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TMR0                 (MXC_V_NSPC_APBPRIV_PERIPH_TMR0 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR0 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TMR1                 ((uint32_t)0x200UL) /**< APBPRIV_PERIPH_TMR1 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TMR1                 (MXC_V_NSPC_APBPRIV_PERIPH_TMR1 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR1 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TMR2                 ((uint32_t)0x400UL) /**< APBPRIV_PERIPH_TMR2 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TMR2                 (MXC_V_NSPC_APBPRIV_PERIPH_TMR2 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR2 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TMR3                 ((uint32_t)0x800UL) /**< APBPRIV_PERIPH_TMR3 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TMR3                 (MXC_V_NSPC_APBPRIV_PERIPH_TMR3 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR3 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TMR4                 ((uint32_t)0x1000UL) /**< APBPRIV_PERIPH_TMR4 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TMR4                 (MXC_V_NSPC_APBPRIV_PERIPH_TMR4 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR4 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TMR5                 ((uint32_t)0x2000UL) /**< APBPRIV_PERIPH_TMR5 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TMR5                 (MXC_V_NSPC_APBPRIV_PERIPH_TMR5 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR5 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_I3C                  ((uint32_t)0x4000UL) /**< APBPRIV_PERIPH_I3C Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_I3C                  (MXC_V_NSPC_APBPRIV_PERIPH_I3C << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_I3C Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_UART                 ((uint32_t)0x8000UL) /**< APBPRIV_PERIPH_UART Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_UART                 (MXC_V_NSPC_APBPRIV_PERIPH_UART << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_UART Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_SPI                  ((uint32_t)0x10000UL) /**< APBPRIV_PERIPH_SPI Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_SPI                  (MXC_V_NSPC_APBPRIV_PERIPH_SPI << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_SPI Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TRNG                 ((uint32_t)0x20000UL) /**< APBPRIV_PERIPH_TRNG Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TRNG                 (MXC_V_NSPC_APBPRIV_PERIPH_TRNG << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TRNG Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_BTLE_DBB             ((uint32_t)0x40000UL) /**< APBPRIV_PERIPH_BTLE_DBB Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_BTLE_DBB             (MXC_V_NSPC_APBPRIV_PERIPH_BTLE_DBB << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_BTLE_DBB Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_BTLE_RFFE            ((uint32_t)0x80000UL) /**< APBPRIV_PERIPH_BTLE_RFFE Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_BTLE_RFFE            (MXC_V_NSPC_APBPRIV_PERIPH_BTLE_RFFE << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_BTLE_RFFE Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_RSTZ                 ((uint32_t)0x100000UL) /**< APBPRIV_PERIPH_RSTZ Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_RSTZ                 (MXC_V_NSPC_APBPRIV_PERIPH_RSTZ << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_RSTZ Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_BOOST                ((uint32_t)0x200000UL) /**< APBPRIV_PERIPH_BOOST Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_BOOST                (MXC_V_NSPC_APBPRIV_PERIPH_BOOST << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_BOOST Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_TRIMSIR              ((uint32_t)0x400000UL) /**< APBPRIV_PERIPH_TRIMSIR Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_TRIMSIR              (MXC_V_NSPC_APBPRIV_PERIPH_TRIMSIR << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TRIMSIR Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_RTC                  ((uint32_t)0x1000000UL) /**< APBPRIV_PERIPH_RTC Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_RTC                  (MXC_V_NSPC_APBPRIV_PERIPH_RTC << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_RTC Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_WUT0                 ((uint32_t)0x2000000UL) /**< APBPRIV_PERIPH_WUT0 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_WUT0                 (MXC_V_NSPC_APBPRIV_PERIPH_WUT0 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_WUT0 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_WUT1                 ((uint32_t)0x4000000UL) /**< APBPRIV_PERIPH_WUT1 Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_WUT1                 (MXC_V_NSPC_APBPRIV_PERIPH_WUT1 << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_WUT1 Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_PWRSEQ               ((uint32_t)0x8000000UL) /**< APBPRIV_PERIPH_PWRSEQ Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_PWRSEQ               (MXC_V_NSPC_APBPRIV_PERIPH_PWRSEQ << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_PWRSEQ Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_MCR                  ((uint32_t)0x10000000UL) /**< APBPRIV_PERIPH_MCR Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_MCR                  (MXC_V_NSPC_APBPRIV_PERIPH_MCR << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_MCR Setting */
#define MXC_V_NSPC_APBPRIV_PERIPH_ALL                  ((uint32_t)0x1F7FFFFFUL) /**< APBPRIV_PERIPH_ALL Value */
#define MXC_S_NSPC_APBPRIV_PERIPH_ALL                  (MXC_V_NSPC_APBPRIV_PERIPH_ALL << MXC_F_NSPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_ALL Setting */

/**@} end of group NSPC_APBPRIV_Register */

/**
 * @ingroup  nspc_registers
 * @defgroup NSPC_AHBMPRIV NSPC_AHBMPRIV
 * @brief    AHB Privileged/Non-Privileged Non-Secure DMA Access Register.
 * @{
 */
#define MXC_F_NSPC_AHBMPRIV_DMA_POS                    1 /**< AHBMPRIV_DMA Position */
#define MXC_F_NSPC_AHBMPRIV_DMA                        ((uint32_t)(0x1UL << MXC_F_NSPC_AHBMPRIV_DMA_POS)) /**< AHBMPRIV_DMA Mask */

/**@} end of group NSPC_AHBMPRIV_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_NSPC_REGS_H_
