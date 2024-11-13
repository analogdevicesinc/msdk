/**
 * @file    spc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPC_REGS_H_

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
 * @ingroup     spc
 * @defgroup    spc_registers SPC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPC Peripheral Module.
 * @details     Secure Privilege Controller.
 */

/**
 * @ingroup spc_registers
 * Structure type to access the SPC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> SPC CTRL Register */
    __R  uint32_t rsv_0x4;
    __IO uint32_t resp;                 /**< <tt>\b 0x0008:</tt> SPC RESP Register */
    __R  uint32_t rsv_0xc_0x1f[5];
    __I  uint8_t  mpc_status;           /**< <tt>\b 0x0020:</tt> SPC MPC_STATUS Register */
    __R  uint8_t  rsv_0x21_0x23[3];
    __IO uint32_t mpc_inten;            /**< <tt>\b 0x0024:</tt> SPC MPC_INTEN Register */
    __R  uint32_t rsv_0x28_0x2f[2];
    __I  uint32_t ppc_status;           /**< <tt>\b 0x0030:</tt> SPC PPC_STATUS Register */
    __O  uint32_t ppc_intclr;           /**< <tt>\b 0x0034:</tt> SPC PPC_INTCLR Register */
    __IO uint32_t ppc_inten;            /**< <tt>\b 0x0038:</tt> SPC PPC_INTEN Register */
    __R  uint32_t rsv_0x3c_0x7f[17];
    __IO uint32_t nscidau;              /**< <tt>\b 0x0080:</tt> SPC NSCIDAU Register */
    __R  uint32_t rsv_0x84_0x8f[3];
    __IO uint32_t m33lock;              /**< <tt>\b 0x0090:</tt> SPC M33LOCK Register */
    __R  uint32_t rsv_0x94_0x11f[35];
    __IO uint32_t apbsec;               /**< <tt>\b 0x0120:</tt> SPC APBSEC Register */
    __R  uint32_t rsv_0x124_0x15f[15];
    __IO uint32_t apbpriv;              /**< <tt>\b 0x0160:</tt> SPC APBPRIV Register */
    __R  uint32_t rsv_0x164_0x16f[3];
    __IO uint32_t ahbmpriv;             /**< <tt>\b 0x0170:</tt> SPC AHBMPRIV Register */
    __R  uint32_t rsv_0x174_0x17f[3];
    __IO uint32_t gpio0;                /**< <tt>\b 0x0180:</tt> SPC GPIO0 Register */
} mxc_spc_regs_t;

/* Register offsets for module SPC */
/**
 * @ingroup    spc_registers
 * @defgroup   SPC_Register_Offsets Register Offsets
 * @brief      SPC Peripheral Register Offsets from the SPC Base Peripheral Address.
 * @{
 */
#define MXC_R_SPC_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from SPC Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPC_RESP                     ((uint32_t)0x00000008UL) /**< Offset from SPC Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPC_MPC_STATUS               ((uint32_t)0x00000020UL) /**< Offset from SPC Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPC_MPC_INTEN                ((uint32_t)0x00000024UL) /**< Offset from SPC Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPC_PPC_STATUS               ((uint32_t)0x00000030UL) /**< Offset from SPC Base Address: <tt> 0x0030</tt> */
#define MXC_R_SPC_PPC_INTCLR               ((uint32_t)0x00000034UL) /**< Offset from SPC Base Address: <tt> 0x0034</tt> */
#define MXC_R_SPC_PPC_INTEN                ((uint32_t)0x00000038UL) /**< Offset from SPC Base Address: <tt> 0x0038</tt> */
#define MXC_R_SPC_NSCIDAU                  ((uint32_t)0x00000080UL) /**< Offset from SPC Base Address: <tt> 0x0080</tt> */
#define MXC_R_SPC_M33LOCK                  ((uint32_t)0x00000090UL) /**< Offset from SPC Base Address: <tt> 0x0090</tt> */
#define MXC_R_SPC_APBSEC                   ((uint32_t)0x00000120UL) /**< Offset from SPC Base Address: <tt> 0x0120</tt> */
#define MXC_R_SPC_APBPRIV                  ((uint32_t)0x00000160UL) /**< Offset from SPC Base Address: <tt> 0x0160</tt> */
#define MXC_R_SPC_AHBMPRIV                 ((uint32_t)0x00000170UL) /**< Offset from SPC Base Address: <tt> 0x0170</tt> */
#define MXC_R_SPC_GPIO0                    ((uint32_t)0x00000180UL) /**< Offset from SPC Base Address: <tt> 0x0180</tt> */
/**@} end of group spc_registers */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_CTRL SPC_CTRL
 * @brief    SPC Secure Configuration Control Register.
 * @{
 */
#define MXC_F_SPC_CTRL_LOCK_POS                        0 /**< CTRL_LOCK Position */
#define MXC_F_SPC_CTRL_LOCK                            ((uint32_t)(0x1UL << MXC_F_SPC_CTRL_LOCK_POS)) /**< CTRL_LOCK Mask */

/**@} end of group SPC_CTRL_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_RESP SPC_RESP
 * @brief    Security Violation Response Configuration Register.
 * @{
 */
#define MXC_F_SPC_RESP_VIOLCFG_POS                     0 /**< RESP_VIOLCFG Position */
#define MXC_F_SPC_RESP_VIOLCFG                         ((uint32_t)(0x1UL << MXC_F_SPC_RESP_VIOLCFG_POS)) /**< RESP_VIOLCFG Mask */

/**@} end of group SPC_RESP_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_MPC_STATUS SPC_MPC_STATUS
 * @brief    Secure MPC Status Register.
 * @{
 */
#define MXC_F_SPC_MPC_STATUS_SRAM0_POS                 0 /**< MPC_STATUS_SRAM0 Position */
#define MXC_F_SPC_MPC_STATUS_SRAM0                     ((uint8_t)(0x1UL << MXC_F_SPC_MPC_STATUS_SRAM0_POS)) /**< MPC_STATUS_SRAM0 Mask */

#define MXC_F_SPC_MPC_STATUS_SRAM1_POS                 1 /**< MPC_STATUS_SRAM1 Position */
#define MXC_F_SPC_MPC_STATUS_SRAM1                     ((uint8_t)(0x1UL << MXC_F_SPC_MPC_STATUS_SRAM1_POS)) /**< MPC_STATUS_SRAM1 Mask */

#define MXC_F_SPC_MPC_STATUS_SRAM2_POS                 2 /**< MPC_STATUS_SRAM2 Position */
#define MXC_F_SPC_MPC_STATUS_SRAM2                     ((uint8_t)(0x1UL << MXC_F_SPC_MPC_STATUS_SRAM2_POS)) /**< MPC_STATUS_SRAM2 Mask */

#define MXC_F_SPC_MPC_STATUS_SRAM3_POS                 3 /**< MPC_STATUS_SRAM3 Position */
#define MXC_F_SPC_MPC_STATUS_SRAM3                     ((uint8_t)(0x1UL << MXC_F_SPC_MPC_STATUS_SRAM3_POS)) /**< MPC_STATUS_SRAM3 Mask */

#define MXC_F_SPC_MPC_STATUS_SRAM4_POS                 4 /**< MPC_STATUS_SRAM4 Position */
#define MXC_F_SPC_MPC_STATUS_SRAM4                     ((uint8_t)(0x1UL << MXC_F_SPC_MPC_STATUS_SRAM4_POS)) /**< MPC_STATUS_SRAM4 Mask */

#define MXC_F_SPC_MPC_STATUS_FLASH_POS                 5 /**< MPC_STATUS_FLASH Position */
#define MXC_F_SPC_MPC_STATUS_FLASH                     ((uint8_t)(0x1UL << MXC_F_SPC_MPC_STATUS_FLASH_POS)) /**< MPC_STATUS_FLASH Mask */

/**@} end of group SPC_MPC_STATUS_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_MPC_INTEN SPC_MPC_INTEN
 * @brief    Secure MPC Interrupt Enable Register.
 * @{
 */
#define MXC_F_SPC_MPC_INTEN_SRAM0_POS                  0 /**< MPC_INTEN_SRAM0 Position */
#define MXC_F_SPC_MPC_INTEN_SRAM0                      ((uint32_t)(0x1UL << MXC_F_SPC_MPC_INTEN_SRAM0_POS)) /**< MPC_INTEN_SRAM0 Mask */

#define MXC_F_SPC_MPC_INTEN_SRAM1_POS                  1 /**< MPC_INTEN_SRAM1 Position */
#define MXC_F_SPC_MPC_INTEN_SRAM1                      ((uint32_t)(0x1UL << MXC_F_SPC_MPC_INTEN_SRAM1_POS)) /**< MPC_INTEN_SRAM1 Mask */

#define MXC_F_SPC_MPC_INTEN_SRAM2_POS                  2 /**< MPC_INTEN_SRAM2 Position */
#define MXC_F_SPC_MPC_INTEN_SRAM2                      ((uint32_t)(0x1UL << MXC_F_SPC_MPC_INTEN_SRAM2_POS)) /**< MPC_INTEN_SRAM2 Mask */

#define MXC_F_SPC_MPC_INTEN_SRAM3_POS                  3 /**< MPC_INTEN_SRAM3 Position */
#define MXC_F_SPC_MPC_INTEN_SRAM3                      ((uint32_t)(0x1UL << MXC_F_SPC_MPC_INTEN_SRAM3_POS)) /**< MPC_INTEN_SRAM3 Mask */

#define MXC_F_SPC_MPC_INTEN_SRAM4_POS                  4 /**< MPC_INTEN_SRAM4 Position */
#define MXC_F_SPC_MPC_INTEN_SRAM4                      ((uint32_t)(0x1UL << MXC_F_SPC_MPC_INTEN_SRAM4_POS)) /**< MPC_INTEN_SRAM4 Mask */

#define MXC_F_SPC_MPC_INTEN_FLASH_POS                  5 /**< MPC_INTEN_FLASH Position */
#define MXC_F_SPC_MPC_INTEN_FLASH                      ((uint32_t)(0x1UL << MXC_F_SPC_MPC_INTEN_FLASH_POS)) /**< MPC_INTEN_FLASH Mask */

/**@} end of group SPC_MPC_INTEN_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_PPC_STATUS SPC_PPC_STATUS
 * @brief    Secure PPC Interrupt Status Register.
 * @{
 */
#define MXC_F_SPC_PPC_STATUS_APBPPC_POS                0 /**< PPC_STATUS_APBPPC Position */
#define MXC_F_SPC_PPC_STATUS_APBPPC                    ((uint32_t)(0xFUL << MXC_F_SPC_PPC_STATUS_APBPPC_POS)) /**< PPC_STATUS_APBPPC Mask */

/**@} end of group SPC_PPC_STATUS_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_PPC_INTCLR SPC_PPC_INTCLR
 * @brief    Secure PPC Interrupt Clear Register.
 * @{
 */
#define MXC_F_SPC_PPC_INTCLR_APBPPC_POS                0 /**< PPC_INTCLR_APBPPC Position */
#define MXC_F_SPC_PPC_INTCLR_APBPPC                    ((uint32_t)(0xFUL << MXC_F_SPC_PPC_INTCLR_APBPPC_POS)) /**< PPC_INTCLR_APBPPC Mask */

/**@} end of group SPC_PPC_INTCLR_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_PPC_INTEN SPC_PPC_INTEN
 * @brief    Secure PPC Interrupt Enable Register.
 * @{
 */
#define MXC_F_SPC_PPC_INTEN_APBPPC_POS                 0 /**< PPC_INTEN_APBPPC Position */
#define MXC_F_SPC_PPC_INTEN_APBPPC                     ((uint32_t)(0xFUL << MXC_F_SPC_PPC_INTEN_APBPPC_POS)) /**< PPC_INTEN_APBPPC Mask */

/**@} end of group SPC_PPC_INTEN_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_NSCIDAU SPC_NSCIDAU
 * @brief    Non-Secure Callabale IDAU Configuration Register.
 * @{
 */
#define MXC_F_SPC_NSCIDAU_CODE_POS                     0 /**< NSCIDAU_CODE Position */
#define MXC_F_SPC_NSCIDAU_CODE                         ((uint32_t)(0x1UL << MXC_F_SPC_NSCIDAU_CODE_POS)) /**< NSCIDAU_CODE Mask */

#define MXC_F_SPC_NSCIDAU_SRAM_POS                     1 /**< NSCIDAU_SRAM Position */
#define MXC_F_SPC_NSCIDAU_SRAM                         ((uint32_t)(0x1UL << MXC_F_SPC_NSCIDAU_SRAM_POS)) /**< NSCIDAU_SRAM Mask */

/**@} end of group SPC_NSCIDAU_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_M33LOCK SPC_M33LOCK
 * @brief    M33 Core Register Lock Configuratrion Register.
 * @{
 */
#define MXC_F_SPC_M33LOCK_AIRCR_VTOR_S_POS             0 /**< M33LOCK_AIRCR_VTOR_S Position */
#define MXC_F_SPC_M33LOCK_AIRCR_VTOR_S                 ((uint32_t)(0x1UL << MXC_F_SPC_M33LOCK_AIRCR_VTOR_S_POS)) /**< M33LOCK_AIRCR_VTOR_S Mask */

#define MXC_F_SPC_M33LOCK_VTOR_NS_POS                  1 /**< M33LOCK_VTOR_NS Position */
#define MXC_F_SPC_M33LOCK_VTOR_NS                      ((uint32_t)(0x1UL << MXC_F_SPC_M33LOCK_VTOR_NS_POS)) /**< M33LOCK_VTOR_NS Mask */

#define MXC_F_SPC_M33LOCK_MPU_S_POS                    2 /**< M33LOCK_MPU_S Position */
#define MXC_F_SPC_M33LOCK_MPU_S                        ((uint32_t)(0x1UL << MXC_F_SPC_M33LOCK_MPU_S_POS)) /**< M33LOCK_MPU_S Mask */

#define MXC_F_SPC_M33LOCK_MPU_NS_POS                   3 /**< M33LOCK_MPU_NS Position */
#define MXC_F_SPC_M33LOCK_MPU_NS                       ((uint32_t)(0x1UL << MXC_F_SPC_M33LOCK_MPU_NS_POS)) /**< M33LOCK_MPU_NS Mask */

#define MXC_F_SPC_M33LOCK_SAU_POS                      4 /**< M33LOCK_SAU Position */
#define MXC_F_SPC_M33LOCK_SAU                          ((uint32_t)(0x1UL << MXC_F_SPC_M33LOCK_SAU_POS)) /**< M33LOCK_SAU Mask */

/**@} end of group SPC_M33LOCK_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_APBSEC SPC_APBSEC
 * @brief    APB Target Secure/Non-secure PPC Access Register.
 * @{
 */
#define MXC_F_SPC_APBSEC_PERIPH_POS                    0 /**< APBSEC_PERIPH Position */
#define MXC_F_SPC_APBSEC_PERIPH                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPC_APBSEC_PERIPH_POS)) /**< APBSEC_PERIPH Mask */
#define MXC_V_SPC_APBSEC_PERIPH_GCR                    ((uint32_t)0x1UL) /**< APBSEC_PERIPH_GCR Value */
#define MXC_S_SPC_APBSEC_PERIPH_GCR                    (MXC_V_SPC_APBSEC_PERIPH_GCR << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_GCR Setting */
#define MXC_V_SPC_APBSEC_PERIPH_SIR                    ((uint32_t)0x2UL) /**< APBSEC_PERIPH_SIR Value */
#define MXC_S_SPC_APBSEC_PERIPH_SIR                    (MXC_V_SPC_APBSEC_PERIPH_SIR << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_SIR Setting */
#define MXC_V_SPC_APBSEC_PERIPH_FCR                    ((uint32_t)0x4UL) /**< APBSEC_PERIPH_FCR Value */
#define MXC_S_SPC_APBSEC_PERIPH_FCR                    (MXC_V_SPC_APBSEC_PERIPH_FCR << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_FCR Setting */
#define MXC_V_SPC_APBSEC_PERIPH_WDT                    ((uint32_t)0x8UL) /**< APBSEC_PERIPH_WDT Value */
#define MXC_S_SPC_APBSEC_PERIPH_WDT                    (MXC_V_SPC_APBSEC_PERIPH_WDT << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_WDT Setting */
#define MXC_V_SPC_APBSEC_PERIPH_AES                    ((uint32_t)0x10UL) /**< APBSEC_PERIPH_AES Value */
#define MXC_S_SPC_APBSEC_PERIPH_AES                    (MXC_V_SPC_APBSEC_PERIPH_AES << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_AES Setting */
#define MXC_V_SPC_APBSEC_PERIPH_AESKEYS                ((uint32_t)0x20UL) /**< APBSEC_PERIPH_AESKEYS Value */
#define MXC_S_SPC_APBSEC_PERIPH_AESKEYS                (MXC_V_SPC_APBSEC_PERIPH_AESKEYS << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_AESKEYS Setting */
#define MXC_V_SPC_APBSEC_PERIPH_CRC                    ((uint32_t)0x40UL) /**< APBSEC_PERIPH_CRC Value */
#define MXC_S_SPC_APBSEC_PERIPH_CRC                    (MXC_V_SPC_APBSEC_PERIPH_CRC << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_CRC Setting */
#define MXC_V_SPC_APBSEC_PERIPH_GPIO0                  ((uint32_t)0x80UL) /**< APBSEC_PERIPH_GPIO0 Value */
#define MXC_S_SPC_APBSEC_PERIPH_GPIO0                  (MXC_V_SPC_APBSEC_PERIPH_GPIO0 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_GPIO0 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TMR0                   ((uint32_t)0x100UL) /**< APBSEC_PERIPH_TMR0 Value */
#define MXC_S_SPC_APBSEC_PERIPH_TMR0                   (MXC_V_SPC_APBSEC_PERIPH_TMR0 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TMR0 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TMR1                   ((uint32_t)0x200UL) /**< APBSEC_PERIPH_TMR1 Value */
#define MXC_S_SPC_APBSEC_PERIPH_TMR1                   (MXC_V_SPC_APBSEC_PERIPH_TMR1 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TMR1 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TMR2                   ((uint32_t)0x400UL) /**< APBSEC_PERIPH_TMR2 Value */
#define MXC_S_SPC_APBSEC_PERIPH_TMR2                   (MXC_V_SPC_APBSEC_PERIPH_TMR2 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TMR2 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TMR3                   ((uint32_t)0x800UL) /**< APBSEC_PERIPH_TMR3 Value */
#define MXC_S_SPC_APBSEC_PERIPH_TMR3                   (MXC_V_SPC_APBSEC_PERIPH_TMR3 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TMR3 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TMR4                   ((uint32_t)0x1000UL) /**< APBSEC_PERIPH_TMR4 Value */
#define MXC_S_SPC_APBSEC_PERIPH_TMR4                   (MXC_V_SPC_APBSEC_PERIPH_TMR4 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TMR4 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TMR5                   ((uint32_t)0x2000UL) /**< APBSEC_PERIPH_TMR5 Value */
#define MXC_S_SPC_APBSEC_PERIPH_TMR5                   (MXC_V_SPC_APBSEC_PERIPH_TMR5 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TMR5 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_I3C                    ((uint32_t)0x4000UL) /**< APBSEC_PERIPH_I3C Value */
#define MXC_S_SPC_APBSEC_PERIPH_I3C                    (MXC_V_SPC_APBSEC_PERIPH_I3C << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_I3C Setting */
#define MXC_V_SPC_APBSEC_PERIPH_UART                   ((uint32_t)0x8000UL) /**< APBSEC_PERIPH_UART Value */
#define MXC_S_SPC_APBSEC_PERIPH_UART                   (MXC_V_SPC_APBSEC_PERIPH_UART << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_UART Setting */
#define MXC_V_SPC_APBSEC_PERIPH_SPI                    ((uint32_t)0x10000UL) /**< APBSEC_PERIPH_SPI Value */
#define MXC_S_SPC_APBSEC_PERIPH_SPI                    (MXC_V_SPC_APBSEC_PERIPH_SPI << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_SPI Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TRNG                   ((uint32_t)0x20000UL) /**< APBSEC_PERIPH_TRNG Value */
#define MXC_S_SPC_APBSEC_PERIPH_TRNG                   (MXC_V_SPC_APBSEC_PERIPH_TRNG << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TRNG Setting */
#define MXC_V_SPC_APBSEC_PERIPH_BTLE_DBB               ((uint32_t)0x40000UL) /**< APBSEC_PERIPH_BTLE_DBB Value */
#define MXC_S_SPC_APBSEC_PERIPH_BTLE_DBB               (MXC_V_SPC_APBSEC_PERIPH_BTLE_DBB << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_BTLE_DBB Setting */
#define MXC_V_SPC_APBSEC_PERIPH_BTLE_RFFE              ((uint32_t)0x80000UL) /**< APBSEC_PERIPH_BTLE_RFFE Value */
#define MXC_S_SPC_APBSEC_PERIPH_BTLE_RFFE              (MXC_V_SPC_APBSEC_PERIPH_BTLE_RFFE << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_BTLE_RFFE Setting */
#define MXC_V_SPC_APBSEC_PERIPH_RSTZ                   ((uint32_t)0x100000UL) /**< APBSEC_PERIPH_RSTZ Value */
#define MXC_S_SPC_APBSEC_PERIPH_RSTZ                   (MXC_V_SPC_APBSEC_PERIPH_RSTZ << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_RSTZ Setting */
#define MXC_V_SPC_APBSEC_PERIPH_BOOST                  ((uint32_t)0x200000UL) /**< APBSEC_PERIPH_BOOST Value */
#define MXC_S_SPC_APBSEC_PERIPH_BOOST                  (MXC_V_SPC_APBSEC_PERIPH_BOOST << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_BOOST Setting */
#define MXC_V_SPC_APBSEC_PERIPH_TRIMSIR                ((uint32_t)0x400000UL) /**< APBSEC_PERIPH_TRIMSIR Value */
#define MXC_S_SPC_APBSEC_PERIPH_TRIMSIR                (MXC_V_SPC_APBSEC_PERIPH_TRIMSIR << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_TRIMSIR Setting */
#define MXC_V_SPC_APBSEC_PERIPH_RTC                    ((uint32_t)0x1000000UL) /**< APBSEC_PERIPH_RTC Value */
#define MXC_S_SPC_APBSEC_PERIPH_RTC                    (MXC_V_SPC_APBSEC_PERIPH_RTC << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_RTC Setting */
#define MXC_V_SPC_APBSEC_PERIPH_WUT0                   ((uint32_t)0x2000000UL) /**< APBSEC_PERIPH_WUT0 Value */
#define MXC_S_SPC_APBSEC_PERIPH_WUT0                   (MXC_V_SPC_APBSEC_PERIPH_WUT0 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_WUT0 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_WUT1                   ((uint32_t)0x4000000UL) /**< APBSEC_PERIPH_WUT1 Value */
#define MXC_S_SPC_APBSEC_PERIPH_WUT1                   (MXC_V_SPC_APBSEC_PERIPH_WUT1 << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_WUT1 Setting */
#define MXC_V_SPC_APBSEC_PERIPH_PWRSEQ                 ((uint32_t)0x8000000UL) /**< APBSEC_PERIPH_PWRSEQ Value */
#define MXC_S_SPC_APBSEC_PERIPH_PWRSEQ                 (MXC_V_SPC_APBSEC_PERIPH_PWRSEQ << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_PWRSEQ Setting */
#define MXC_V_SPC_APBSEC_PERIPH_MCR                    ((uint32_t)0x10000000UL) /**< APBSEC_PERIPH_MCR Value */
#define MXC_S_SPC_APBSEC_PERIPH_MCR                    (MXC_V_SPC_APBSEC_PERIPH_MCR << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_MCR Setting */
#define MXC_V_SPC_APBSEC_PERIPH_ALL                    ((uint32_t)0x1F7FFFFFUL) /**< APBSEC_PERIPH_ALL Value */
#define MXC_S_SPC_APBSEC_PERIPH_ALL                    (MXC_V_SPC_APBSEC_PERIPH_ALL << MXC_F_SPC_APBSEC_PERIPH_POS) /**< APBSEC_PERIPH_ALL Setting */

/**@} end of group SPC_APBSEC_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_APBPRIV SPC_APBPRIV
 * @brief    APB Tartet Privileged/Non-privileged PPC Access Register.
 * @{
 */
#define MXC_F_SPC_APBPRIV_PERIPH_POS                   0 /**< APBPRIV_PERIPH Position */
#define MXC_F_SPC_APBPRIV_PERIPH                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPC_APBPRIV_PERIPH_POS)) /**< APBPRIV_PERIPH Mask */
#define MXC_V_SPC_APBPRIV_PERIPH_GCR                   ((uint32_t)0x1UL) /**< APBPRIV_PERIPH_GCR Value */
#define MXC_S_SPC_APBPRIV_PERIPH_GCR                   (MXC_V_SPC_APBPRIV_PERIPH_GCR << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_GCR Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_SIR                   ((uint32_t)0x2UL) /**< APBPRIV_PERIPH_SIR Value */
#define MXC_S_SPC_APBPRIV_PERIPH_SIR                   (MXC_V_SPC_APBPRIV_PERIPH_SIR << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_SIR Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_FCR                   ((uint32_t)0x4UL) /**< APBPRIV_PERIPH_FCR Value */
#define MXC_S_SPC_APBPRIV_PERIPH_FCR                   (MXC_V_SPC_APBPRIV_PERIPH_FCR << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_FCR Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_WDT                   ((uint32_t)0x8UL) /**< APBPRIV_PERIPH_WDT Value */
#define MXC_S_SPC_APBPRIV_PERIPH_WDT                   (MXC_V_SPC_APBPRIV_PERIPH_WDT << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_WDT Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_AES                   ((uint32_t)0x10UL) /**< APBPRIV_PERIPH_AES Value */
#define MXC_S_SPC_APBPRIV_PERIPH_AES                   (MXC_V_SPC_APBPRIV_PERIPH_AES << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_AES Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_AESKEYS               ((uint32_t)0x20UL) /**< APBPRIV_PERIPH_AESKEYS Value */
#define MXC_S_SPC_APBPRIV_PERIPH_AESKEYS               (MXC_V_SPC_APBPRIV_PERIPH_AESKEYS << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_AESKEYS Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_CRC                   ((uint32_t)0x40UL) /**< APBPRIV_PERIPH_CRC Value */
#define MXC_S_SPC_APBPRIV_PERIPH_CRC                   (MXC_V_SPC_APBPRIV_PERIPH_CRC << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_CRC Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_GPIO0                 ((uint32_t)0x80UL) /**< APBPRIV_PERIPH_GPIO0 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_GPIO0                 (MXC_V_SPC_APBPRIV_PERIPH_GPIO0 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_GPIO0 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TMR0                  ((uint32_t)0x100UL) /**< APBPRIV_PERIPH_TMR0 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TMR0                  (MXC_V_SPC_APBPRIV_PERIPH_TMR0 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR0 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TMR1                  ((uint32_t)0x200UL) /**< APBPRIV_PERIPH_TMR1 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TMR1                  (MXC_V_SPC_APBPRIV_PERIPH_TMR1 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR1 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TMR2                  ((uint32_t)0x400UL) /**< APBPRIV_PERIPH_TMR2 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TMR2                  (MXC_V_SPC_APBPRIV_PERIPH_TMR2 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR2 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TMR3                  ((uint32_t)0x800UL) /**< APBPRIV_PERIPH_TMR3 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TMR3                  (MXC_V_SPC_APBPRIV_PERIPH_TMR3 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR3 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TMR4                  ((uint32_t)0x1000UL) /**< APBPRIV_PERIPH_TMR4 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TMR4                  (MXC_V_SPC_APBPRIV_PERIPH_TMR4 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR4 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TMR5                  ((uint32_t)0x2000UL) /**< APBPRIV_PERIPH_TMR5 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TMR5                  (MXC_V_SPC_APBPRIV_PERIPH_TMR5 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TMR5 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_I3C                   ((uint32_t)0x4000UL) /**< APBPRIV_PERIPH_I3C Value */
#define MXC_S_SPC_APBPRIV_PERIPH_I3C                   (MXC_V_SPC_APBPRIV_PERIPH_I3C << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_I3C Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_UART                  ((uint32_t)0x8000UL) /**< APBPRIV_PERIPH_UART Value */
#define MXC_S_SPC_APBPRIV_PERIPH_UART                  (MXC_V_SPC_APBPRIV_PERIPH_UART << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_UART Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_SPI                   ((uint32_t)0x10000UL) /**< APBPRIV_PERIPH_SPI Value */
#define MXC_S_SPC_APBPRIV_PERIPH_SPI                   (MXC_V_SPC_APBPRIV_PERIPH_SPI << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_SPI Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TRNG                  ((uint32_t)0x20000UL) /**< APBPRIV_PERIPH_TRNG Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TRNG                  (MXC_V_SPC_APBPRIV_PERIPH_TRNG << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TRNG Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_BTLE_DBB              ((uint32_t)0x40000UL) /**< APBPRIV_PERIPH_BTLE_DBB Value */
#define MXC_S_SPC_APBPRIV_PERIPH_BTLE_DBB              (MXC_V_SPC_APBPRIV_PERIPH_BTLE_DBB << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_BTLE_DBB Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_BTLE_RFFE             ((uint32_t)0x80000UL) /**< APBPRIV_PERIPH_BTLE_RFFE Value */
#define MXC_S_SPC_APBPRIV_PERIPH_BTLE_RFFE             (MXC_V_SPC_APBPRIV_PERIPH_BTLE_RFFE << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_BTLE_RFFE Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_RSTZ                  ((uint32_t)0x100000UL) /**< APBPRIV_PERIPH_RSTZ Value */
#define MXC_S_SPC_APBPRIV_PERIPH_RSTZ                  (MXC_V_SPC_APBPRIV_PERIPH_RSTZ << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_RSTZ Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_BOOST                 ((uint32_t)0x200000UL) /**< APBPRIV_PERIPH_BOOST Value */
#define MXC_S_SPC_APBPRIV_PERIPH_BOOST                 (MXC_V_SPC_APBPRIV_PERIPH_BOOST << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_BOOST Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_TRIMSIR               ((uint32_t)0x400000UL) /**< APBPRIV_PERIPH_TRIMSIR Value */
#define MXC_S_SPC_APBPRIV_PERIPH_TRIMSIR               (MXC_V_SPC_APBPRIV_PERIPH_TRIMSIR << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_TRIMSIR Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_RTC                   ((uint32_t)0x1000000UL) /**< APBPRIV_PERIPH_RTC Value */
#define MXC_S_SPC_APBPRIV_PERIPH_RTC                   (MXC_V_SPC_APBPRIV_PERIPH_RTC << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_RTC Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_WUT0                  ((uint32_t)0x2000000UL) /**< APBPRIV_PERIPH_WUT0 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_WUT0                  (MXC_V_SPC_APBPRIV_PERIPH_WUT0 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_WUT0 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_WUT1                  ((uint32_t)0x4000000UL) /**< APBPRIV_PERIPH_WUT1 Value */
#define MXC_S_SPC_APBPRIV_PERIPH_WUT1                  (MXC_V_SPC_APBPRIV_PERIPH_WUT1 << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_WUT1 Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_PWRSEQ                ((uint32_t)0x8000000UL) /**< APBPRIV_PERIPH_PWRSEQ Value */
#define MXC_S_SPC_APBPRIV_PERIPH_PWRSEQ                (MXC_V_SPC_APBPRIV_PERIPH_PWRSEQ << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_PWRSEQ Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_MCR                   ((uint32_t)0x10000000UL) /**< APBPRIV_PERIPH_MCR Value */
#define MXC_S_SPC_APBPRIV_PERIPH_MCR                   (MXC_V_SPC_APBPRIV_PERIPH_MCR << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_MCR Setting */
#define MXC_V_SPC_APBPRIV_PERIPH_ALL                   ((uint32_t)0x1F7FFFFFUL) /**< APBPRIV_PERIPH_ALL Value */
#define MXC_S_SPC_APBPRIV_PERIPH_ALL                   (MXC_V_SPC_APBPRIV_PERIPH_ALL << MXC_F_SPC_APBPRIV_PERIPH_POS) /**< APBPRIV_PERIPH_ALL Setting */

/**@} end of group SPC_APBPRIV_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_AHBMPRIV SPC_AHBMPRIV
 * @brief    AHB Privileged/Non-privileged Secure DMA Access.
 * @{
 */
#define MXC_F_SPC_AHBMPRIV_DMA_POS                     0 /**< AHBMPRIV_DMA Position */
#define MXC_F_SPC_AHBMPRIV_DMA                         ((uint32_t)(0x1UL << MXC_F_SPC_AHBMPRIV_DMA_POS)) /**< AHBMPRIV_DMA Mask */

/**@} end of group SPC_AHBMPRIV_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_GPIO0 SPC_GPIO0
 * @brief    Secure GPIO0 Configuration Register.
 * @{
 */
#define MXC_F_SPC_GPIO0_PINS_POS                       0 /**< GPIO0_PINS Position */
#define MXC_F_SPC_GPIO0_PINS                           ((uint32_t)(0x3FFFUL << MXC_F_SPC_GPIO0_PINS_POS)) /**< GPIO0_PINS Mask */

/**@} end of group SPC_GPIO0_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPC_REGS_H_
