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
    __R  uint32_t rsv_0x164_0x17f[7];
    __IO uint32_t gpio0;                /**< <tt>\b 0x0180:</tt> SPC GPIO0 Register */
    __IO uint32_t gpio1;                /**< <tt>\b 0x0184:</tt> SPC GPIO1 Register */
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
#define MXC_R_SPC_GPIO0                    ((uint32_t)0x00000180UL) /**< Offset from SPC Base Address: <tt> 0x0180</tt> */
#define MXC_R_SPC_GPIO1                    ((uint32_t)0x00000184UL) /**< Offset from SPC Base Address: <tt> 0x0184</tt> */
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
#define MXC_F_SPC_PPC_STATUS_APBPPC                    ((uint32_t)(0x3UL << MXC_F_SPC_PPC_STATUS_APBPPC_POS)) /**< PPC_STATUS_APBPPC Mask */

/**@} end of group SPC_PPC_STATUS_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_PPC_INTCLR SPC_PPC_INTCLR
 * @brief    Secure PPC Interrupt Clear Register.
 * @{
 */
#define MXC_F_SPC_PPC_INTCLR_APBPPC_POS                0 /**< PPC_INTCLR_APBPPC Position */
#define MXC_F_SPC_PPC_INTCLR_APBPPC                    ((uint32_t)(0x3UL << MXC_F_SPC_PPC_INTCLR_APBPPC_POS)) /**< PPC_INTCLR_APBPPC Mask */

/**@} end of group SPC_PPC_INTCLR_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_PPC_INTEN SPC_PPC_INTEN
 * @brief    Secure PPC Interrupt Enable Register.
 * @{
 */
#define MXC_F_SPC_PPC_INTEN_APBPPC_POS                 0 /**< PPC_INTEN_APBPPC Position */
#define MXC_F_SPC_PPC_INTEN_APBPPC                     ((uint32_t)(0x3UL << MXC_F_SPC_PPC_INTEN_APBPPC_POS)) /**< PPC_INTEN_APBPPC Mask */

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

/**@} end of group SPC_APBSEC_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_APBPRIV SPC_APBPRIV
 * @brief    APB Tartet Privileged/Non-privileged PPC Access Register.
 * @{
 */
#define MXC_F_SPC_APBPRIV_PERIPH_POS                   0 /**< APBPRIV_PERIPH Position */
#define MXC_F_SPC_APBPRIV_PERIPH                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPC_APBPRIV_PERIPH_POS)) /**< APBPRIV_PERIPH Mask */

/**@} end of group SPC_APBPRIV_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_GPIO0 SPC_GPIO0
 * @brief    Secure GPIO0 Configuration Register.
 * @{
 */
#define MXC_F_SPC_GPIO0_PINS_POS                       0 /**< GPIO0_PINS Position */
#define MXC_F_SPC_GPIO0_PINS                           ((uint32_t)(0xFFFUL << MXC_F_SPC_GPIO0_PINS_POS)) /**< GPIO0_PINS Mask */

/**@} end of group SPC_GPIO0_Register */

/**
 * @ingroup  spc_registers
 * @defgroup SPC_GPIO1 SPC_GPIO1
 * @brief    Secure GPIO1 Configuration Register.
 * @{
 */
#define MXC_F_SPC_GPIO1_PINS_POS                       0 /**< GPIO1_PINS Position */
#define MXC_F_SPC_GPIO1_PINS                           ((uint32_t)(0x3UL << MXC_F_SPC_GPIO1_PINS_POS)) /**< GPIO1_PINS Mask */

/**@} end of group SPC_GPIO1_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPC_REGS_H_
