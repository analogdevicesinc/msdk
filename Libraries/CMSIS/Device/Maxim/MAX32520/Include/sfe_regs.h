/**
 * @file    sfe_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SFE Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sfe_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_SFE_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_SFE_REGS_H_

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
 * @ingroup     sfe
 * @defgroup    sfe_registers SFE_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SFE Peripheral Module.
 * @details     Serial Flash Emulator.
 */

/**
 * @ingroup sfe_registers
 * Structure type to access the SFE Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x3ff[256];
    __IO uint32_t cfg;                  /**< <tt>\b 0x0400:</tt> SFE CFG Register */
    __R  uint32_t rsv_0x404;
    __IO uint32_t hfsa;                 /**< <tt>\b 0x0408:</tt> SFE HFSA Register */
    __IO uint32_t hrsa;                 /**< <tt>\b 0x040C:</tt> SFE HRSA Register */
    __IO uint32_t sfdp_sba;             /**< <tt>\b 0x0410:</tt> SFE SFDP_SBA Register */
    __IO uint32_t flash_sba;            /**< <tt>\b 0x0414:</tt> SFE FLASH_SBA Register */
    __IO uint32_t flash_sta;            /**< <tt>\b 0x0418:</tt> SFE FLASH_STA Register */
    __IO uint32_t ram_sba;              /**< <tt>\b 0x041C:</tt> SFE RAM_SBA Register */
    __IO uint32_t ram_sta;              /**< <tt>\b 0x0420:</tt> SFE RAM_STA Register */
} mxc_sfe_regs_t;

/* Register offsets for module SFE */
/**
 * @ingroup    sfe_registers
 * @defgroup   SFE_Register_Offsets Register Offsets
 * @brief      SFE Peripheral Register Offsets from the SFE Base Peripheral Address.
 * @{
 */
#define MXC_R_SFE_CFG                      ((uint32_t)0x00000400UL) /**< Offset from SFE Base Address: <tt> 0x0400</tt> */
#define MXC_R_SFE_HFSA                     ((uint32_t)0x00000408UL) /**< Offset from SFE Base Address: <tt> 0x0408</tt> */
#define MXC_R_SFE_HRSA                     ((uint32_t)0x0000040CUL) /**< Offset from SFE Base Address: <tt> 0x040C</tt> */
#define MXC_R_SFE_SFDP_SBA                 ((uint32_t)0x00000410UL) /**< Offset from SFE Base Address: <tt> 0x0410</tt> */
#define MXC_R_SFE_FLASH_SBA                ((uint32_t)0x00000414UL) /**< Offset from SFE Base Address: <tt> 0x0414</tt> */
#define MXC_R_SFE_FLASH_STA                ((uint32_t)0x00000418UL) /**< Offset from SFE Base Address: <tt> 0x0418</tt> */
#define MXC_R_SFE_RAM_SBA                  ((uint32_t)0x0000041CUL) /**< Offset from SFE Base Address: <tt> 0x041C</tt> */
#define MXC_R_SFE_RAM_STA                  ((uint32_t)0x00000420UL) /**< Offset from SFE Base Address: <tt> 0x0420</tt> */
/**@} end of group sfe_registers */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_CFG SFE_CFG
 * @brief    SFE Configuration Register.
 * @{
 */
#define MXC_F_SFE_CFG_DRLE_POS                         0 /**< CFG_DRLE Position */
#define MXC_F_SFE_CFG_DRLE                             ((uint32_t)(0x1UL << MXC_F_SFE_CFG_DRLE_POS)) /**< CFG_DRLE Mask */

#define MXC_F_SFE_CFG_FLOCK_POS                        15 /**< CFG_FLOCK Position */
#define MXC_F_SFE_CFG_FLOCK                            ((uint32_t)(0x1UL << MXC_F_SFE_CFG_FLOCK_POS)) /**< CFG_FLOCK Mask */

#define MXC_F_SFE_CFG_RD_EN_POS                        16 /**< CFG_RD_EN Position */
#define MXC_F_SFE_CFG_RD_EN                            ((uint32_t)(0x1UL << MXC_F_SFE_CFG_RD_EN_POS)) /**< CFG_RD_EN Mask */

#define MXC_F_SFE_CFG_WR_EN_POS                        17 /**< CFG_WR_EN Position */
#define MXC_F_SFE_CFG_WR_EN                            ((uint32_t)(0x1UL << MXC_F_SFE_CFG_WR_EN_POS)) /**< CFG_WR_EN Mask */

#define MXC_F_SFE_CFG_RRLOCK_POS                       22 /**< CFG_RRLOCK Position */
#define MXC_F_SFE_CFG_RRLOCK                           ((uint32_t)(0x1UL << MXC_F_SFE_CFG_RRLOCK_POS)) /**< CFG_RRLOCK Mask */

#define MXC_F_SFE_CFG_RWLOCK_POS                       23 /**< CFG_RWLOCK Position */
#define MXC_F_SFE_CFG_RWLOCK                           ((uint32_t)(0x1UL << MXC_F_SFE_CFG_RWLOCK_POS)) /**< CFG_RWLOCK Mask */

/**@} end of group SFE_CFG_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_HFSA SFE_HFSA
 * @brief    SFE Host Flash Start Address Register.
 * @{
 */
#define MXC_F_SFE_HFSA_HFSA_POS                        10 /**< HFSA_HFSA Position */
#define MXC_F_SFE_HFSA_HFSA                            ((uint32_t)(0x3FFFFFUL << MXC_F_SFE_HFSA_HFSA_POS)) /**< HFSA_HFSA Mask */

/**@} end of group SFE_HFSA_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_HRSA SFE_HRSA
 * @brief    SFE Host RAM Start Address Register.
 * @{
 */
#define MXC_F_SFE_HRSA_HRSA_POS                        10 /**< HRSA_HRSA Position */
#define MXC_F_SFE_HRSA_HRSA                            ((uint32_t)(0x3FFFFFUL << MXC_F_SFE_HRSA_HRSA_POS)) /**< HRSA_HRSA Mask */

/**@} end of group SFE_HRSA_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_SFDP_SBA SFE_SFDP_SBA
 * @brief    SFE Discoverable Parameter System Base Register.
 * @{
 */
#define MXC_F_SFE_SFDP_SBA_SFDP_SBA_POS                8 /**< SFDP_SBA_SFDP_SBA Position */
#define MXC_F_SFE_SFDP_SBA_SFDP_SBA                    ((uint32_t)(0xFFFFFFUL << MXC_F_SFE_SFDP_SBA_SFDP_SBA_POS)) /**< SFDP_SBA_SFDP_SBA Mask */

/**@} end of group SFE_SFDP_SBA_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_FLASH_SBA SFE_FLASH_SBA
 * @brief    Flash System Base Address Register.
 * @{
 */
#define MXC_F_SFE_FLASH_SBA_FLASH_SBA_POS              10 /**< FLASH_SBA_FLASH_SBA Position */
#define MXC_F_SFE_FLASH_SBA_FLASH_SBA                  ((uint32_t)(0x3FFFFFUL << MXC_F_SFE_FLASH_SBA_FLASH_SBA_POS)) /**< FLASH_SBA_FLASH_SBA Mask */

/**@} end of group SFE_FLASH_SBA_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_FLASH_STA SFE_FLASH_STA
 * @brief    Flash System Top Address Register.
 * @{
 */
#define MXC_F_SFE_FLASH_STA_FLASH_STA_POS              10 /**< FLASH_STA_FLASH_STA Position */
#define MXC_F_SFE_FLASH_STA_FLASH_STA                  ((uint32_t)(0x3FFFFFUL << MXC_F_SFE_FLASH_STA_FLASH_STA_POS)) /**< FLASH_STA_FLASH_STA Mask */

/**@} end of group SFE_FLASH_STA_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_RAM_SBA SFE_RAM_SBA
 * @brief    RAM System Base Address Register.
 * @{
 */
#define MXC_F_SFE_RAM_SBA_RAM_SBA_POS                  10 /**< RAM_SBA_RAM_SBA Position */
#define MXC_F_SFE_RAM_SBA_RAM_SBA                      ((uint32_t)(0x3FFFFFUL << MXC_F_SFE_RAM_SBA_RAM_SBA_POS)) /**< RAM_SBA_RAM_SBA Mask */

/**@} end of group SFE_RAM_SBA_Register */

/**
 * @ingroup  sfe_registers
 * @defgroup SFE_RAM_STA SFE_RAM_STA
 * @brief    RAM System Top Address Register.
 * @{
 */
#define MXC_F_SFE_RAM_STA_RAM_STA_POS                  10 /**< RAM_STA_RAM_STA Position */
#define MXC_F_SFE_RAM_STA_RAM_STA                      ((uint32_t)(0x3FFFFFUL << MXC_F_SFE_RAM_STA_RAM_STA_POS)) /**< RAM_STA_RAM_STA Mask */

/**@} end of group SFE_RAM_STA_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_SFE_REGS_H_
