/**
 * @file    pwrseq_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @note    This file is @generated.
 * @ingroup pwrseq_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_PWRSEQ_REGS_H_

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
 * @ingroup     pwrseq
 * @defgroup    pwrseq_registers PWRSEQ_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @details     Power Sequencer / Low Power Control Register.
 */

/**
 * @ingroup pwrseq_registers
 * Structure type to access the PWRSEQ Registers.
 */
typedef struct {
    __IO uint32_t lpcn;                 /**< <tt>\b 0x00:</tt> PWRSEQ LPCN Register */
    __IO uint32_t lpwkst0;              /**< <tt>\b 0x04:</tt> PWRSEQ LPWKST0 Register */
    __IO uint32_t lpwken0;              /**< <tt>\b 0x08:</tt> PWRSEQ LPWKEN0 Register */
    __IO uint32_t lpwkst1;              /**< <tt>\b 0x0C:</tt> PWRSEQ LPWKST1 Register */
    __IO uint32_t lpwken1;              /**< <tt>\b 0x10:</tt> PWRSEQ LPWKEN1 Register */
    __R  uint32_t rsv_0x14_0x2f[7];
    __IO uint32_t lppwkst;              /**< <tt>\b 0x30:</tt> PWRSEQ LPPWKST Register */
    __R  uint32_t rsv_0x34_0x3f[3];
    __IO uint32_t lpmemsd;              /**< <tt>\b 0x40:</tt> PWRSEQ LPMEMSD Register */
    __R  uint32_t rsv_0x44;
    __IO uint32_t gp0;                  /**< <tt>\b 0x48:</tt> PWRSEQ GP0 Register */
    __IO uint32_t gp1;                  /**< <tt>\b 0x4C:</tt> PWRSEQ GP1 Register */
} mxc_pwrseq_regs_t;

/* Register offsets for module PWRSEQ */
/**
 * @ingroup    pwrseq_registers
 * @defgroup   PWRSEQ_Register_Offsets Register Offsets
 * @brief      PWRSEQ Peripheral Register Offsets from the PWRSEQ Base Peripheral Address.
 * @{
 */
#define MXC_R_PWRSEQ_LPCN                  ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_LPWKST0               ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_LPWKEN0               ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_LPWKST1               ((uint32_t)0x0000000CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x000C</tt> */
#define MXC_R_PWRSEQ_LPWKEN1               ((uint32_t)0x00000010UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0010</tt> */
#define MXC_R_PWRSEQ_LPPWKST               ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
#define MXC_R_PWRSEQ_GP0                   ((uint32_t)0x00000048UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0048</tt> */
#define MXC_R_PWRSEQ_GP1                   ((uint32_t)0x0000004CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x004C</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCN PWRSEQ_LPCN
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCN_RAMRET_EN_POS                0 /**< LPCN_RAMRET_EN Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET_EN                    ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCN_RAMRET_EN_POS)) /**< LPCN_RAMRET_EN Mask */

#define MXC_F_PWRSEQ_LPCN_LDO_DIS_POS                  16 /**< LPCN_LDO_DIS Position */
#define MXC_F_PWRSEQ_LPCN_LDO_DIS                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_LDO_DIS_POS)) /**< LPCN_LDO_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VCOREMON_DIS_POS             20 /**< LPCN_VCOREMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VCOREMON_DIS                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VCOREMON_DIS_POS)) /**< LPCN_VCOREMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VDDAMON_DIS_POS              22 /**< LPCN_VDDAMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VDDAMON_DIS                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDAMON_DIS_POS)) /**< LPCN_VDDAMON_DIS Mask */

/**@} end of group PWRSEQ_LPCN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST0 PWRSEQ_LPWKST0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST0_ST_POS                    0 /**< LPWKST0_ST Position */
#define MXC_F_PWRSEQ_LPWKST0_ST                        ((uint32_t)(0xFFFFUL << MXC_F_PWRSEQ_LPWKST0_ST_POS)) /**< LPWKST0_ST Mask */

/**@} end of group PWRSEQ_LPWKST0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_EN_POS                    0 /**< LPWKEN0_EN Position */
#define MXC_F_PWRSEQ_LPWKEN0_EN                        ((uint32_t)(0xFFFFUL << MXC_F_PWRSEQ_LPWKEN0_EN_POS)) /**< LPWKEN0_EN Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST1 PWRSEQ_LPWKST1
 * @brief    Low Power I/O Wakeup Status Register 1. This register indicates the low power
 *           wakeup status for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST1_ST_POS                    0 /**< LPWKST1_ST Position */
#define MXC_F_PWRSEQ_LPWKST1_ST                        ((uint32_t)(0x7FFUL << MXC_F_PWRSEQ_LPWKST1_ST_POS)) /**< LPWKST1_ST Mask */

/**@} end of group PWRSEQ_LPWKST1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN1 PWRSEQ_LPWKEN1
 * @brief    Low Power I/O Wakeup Enable Register 1. This register enables low power wakeup
 *           functionality for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN1_EN_POS                    0 /**< LPWKEN1_EN Position */
#define MXC_F_PWRSEQ_LPWKEN1_EN                        ((uint32_t)(0x7FFUL << MXC_F_PWRSEQ_LPWKEN1_EN_POS)) /**< LPWKEN1_EN Mask */

/**@} end of group PWRSEQ_LPWKEN1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKST PWRSEQ_LPPWKST
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKST_BBMOD_POS                 16 /**< LPPWKST_BBMOD Position */
#define MXC_F_PWRSEQ_LPPWKST_BBMOD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_BBMOD_POS)) /**< LPPWKST_BBMOD Mask */

#define MXC_F_PWRSEQ_LPPWKST_RST_POS                   17 /**< LPPWKST_RST Position */
#define MXC_F_PWRSEQ_LPPWKST_RST                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_RST_POS)) /**< LPPWKST_RST Mask */

#define MXC_F_PWRSEQ_LPPWKST_SDMA1_POS                 18 /**< LPPWKST_SDMA1 Position */
#define MXC_F_PWRSEQ_LPPWKST_SDMA1                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_SDMA1_POS)) /**< LPPWKST_SDMA1 Mask */

/**@} end of group PWRSEQ_LPPWKST_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPMEMSD PWRSEQ_LPMEMSD
 * @brief    Low Power Memory Shutdown Control.
 * @{
 */
#define MXC_F_PWRSEQ_LPMEMSD_RAM0_POS                  0 /**< LPMEMSD_RAM0 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM0_POS)) /**< LPMEMSD_RAM0 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM1_POS                  1 /**< LPMEMSD_RAM1 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM1_POS)) /**< LPMEMSD_RAM1 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM2_POS                  2 /**< LPMEMSD_RAM2 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM2                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM2_POS)) /**< LPMEMSD_RAM2 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM3_POS                  3 /**< LPMEMSD_RAM3 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM3                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM3_POS)) /**< LPMEMSD_RAM3 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_RAM4_POS                  4 /**< LPMEMSD_RAM4 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM4                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM4_POS)) /**< LPMEMSD_RAM4 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICACHE_POS                7 /**< LPMEMSD_ICACHE Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICACHE                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICACHE_POS)) /**< LPMEMSD_ICACHE Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICACHEXIP_POS             8 /**< LPMEMSD_ICACHEXIP Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICACHEXIP                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICACHEXIP_POS)) /**< LPMEMSD_ICACHEXIP Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM_POS                   12 /**< LPMEMSD_ROM Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM_POS)) /**< LPMEMSD_ROM Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_PWRSEQ_REGS_H_
