/**
 * @file    pwrseq_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @note    This file is @generated.
 * @ingroup pwrseq_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t lppwst;               /**< <tt>\b 0x30:</tt> PWRSEQ LPPWST Register */
    __R  uint32_t rsv_0x34_0x47[5];
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
#define MXC_R_PWRSEQ_LPPWST                ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_GP0                   ((uint32_t)0x00000048UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0048</tt> */
#define MXC_R_PWRSEQ_GP1                   ((uint32_t)0x0000004CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x004C</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCN PWRSEQ_LPCN
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCN_RAMRET0_POS                  0 /**< LPCN_RAMRET0 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET0_POS)) /**< LPCN_RAMRET0 Mask */

#define MXC_F_PWRSEQ_LPCN_RAMRET1_POS                  1 /**< LPCN_RAMRET1 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET1_POS)) /**< LPCN_RAMRET1 Mask */

#define MXC_F_PWRSEQ_LPCN_RAMRET2_POS                  2 /**< LPCN_RAMRET2 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET2                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET2_POS)) /**< LPCN_RAMRET2 Mask */

#define MXC_F_PWRSEQ_LPCN_RAMRET3_POS                  3 /**< LPCN_RAMRET3 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET3                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET3_POS)) /**< LPCN_RAMRET3 Mask */

#define MXC_F_PWRSEQ_LPCN_RAMRET4_POS                  4 /**< LPCN_RAMRET4 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET4                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET4_POS)) /**< LPCN_RAMRET4 Mask */

#define MXC_F_PWRSEQ_LPCN_LPMFAST_POS                  8 /**< LPCN_LPMFAST Position */
#define MXC_F_PWRSEQ_LPCN_LPMFAST                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_LPMFAST_POS)) /**< LPCN_LPMFAST Mask */

#define MXC_F_PWRSEQ_LPCN_BG_DIS_POS                   11 /**< LPCN_BG_DIS Position */
#define MXC_F_PWRSEQ_LPCN_BG_DIS                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_BG_DIS_POS)) /**< LPCN_BG_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_LPWKST_CLR_POS               31 /**< LPCN_LPWKST_CLR Position */
#define MXC_F_PWRSEQ_LPCN_LPWKST_CLR                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_LPWKST_CLR_POS)) /**< LPCN_LPWKST_CLR Mask */

/**@} end of group PWRSEQ_LPCN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST0 PWRSEQ_LPWKST0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST0_ALL_POS                   0 /**< LPWKST0_ALL Position */
#define MXC_F_PWRSEQ_LPWKST0_ALL                       ((uint32_t)(0xFFFUL << MXC_F_PWRSEQ_LPWKST0_ALL_POS)) /**< LPWKST0_ALL Mask */

/**@} end of group PWRSEQ_LPWKST0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_ALL_POS                   0 /**< LPWKEN0_ALL Position */
#define MXC_F_PWRSEQ_LPWKEN0_ALL                       ((uint32_t)(0xFFFUL << MXC_F_PWRSEQ_LPWKEN0_ALL_POS)) /**< LPWKEN0_ALL Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST1 PWRSEQ_LPWKST1
 * @brief    Low Power I/O Wakeup Status Register 1. This register indicates the low power
 *           wakeup status for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST1_WAKEST_POS                0 /**< LPWKST1_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST1_WAKEST                    ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKST1_WAKEST_POS)) /**< LPWKST1_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN1 PWRSEQ_LPWKEN1
 * @brief    Low Power I/O Wakeup Enable Register 1. This register enables low power wakeup
 *           functionality for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN1_EN_POS                    0 /**< LPWKEN1_EN Position */
#define MXC_F_PWRSEQ_LPWKEN1_EN                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKEN1_EN_POS)) /**< LPWKEN1_EN Mask */

/**@} end of group PWRSEQ_LPWKEN1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWST PWRSEQ_LPPWST
 * @brief    Low Power Peripheral Wakeup Status Flags.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWST_BACKUP_POS                 16 /**< LPPWST_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWST_BACKUP                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_BACKUP_POS)) /**< LPPWST_BACKUP Mask */

#define MXC_F_PWRSEQ_LPPWST_RESET_POS                  17 /**< LPPWST_RESET Position */
#define MXC_F_PWRSEQ_LPPWST_RESET                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_RESET_POS)) /**< LPPWST_RESET Mask */

/**@} end of group PWRSEQ_LPPWST_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GP0 PWRSEQ_GP0
 * @brief    Low Power General Purpose Register 0.
 * @{
 */
#define MXC_F_PWRSEQ_GP0_ALL_POS                       0 /**< GP0_ALL Position */
#define MXC_F_PWRSEQ_GP0_ALL                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GP0_ALL_POS)) /**< GP0_ALL Mask */

/**@} end of group PWRSEQ_GP0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_GP1 PWRSEQ_GP1
 * @brief    Low Power General Purpose Register 1.
 * @{
 */
#define MXC_F_PWRSEQ_GP1_ALL_POS                       0 /**< GP1_ALL Position */
#define MXC_F_PWRSEQ_GP1_ALL                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_GP1_ALL_POS)) /**< GP1_ALL Mask */

/**@} end of group PWRSEQ_GP1_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_PWRSEQ_REGS_H_
