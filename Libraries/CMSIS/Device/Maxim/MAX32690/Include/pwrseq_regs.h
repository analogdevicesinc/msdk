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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t lpwkst2;              /**< <tt>\b 0x14:</tt> PWRSEQ LPWKST2 Register */
    __IO uint32_t lpwken2;              /**< <tt>\b 0x18:</tt> PWRSEQ LPWKEN2 Register */
    __IO uint32_t lpwkst3;              /**< <tt>\b 0x1C:</tt> PWRSEQ LPWKST3 Register */
    __IO uint32_t lpwken3;              /**< <tt>\b 0x20:</tt> PWRSEQ LPWKEN3 Register */
    __IO uint32_t lpwkst4;              /**< <tt>\b 0x24:</tt> PWRSEQ LPWKST4 Register */
    __IO uint32_t lpwken4;              /**< <tt>\b 0x28:</tt> PWRSEQ LPWKEN4 Register */
    __R  uint32_t rsv_0x2c;
    __IO uint32_t lppwst;               /**< <tt>\b 0x30:</tt> PWRSEQ LPPWST Register */
    __IO uint32_t lppwen;               /**< <tt>\b 0x34:</tt> PWRSEQ LPPWEN Register */
    __R  uint32_t rsv_0x38_0x47[4];
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
#define MXC_R_PWRSEQ_LPWKST2               ((uint32_t)0x00000014UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0014</tt> */
#define MXC_R_PWRSEQ_LPWKEN2               ((uint32_t)0x00000018UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0018</tt> */
#define MXC_R_PWRSEQ_LPWKST3               ((uint32_t)0x0000001CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x001C</tt> */
#define MXC_R_PWRSEQ_LPWKEN3               ((uint32_t)0x00000020UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0020</tt> */
#define MXC_R_PWRSEQ_LPWKST4               ((uint32_t)0x00000024UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0024</tt> */
#define MXC_R_PWRSEQ_LPWKEN4               ((uint32_t)0x00000028UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0028</tt> */
#define MXC_R_PWRSEQ_LPPWST                ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPPWEN                ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
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

#define MXC_F_PWRSEQ_LPCN_RAMRET5_POS                  5 /**< LPCN_RAMRET5 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET5                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET5_POS)) /**< LPCN_RAMRET5 Mask */

#define MXC_F_PWRSEQ_LPCN_RAMRET6_POS                  6 /**< LPCN_RAMRET6 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET6                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET6_POS)) /**< LPCN_RAMRET6 Mask */

#define MXC_F_PWRSEQ_LPCN_RAMRET8_POS                  7 /**< LPCN_RAMRET8 Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET8                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RAMRET8_POS)) /**< LPCN_RAMRET8 Mask */

#define MXC_F_PWRSEQ_LPCN_ISOCLK_SELECT_POS            8 /**< LPCN_ISOCLK_SELECT Position */
#define MXC_F_PWRSEQ_LPCN_ISOCLK_SELECT                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_ISOCLK_SELECT_POS)) /**< LPCN_ISOCLK_SELECT Mask */

#define MXC_F_PWRSEQ_LPCN_FAST_ENTRY_DIS_POS           9 /**< LPCN_FAST_ENTRY_DIS Position */
#define MXC_F_PWRSEQ_LPCN_FAST_ENTRY_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_FAST_ENTRY_DIS_POS)) /**< LPCN_FAST_ENTRY_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_BGOFF_POS                    11 /**< LPCN_BGOFF Position */
#define MXC_F_PWRSEQ_LPCN_BGOFF                        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_BGOFF_POS)) /**< LPCN_BGOFF Mask */

#define MXC_F_PWRSEQ_LPCN_WKRST_POS                    31 /**< LPCN_WKRST Position */
#define MXC_F_PWRSEQ_LPCN_WKRST                        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_WKRST_POS)) /**< LPCN_WKRST Mask */

/**@} end of group PWRSEQ_LPCN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST0 PWRSEQ_LPWKST0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST0_WAKEST_POS                0 /**< LPWKST0_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST0_WAKEST                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPWKST0_WAKEST_POS)) /**< LPWKST0_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_WAKEEN_POS                0 /**< LPWKEN0_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWKEN0_WAKEEN                    ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKEN0_WAKEEN_POS)) /**< LPWKEN0_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST1 PWRSEQ_LPWKST1
 * @brief    Low Power I/O Wakeup Status Register 1. This register indicates the low power
 *           wakeup status for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST1_WAKEST_POS                0 /**< LPWKST1_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST1_WAKEST                    ((uint32_t)(0x3FFUL << MXC_F_PWRSEQ_LPWKST1_WAKEST_POS)) /**< LPWKST1_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN1 PWRSEQ_LPWKEN1
 * @brief    Low Power I/O Wakeup Enable Register 1. This register enables low power wakeup
 *           functionality for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN1_WAKEEN_POS                0 /**< LPWKEN1_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWKEN1_WAKEEN                    ((uint32_t)(0x3FFUL << MXC_F_PWRSEQ_LPWKEN1_WAKEEN_POS)) /**< LPWKEN1_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWKEN1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST2 PWRSEQ_LPWKST2
 * @brief    Low Power I/O Wakeup Status Register 2. This register indicates the low power
 *           wakeup status for GPIO2.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST2_WAKEST_POS                0 /**< LPWKST2_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST2_WAKEST                    ((uint32_t)(0xFFUL << MXC_F_PWRSEQ_LPWKST2_WAKEST_POS)) /**< LPWKST2_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST2_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN2 PWRSEQ_LPWKEN2
 * @brief    Low Power I/O Wakeup Enable Register 2. This register enables low power wakeup
 *           functionality for GPIO2.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN2_WAKEEN_POS                0 /**< LPWKEN2_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWKEN2_WAKEEN                    ((uint32_t)(0xFFUL << MXC_F_PWRSEQ_LPWKEN2_WAKEEN_POS)) /**< LPWKEN2_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWKEN2_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST3 PWRSEQ_LPWKST3
 * @brief    Low Power I/O Wakeup Status Register 3. This register indicates the low power
 *           wakeup status for GPIO3.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST3_WAKEST_POS                0 /**< LPWKST3_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST3_WAKEST                    ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKST3_WAKEST_POS)) /**< LPWKST3_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST3_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN3 PWRSEQ_LPWKEN3
 * @brief    Low Power I/O Wakeup Enable Register 3. This register enables low power wakeup
 *           functionality for GPIO3.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN3_WAKEEN_POS                0 /**< LPWKEN3_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWKEN3_WAKEEN                    ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKEN3_WAKEEN_POS)) /**< LPWKEN3_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWKEN3_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST4 PWRSEQ_LPWKST4
 * @brief    Low Power I/O Wakeup Status Register 4. This register indicates the low power
 *           wakeup status for GPIO4.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST4_WAKEST_POS                0 /**< LPWKST4_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST4_WAKEST                    ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKST4_WAKEST_POS)) /**< LPWKST4_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST4_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN4 PWRSEQ_LPWKEN4
 * @brief    Low Power I/O Wakeup Enable Register 4. This register enables low power wakeup
 *           functionality for GPIO4.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN4_WAKEEN_POS                0 /**< LPWKEN4_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWKEN4_WAKEEN                    ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKEN4_WAKEEN_POS)) /**< LPWKEN4_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWKEN4_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWST PWRSEQ_LPPWST
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP0_POS               4 /**< LPPWST_AINCOMP0 Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP0                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP0_POS)) /**< LPPWST_AINCOMP0 Mask */

#define MXC_F_PWRSEQ_LPPWST_BACKUP_POS                 16 /**< LPPWST_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWST_BACKUP                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_BACKUP_POS)) /**< LPPWST_BACKUP Mask */

#define MXC_F_PWRSEQ_LPPWST_RESET_POS                  17 /**< LPPWST_RESET Position */
#define MXC_F_PWRSEQ_LPPWST_RESET                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_RESET_POS)) /**< LPPWST_RESET Mask */

/**@} end of group PWRSEQ_LPPWST_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWEN PWRSEQ_LPPWEN
 * @brief    Low Power Peripheral Wakeup Enable Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWEN_USBLS_POS                  0 /**< LPPWEN_USBLS Position */
#define MXC_F_PWRSEQ_LPPWEN_USBLS                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWEN_USBLS_POS)) /**< LPPWEN_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWEN_USBVBUS_POS                2 /**< LPPWEN_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWEN_USBVBUS                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_USBVBUS_POS)) /**< LPPWEN_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWEN_AINCOMP0_POS               4 /**< LPPWEN_AINCOMP0 Position */
#define MXC_F_PWRSEQ_LPPWEN_AINCOMP0                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_AINCOMP0_POS)) /**< LPPWEN_AINCOMP0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_WDT0_POS                   8 /**< LPPWEN_WDT0 Position */
#define MXC_F_PWRSEQ_LPPWEN_WDT0                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_WDT0_POS)) /**< LPPWEN_WDT0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_WDT1_POS                   9 /**< LPPWEN_WDT1 Position */
#define MXC_F_PWRSEQ_LPPWEN_WDT1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_WDT1_POS)) /**< LPPWEN_WDT1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_CPU1_POS                   10 /**< LPPWEN_CPU1 Position */
#define MXC_F_PWRSEQ_LPPWEN_CPU1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_CPU1_POS)) /**< LPPWEN_CPU1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_TMR0_POS                   11 /**< LPPWEN_TMR0 Position */
#define MXC_F_PWRSEQ_LPPWEN_TMR0                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_TMR0_POS)) /**< LPPWEN_TMR0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_TMR1_POS                   12 /**< LPPWEN_TMR1 Position */
#define MXC_F_PWRSEQ_LPPWEN_TMR1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_TMR1_POS)) /**< LPPWEN_TMR1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_TMR2_POS                   13 /**< LPPWEN_TMR2 Position */
#define MXC_F_PWRSEQ_LPPWEN_TMR2                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_TMR2_POS)) /**< LPPWEN_TMR2 Mask */

#define MXC_F_PWRSEQ_LPPWEN_TMR3_POS                   14 /**< LPPWEN_TMR3 Position */
#define MXC_F_PWRSEQ_LPPWEN_TMR3                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_TMR3_POS)) /**< LPPWEN_TMR3 Mask */

#define MXC_F_PWRSEQ_LPPWEN_TMR4_POS                   15 /**< LPPWEN_TMR4 Position */
#define MXC_F_PWRSEQ_LPPWEN_TMR4                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_TMR4_POS)) /**< LPPWEN_TMR4 Mask */

#define MXC_F_PWRSEQ_LPPWEN_TMR5_POS                   16 /**< LPPWEN_TMR5 Position */
#define MXC_F_PWRSEQ_LPPWEN_TMR5                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_TMR5_POS)) /**< LPPWEN_TMR5 Mask */

#define MXC_F_PWRSEQ_LPPWEN_UART0_POS                  17 /**< LPPWEN_UART0 Position */
#define MXC_F_PWRSEQ_LPPWEN_UART0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_UART0_POS)) /**< LPPWEN_UART0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_UART1_POS                  18 /**< LPPWEN_UART1 Position */
#define MXC_F_PWRSEQ_LPPWEN_UART1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_UART1_POS)) /**< LPPWEN_UART1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_UART2_POS                  19 /**< LPPWEN_UART2 Position */
#define MXC_F_PWRSEQ_LPPWEN_UART2                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_UART2_POS)) /**< LPPWEN_UART2 Mask */

#define MXC_F_PWRSEQ_LPPWEN_UART3_POS                  20 /**< LPPWEN_UART3 Position */
#define MXC_F_PWRSEQ_LPPWEN_UART3                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_UART3_POS)) /**< LPPWEN_UART3 Mask */

#define MXC_F_PWRSEQ_LPPWEN_I2C0_POS                   21 /**< LPPWEN_I2C0 Position */
#define MXC_F_PWRSEQ_LPPWEN_I2C0                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_I2C0_POS)) /**< LPPWEN_I2C0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_I2C1_POS                   22 /**< LPPWEN_I2C1 Position */
#define MXC_F_PWRSEQ_LPPWEN_I2C1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_I2C1_POS)) /**< LPPWEN_I2C1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_I2C2_POS                   23 /**< LPPWEN_I2C2 Position */
#define MXC_F_PWRSEQ_LPPWEN_I2C2                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_I2C2_POS)) /**< LPPWEN_I2C2 Mask */

#define MXC_F_PWRSEQ_LPPWEN_I2S_POS                    24 /**< LPPWEN_I2S Position */
#define MXC_F_PWRSEQ_LPPWEN_I2S                        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_I2S_POS)) /**< LPPWEN_I2S Mask */

#define MXC_F_PWRSEQ_LPPWEN_SPI0_POS                   25 /**< LPPWEN_SPI0 Position */
#define MXC_F_PWRSEQ_LPPWEN_SPI0                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_SPI0_POS)) /**< LPPWEN_SPI0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_LPCMP_POS                  26 /**< LPPWEN_LPCMP Position */
#define MXC_F_PWRSEQ_LPPWEN_LPCMP                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_LPCMP_POS)) /**< LPPWEN_LPCMP Mask */

#define MXC_F_PWRSEQ_LPPWEN_BTLE_POS                   27 /**< LPPWEN_BTLE Position */
#define MXC_F_PWRSEQ_LPPWEN_BTLE                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_BTLE_POS)) /**< LPPWEN_BTLE Mask */

#define MXC_F_PWRSEQ_LPPWEN_SPI1_POS                   28 /**< LPPWEN_SPI1 Position */
#define MXC_F_PWRSEQ_LPPWEN_SPI1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_SPI1_POS)) /**< LPPWEN_SPI1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_SPI2_POS                   29 /**< LPPWEN_SPI2 Position */
#define MXC_F_PWRSEQ_LPPWEN_SPI2                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_SPI2_POS)) /**< LPPWEN_SPI2 Mask */

#define MXC_F_PWRSEQ_LPPWEN_CAN0_POS                   30 /**< LPPWEN_CAN0 Position */
#define MXC_F_PWRSEQ_LPPWEN_CAN0                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_CAN0_POS)) /**< LPPWEN_CAN0 Mask */

#define MXC_F_PWRSEQ_LPPWEN_CAN1_POS                   31 /**< LPPWEN_CAN1 Position */
#define MXC_F_PWRSEQ_LPPWEN_CAN1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_CAN1_POS)) /**< LPPWEN_CAN1 Mask */

/**@} end of group PWRSEQ_LPPWEN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_PWRSEQ_REGS_H_
