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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t lpwkst2;              /**< <tt>\b 0x14:</tt> PWRSEQ LPWKST2 Register */
    __IO uint32_t lpwken2;              /**< <tt>\b 0x18:</tt> PWRSEQ LPWKEN2 Register */
    __IO uint32_t lpwkst3;              /**< <tt>\b 0x1C:</tt> PWRSEQ LPWKST3 Register */
    __IO uint32_t lpwken3;              /**< <tt>\b 0x20:</tt> PWRSEQ LPWKEN3 Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t lppwst;               /**< <tt>\b 0x30:</tt> PWRSEQ LPPWST Register */
    __IO uint32_t lppwen;               /**< <tt>\b 0x34:</tt> PWRSEQ LPPWEN Register */
    __R  uint32_t rsv_0x38_0x43[3];
    __IO uint32_t vbtlepd;              /**< <tt>\b 0x44:</tt> PWRSEQ VBTLEPD Register */
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
#define MXC_R_PWRSEQ_LPPWST                ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPPWEN                ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_VBTLEPD               ((uint32_t)0x00000044UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0044</tt> */
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

#define MXC_F_PWRSEQ_LPCN_LPMCLKSEL_POS                8 /**< LPCN_LPMCLKSEL Position */
#define MXC_F_PWRSEQ_LPCN_LPMCLKSEL                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_LPMCLKSEL_POS)) /**< LPCN_LPMCLKSEL Mask */

#define MXC_F_PWRSEQ_LPCN_LPMFAST_POS                  9 /**< LPCN_LPMFAST Position */
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

#define MXC_F_PWRSEQ_LPPWEN_SPI1_POS                   25 /**< LPPWEN_SPI1 Position */
#define MXC_F_PWRSEQ_LPPWEN_SPI1                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_SPI1_POS)) /**< LPPWEN_SPI1 Mask */

#define MXC_F_PWRSEQ_LPPWEN_LPCMP_POS                  26 /**< LPPWEN_LPCMP Position */
#define MXC_F_PWRSEQ_LPPWEN_LPCMP                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_LPCMP_POS)) /**< LPPWEN_LPCMP Mask */

/**@} end of group PWRSEQ_LPPWEN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_VBTLEPD PWRSEQ_VBTLEPD
 * @brief    Low-Power VBTLE Power Down Register.
 * @{
 */
#define MXC_F_PWRSEQ_VBTLEPD_BTLE_POS                  1 /**< VBTLEPD_BTLE Position */
#define MXC_F_PWRSEQ_VBTLEPD_BTLE                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_VBTLEPD_BTLE_POS)) /**< VBTLEPD_BTLE Mask */

/**@} end of group PWRSEQ_VBTLEPD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_PWRSEQ_REGS_H_
