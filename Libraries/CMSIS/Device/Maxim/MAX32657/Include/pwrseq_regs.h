/**
 * @file    pwrseq_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @note    This file is @generated.
 * @ingroup pwrseq_registers
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
    __IO uint32_t lpctrl;               /**< <tt>\b 0x00:</tt> PWRSEQ LPCTRL Register */
    __IO uint32_t lpwkfl0;              /**< <tt>\b 0x04:</tt> PWRSEQ LPWKFL0 Register */
    __IO uint32_t lpwken0;              /**< <tt>\b 0x08:</tt> PWRSEQ LPWKEN0 Register */
    __IO uint32_t lpwkfl1;              /**< <tt>\b 0x0C:</tt> PWRSEQ LPWKFL1 Register */
    __IO uint32_t lpwken1;              /**< <tt>\b 0x10:</tt> PWRSEQ LPWKEN1 Register */
    __IO uint32_t lpwkfl2;              /**< <tt>\b 0x14:</tt> PWRSEQ LPWKFL2 Register */
    __IO uint32_t lpwken2;              /**< <tt>\b 0x18:</tt> PWRSEQ LPWKEN2 Register */
    __IO uint32_t lpwkfl3;              /**< <tt>\b 0x1C:</tt> PWRSEQ LPWKFL3 Register */
    __IO uint32_t lpwken3;              /**< <tt>\b 0x20:</tt> PWRSEQ LPWKEN3 Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t lppwkfl;              /**< <tt>\b 0x30:</tt> PWRSEQ LPPWKFL Register */
    __IO uint32_t lppwken;              /**< <tt>\b 0x34:</tt> PWRSEQ LPPWKEN Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t lpmemsd;              /**< <tt>\b 0x40:</tt> PWRSEQ LPMEMSD Register */
    __IO uint32_t lpvddpd;              /**< <tt>\b 0x44:</tt> PWRSEQ LPVDDPD Register */
    __IO uint32_t gp0;                  /**< <tt>\b 0x48:</tt> PWRSEQ GP0 Register */
    __IO uint32_t gp1;                  /**< <tt>\b 0x4C:</tt> PWRSEQ GP1 Register */
    __IO uint32_t lpwkpol0a;            /**< <tt>\b 0x50:</tt> PWRSEQ LPWKPOL0A Register */
    __IO uint32_t lpwkpol0b;            /**< <tt>\b 0x54:</tt> PWRSEQ LPWKPOL0B Register */
    __IO uint32_t lpwkpol1a;            /**< <tt>\b 0x58:</tt> PWRSEQ LPWKPOL1A Register */
    __IO uint32_t lpwkpol1b;            /**< <tt>\b 0x5C:</tt> PWRSEQ LPWKPOL1B Register */
    __IO uint32_t lpwkpol2a;            /**< <tt>\b 0x60:</tt> PWRSEQ LPWKPOL2A Register */
    __IO uint32_t lpwkpol2b;            /**< <tt>\b 0x64:</tt> PWRSEQ LPWKPOL2B Register */
    __IO uint32_t lpwkpol3;             /**< <tt>\b 0x68:</tt> PWRSEQ LPWKPOL3 Register */
} mxc_pwrseq_regs_t;

/* Register offsets for module PWRSEQ */
/**
 * @ingroup    pwrseq_registers
 * @defgroup   PWRSEQ_Register_Offsets Register Offsets
 * @brief      PWRSEQ Peripheral Register Offsets from the PWRSEQ Base Peripheral Address.
 * @{
 */
#define MXC_R_PWRSEQ_LPCTRL                ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_LPWKFL0               ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_LPWKEN0               ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_LPWKFL1               ((uint32_t)0x0000000CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x000C</tt> */
#define MXC_R_PWRSEQ_LPWKEN1               ((uint32_t)0x00000010UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0010</tt> */
#define MXC_R_PWRSEQ_LPWKFL2               ((uint32_t)0x00000014UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0014</tt> */
#define MXC_R_PWRSEQ_LPWKEN2               ((uint32_t)0x00000018UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0018</tt> */
#define MXC_R_PWRSEQ_LPWKFL3               ((uint32_t)0x0000001CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x001C</tt> */
#define MXC_R_PWRSEQ_LPWKEN3               ((uint32_t)0x00000020UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0020</tt> */
#define MXC_R_PWRSEQ_LPPWKFL               ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPPWKEN               ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
#define MXC_R_PWRSEQ_LPVDDPD               ((uint32_t)0x00000044UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0044</tt> */
#define MXC_R_PWRSEQ_GP0                   ((uint32_t)0x00000048UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0048</tt> */
#define MXC_R_PWRSEQ_GP1                   ((uint32_t)0x0000004CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x004C</tt> */
#define MXC_R_PWRSEQ_LPWKPOL0A             ((uint32_t)0x00000050UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0050</tt> */
#define MXC_R_PWRSEQ_LPWKPOL0B             ((uint32_t)0x00000054UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0054</tt> */
#define MXC_R_PWRSEQ_LPWKPOL1A             ((uint32_t)0x00000058UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0058</tt> */
#define MXC_R_PWRSEQ_LPWKPOL1B             ((uint32_t)0x0000005CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x005C</tt> */
#define MXC_R_PWRSEQ_LPWKPOL2A             ((uint32_t)0x00000060UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0060</tt> */
#define MXC_R_PWRSEQ_LPWKPOL2B             ((uint32_t)0x00000064UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0064</tt> */
#define MXC_R_PWRSEQ_LPWKPOL3              ((uint32_t)0x00000068UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0068</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCTRL PWRSEQ_LPCTRL
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCTRL_RAMRET_EN_POS              0 /**< LPCTRL_RAMRET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RAMRET_EN                  ((uint32_t)(0xFUL << MXC_F_PWRSEQ_LPCTRL_RAMRET_EN_POS)) /**< LPCTRL_RAMRET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_OVR_POS                    4 /**< LPCTRL_OVR Position */
#define MXC_F_PWRSEQ_LPCTRL_OVR                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCTRL_OVR_POS)) /**< LPCTRL_OVR Mask */
#define MXC_V_PWRSEQ_LPCTRL_OVR_1_1V                   ((uint32_t)0x2UL) /**< LPCTRL_OVR_1_1V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_1_1V                   (MXC_V_PWRSEQ_LPCTRL_OVR_1_1V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_1_1V Setting */

#define MXC_F_PWRSEQ_LPCTRL_RETREG_EN_POS              8 /**< LPCTRL_RETREG_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RETREG_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RETREG_EN_POS)) /**< LPCTRL_RETREG_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_FASTWK_EN_POS              10 /**< LPCTRL_FASTWK_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_FASTWK_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_FASTWK_EN_POS)) /**< LPCTRL_FASTWK_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS                 11 /**< LPCTRL_BG_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_BG_DIS                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS)) /**< LPCTRL_BG_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_PORVDDCMON_DIS_POS         12 /**< LPCTRL_PORVDDCMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_PORVDDCMON_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_PORVDDCMON_DIS_POS)) /**< LPCTRL_PORVDDCMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOHHVMON_DIS_POS        17 /**< LPCTRL_VDDIOHHVMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOHHVMON_DIS            ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOHHVMON_DIS_POS)) /**< LPCTRL_VDDIOHHVMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOHVMON_DIS_POS         18 /**< LPCTRL_VDDIOHVMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOHVMON_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOHVMON_DIS_POS)) /**< LPCTRL_VDDIOHVMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREHVMON_DIS_POS         19 /**< LPCTRL_VCOREHVMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREHVMON_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREHVMON_DIS_POS)) /**< LPCTRL_VCOREHVMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDCMON_DIS_POS            20 /**< LPCTRL_VDDCMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDCMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDCMON_DIS_POS)) /**< LPCTRL_VDDCMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VRTCMON_DIS_POS            21 /**< LPCTRL_VRTCMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VRTCMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VRTCMON_DIS_POS)) /**< LPCTRL_VRTCMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS_POS            22 /**< LPCTRL_VDDAMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS_POS)) /**< LPCTRL_VDDAMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOMON_DIS_POS           23 /**< LPCTRL_VDDIOMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOMON_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOMON_DIS_POS)) /**< LPCTRL_VDDIOMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOHMON_DIS_POS          24 /**< LPCTRL_VDDIOHMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOHMON_DIS              ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOHMON_DIS_POS)) /**< LPCTRL_VDDIOHMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDBMON_DIS_POS            27 /**< LPCTRL_VDDBMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDBMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDBMON_DIS_POS)) /**< LPCTRL_VDDBMON_DIS Mask */

/**@} end of group PWRSEQ_LPCTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKFL0 PWRSEQ_LPWKFL0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKFL0_ALL_POS                   0 /**< LPWKFL0_ALL Position */
#define MXC_F_PWRSEQ_LPWKFL0_ALL                       ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKFL0_ALL_POS)) /**< LPWKFL0_ALL Mask */

/**@} end of group PWRSEQ_LPWKFL0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_ALL_POS                   0 /**< LPWKEN0_ALL Position */
#define MXC_F_PWRSEQ_LPWKEN0_ALL                       ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKEN0_ALL_POS)) /**< LPWKEN0_ALL Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKFL PWRSEQ_LPPWKFL
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKFL_USBLS_POS                 0 /**< LPPWKFL_USBLS Position */
#define MXC_F_PWRSEQ_LPPWKFL_USBLS                     ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWKFL_USBLS_POS)) /**< LPPWKFL_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_POS              4 /**< LPPWKFL_AINCOMP0 Position */
#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP0                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_POS)) /**< LPPWKFL_AINCOMP0 Mask */

#define MXC_F_PWRSEQ_LPPWKFL_USBVBUS_POS               2 /**< LPPWKFL_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWKFL_USBVBUS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_USBVBUS_POS)) /**< LPPWKFL_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS                16 /**< LPPWKFL_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWKFL_BACKUP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS)) /**< LPPWKFL_BACKUP Mask */

#define MXC_F_PWRSEQ_LPPWKFL_RSTDET_POS                17 /**< LPPWKFL_RSTDET Position */
#define MXC_F_PWRSEQ_LPPWKFL_RSTDET                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_RSTDET_POS)) /**< LPPWKFL_RSTDET Mask */

/**@} end of group PWRSEQ_LPPWKFL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKEN PWRSEQ_LPPWKEN
 * @brief    Low Power Peripheral Wakeup Enable Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKEN_USBLS_POS                 0 /**< LPPWKEN_USBLS Position */
#define MXC_F_PWRSEQ_LPPWKEN_USBLS                     ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWKEN_USBLS_POS)) /**< LPPWKEN_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWKEN_USBVBUS_POS               2 /**< LPPWKEN_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWKEN_USBVBUS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_USBVBUS_POS)) /**< LPPWKEN_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWKEN_AINCOMP0_POS              4 /**< LPPWKEN_AINCOMP0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_AINCOMP0                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_AINCOMP0_POS)) /**< LPPWKEN_AINCOMP0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_WDT0_POS                  8 /**< LPPWKEN_WDT0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_WDT0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_WDT0_POS)) /**< LPPWKEN_WDT0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_WDT1_POS                  9 /**< LPPWKEN_WDT1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_WDT1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_WDT1_POS)) /**< LPPWKEN_WDT1 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_CPU1_POS                  10 /**< LPPWKEN_CPU1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_CPU1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_CPU1_POS)) /**< LPPWKEN_CPU1 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_TMR0_POS                  11 /**< LPPWKEN_TMR0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_TMR0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_TMR0_POS)) /**< LPPWKEN_TMR0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_TMR1_POS                  12 /**< LPPWKEN_TMR1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_TMR1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_TMR1_POS)) /**< LPPWKEN_TMR1 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_TMR2_POS                  13 /**< LPPWKEN_TMR2 Position */
#define MXC_F_PWRSEQ_LPPWKEN_TMR2                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_TMR2_POS)) /**< LPPWKEN_TMR2 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_TMR3_POS                  14 /**< LPPWKEN_TMR3 Position */
#define MXC_F_PWRSEQ_LPPWKEN_TMR3                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_TMR3_POS)) /**< LPPWKEN_TMR3 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_TMR4_POS                  15 /**< LPPWKEN_TMR4 Position */
#define MXC_F_PWRSEQ_LPPWKEN_TMR4                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_TMR4_POS)) /**< LPPWKEN_TMR4 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_TMR5_POS                  16 /**< LPPWKEN_TMR5 Position */
#define MXC_F_PWRSEQ_LPPWKEN_TMR5                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_TMR5_POS)) /**< LPPWKEN_TMR5 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_UART0_POS                 17 /**< LPPWKEN_UART0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_UART0                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_UART0_POS)) /**< LPPWKEN_UART0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_UART1_POS                 18 /**< LPPWKEN_UART1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_UART1                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_UART1_POS)) /**< LPPWKEN_UART1 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_UART2_POS                 19 /**< LPPWKEN_UART2 Position */
#define MXC_F_PWRSEQ_LPPWKEN_UART2                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_UART2_POS)) /**< LPPWKEN_UART2 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_UART3_POS                 20 /**< LPPWKEN_UART3 Position */
#define MXC_F_PWRSEQ_LPPWKEN_UART3                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_UART3_POS)) /**< LPPWKEN_UART3 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_I2C0_POS                  21 /**< LPPWKEN_I2C0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_I2C0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_I2C0_POS)) /**< LPPWKEN_I2C0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_I2C1_POS                  22 /**< LPPWKEN_I2C1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_I2C1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_I2C1_POS)) /**< LPPWKEN_I2C1 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_I2C2_POS                  23 /**< LPPWKEN_I2C2 Position */
#define MXC_F_PWRSEQ_LPPWKEN_I2C2                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_I2C2_POS)) /**< LPPWKEN_I2C2 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_I2S_POS                   24 /**< LPPWKEN_I2S Position */
#define MXC_F_PWRSEQ_LPPWKEN_I2S                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_I2S_POS)) /**< LPPWKEN_I2S Mask */

#define MXC_F_PWRSEQ_LPPWKEN_SPI1_POS                  25 /**< LPPWKEN_SPI1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_SPI1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_SPI1_POS)) /**< LPPWKEN_SPI1 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_LPCMP_POS                 26 /**< LPPWKEN_LPCMP Position */
#define MXC_F_PWRSEQ_LPPWKEN_LPCMP                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_LPCMP_POS)) /**< LPPWKEN_LPCMP Mask */

/**@} end of group PWRSEQ_LPPWKEN_Register */

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

#define MXC_F_PWRSEQ_LPMEMSD_RAM5_POS                  5 /**< LPMEMSD_RAM5 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM5                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM5_POS)) /**< LPMEMSD_RAM5 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICC_POS                   7 /**< LPMEMSD_ICC Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICC                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICC_POS)) /**< LPMEMSD_ICC Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICCXIP_POS                8 /**< LPMEMSD_ICCXIP Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICCXIP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICCXIP_POS)) /**< LPMEMSD_ICCXIP Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRCC_POS                  9 /**< LPMEMSD_SRCC Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRCC                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRCC_POS)) /**< LPMEMSD_SRCC Mask */

#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS               11 /**< LPMEMSD_USBFIFO Position */
#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS)) /**< LPMEMSD_USBFIFO Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM0_POS                  12 /**< LPMEMSD_ROM0 Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM0_POS)) /**< LPMEMSD_ROM0 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_MEU_POS                   13 /**< LPMEMSD_MEU Position */
#define MXC_F_PWRSEQ_LPMEMSD_MEU                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_MEU_POS)) /**< LPMEMSD_MEU Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM1_POS                  15 /**< LPMEMSD_ROM1 Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM1_POS)) /**< LPMEMSD_ROM1 Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPVDDPD PWRSEQ_LPVDDPD
 * @brief    Low Power VDD Domain Power Down Control.
 * @{
 */
#define MXC_F_PWRSEQ_LPVDDPD_BTLE_POS                  1 /**< LPVDDPD_BTLE Position */
#define MXC_F_PWRSEQ_LPVDDPD_BTLE                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_BTLE_POS)) /**< LPVDDPD_BTLE Mask */

/**@} end of group PWRSEQ_LPVDDPD_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL0A PWRSEQ_LPWKPOL0A
 * @brief    Low Power Wakeup Polarity Select for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL0A_0_POS                   0 /**< LPWKPOL0A_0 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_0                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_0_POS)) /**< LPWKPOL0A_0 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_0_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_0_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_0_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_0_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_0_POS) /**< LPWKPOL0A_0_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_0_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_0_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_0_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_0_RISING << MXC_F_PWRSEQ_LPWKPOL0A_0_POS) /**< LPWKPOL0A_0_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_0_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_0_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_0_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_0_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_0_POS) /**< LPWKPOL0A_0_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_1_POS                   2 /**< LPWKPOL0A_1 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_1                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_1_POS)) /**< LPWKPOL0A_1 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_1_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_1_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_1_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_1_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_1_POS) /**< LPWKPOL0A_1_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_1_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_1_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_1_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_1_RISING << MXC_F_PWRSEQ_LPWKPOL0A_1_POS) /**< LPWKPOL0A_1_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_1_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_1_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_1_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_1_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_1_POS) /**< LPWKPOL0A_1_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_2_POS                   4 /**< LPWKPOL0A_2 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_2                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_2_POS)) /**< LPWKPOL0A_2 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_2_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_2_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_2_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_2_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_2_POS) /**< LPWKPOL0A_2_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_2_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_2_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_2_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_2_RISING << MXC_F_PWRSEQ_LPWKPOL0A_2_POS) /**< LPWKPOL0A_2_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_2_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_2_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_2_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_2_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_2_POS) /**< LPWKPOL0A_2_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_3_POS                   6 /**< LPWKPOL0A_3 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_3                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_3_POS)) /**< LPWKPOL0A_3 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_3_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_3_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_3_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_3_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_3_POS) /**< LPWKPOL0A_3_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_3_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_3_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_3_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_3_RISING << MXC_F_PWRSEQ_LPWKPOL0A_3_POS) /**< LPWKPOL0A_3_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_3_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_3_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_3_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_3_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_3_POS) /**< LPWKPOL0A_3_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_4_POS                   8 /**< LPWKPOL0A_4 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_4                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_4_POS)) /**< LPWKPOL0A_4 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_4_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_4_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_4_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_4_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_4_POS) /**< LPWKPOL0A_4_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_4_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_4_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_4_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_4_RISING << MXC_F_PWRSEQ_LPWKPOL0A_4_POS) /**< LPWKPOL0A_4_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_4_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_4_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_4_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_4_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_4_POS) /**< LPWKPOL0A_4_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_5_POS                   10 /**< LPWKPOL0A_5 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_5                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_5_POS)) /**< LPWKPOL0A_5 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_5_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_5_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_5_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_5_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_5_POS) /**< LPWKPOL0A_5_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_5_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_5_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_5_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_5_RISING << MXC_F_PWRSEQ_LPWKPOL0A_5_POS) /**< LPWKPOL0A_5_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_5_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_5_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_5_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_5_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_5_POS) /**< LPWKPOL0A_5_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_6_POS                   12 /**< LPWKPOL0A_6 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_6                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_6_POS)) /**< LPWKPOL0A_6 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_6_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_6_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_6_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_6_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_6_POS) /**< LPWKPOL0A_6_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_6_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_6_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_6_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_6_RISING << MXC_F_PWRSEQ_LPWKPOL0A_6_POS) /**< LPWKPOL0A_6_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_6_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_6_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_6_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_6_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_6_POS) /**< LPWKPOL0A_6_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_7_POS                   14 /**< LPWKPOL0A_7 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_7                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_7_POS)) /**< LPWKPOL0A_7 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_7_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_7_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_7_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_7_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_7_POS) /**< LPWKPOL0A_7_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_7_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_7_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_7_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_7_RISING << MXC_F_PWRSEQ_LPWKPOL0A_7_POS) /**< LPWKPOL0A_7_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_7_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_7_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_7_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_7_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_7_POS) /**< LPWKPOL0A_7_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_8_POS                   16 /**< LPWKPOL0A_8 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_8                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_8_POS)) /**< LPWKPOL0A_8 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_8_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_8_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_8_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_8_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_8_POS) /**< LPWKPOL0A_8_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_8_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_8_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_8_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_8_RISING << MXC_F_PWRSEQ_LPWKPOL0A_8_POS) /**< LPWKPOL0A_8_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_8_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_8_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_8_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_8_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_8_POS) /**< LPWKPOL0A_8_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_9_POS                   18 /**< LPWKPOL0A_9 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_9                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_9_POS)) /**< LPWKPOL0A_9 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_9_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL0A_9_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_9_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL0A_9_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_9_POS) /**< LPWKPOL0A_9_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_9_RISING                ((uint32_t)0x1UL) /**< LPWKPOL0A_9_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_9_RISING                (MXC_V_PWRSEQ_LPWKPOL0A_9_RISING << MXC_F_PWRSEQ_LPWKPOL0A_9_POS) /**< LPWKPOL0A_9_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_9_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL0A_9_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_9_FALLING               (MXC_V_PWRSEQ_LPWKPOL0A_9_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_9_POS) /**< LPWKPOL0A_9_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_10_POS                  20 /**< LPWKPOL0A_10 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_10                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_10_POS)) /**< LPWKPOL0A_10 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_10_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0A_10_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_10_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0A_10_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_10_POS) /**< LPWKPOL0A_10_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_10_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0A_10_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_10_RISING               (MXC_V_PWRSEQ_LPWKPOL0A_10_RISING << MXC_F_PWRSEQ_LPWKPOL0A_10_POS) /**< LPWKPOL0A_10_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_10_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0A_10_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_10_FALLING              (MXC_V_PWRSEQ_LPWKPOL0A_10_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_10_POS) /**< LPWKPOL0A_10_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_11_POS                  22 /**< LPWKPOL0A_11 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_11                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_11_POS)) /**< LPWKPOL0A_11 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_11_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0A_11_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_11_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0A_11_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_11_POS) /**< LPWKPOL0A_11_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_11_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0A_11_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_11_RISING               (MXC_V_PWRSEQ_LPWKPOL0A_11_RISING << MXC_F_PWRSEQ_LPWKPOL0A_11_POS) /**< LPWKPOL0A_11_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_11_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0A_11_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_11_FALLING              (MXC_V_PWRSEQ_LPWKPOL0A_11_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_11_POS) /**< LPWKPOL0A_11_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_12_POS                  24 /**< LPWKPOL0A_12 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_12                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_12_POS)) /**< LPWKPOL0A_12 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_12_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0A_12_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_12_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0A_12_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_12_POS) /**< LPWKPOL0A_12_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_12_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0A_12_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_12_RISING               (MXC_V_PWRSEQ_LPWKPOL0A_12_RISING << MXC_F_PWRSEQ_LPWKPOL0A_12_POS) /**< LPWKPOL0A_12_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_12_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0A_12_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_12_FALLING              (MXC_V_PWRSEQ_LPWKPOL0A_12_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_12_POS) /**< LPWKPOL0A_12_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_13_POS                  26 /**< LPWKPOL0A_13 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_13                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_13_POS)) /**< LPWKPOL0A_13 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_13_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0A_13_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_13_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0A_13_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_13_POS) /**< LPWKPOL0A_13_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_13_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0A_13_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_13_RISING               (MXC_V_PWRSEQ_LPWKPOL0A_13_RISING << MXC_F_PWRSEQ_LPWKPOL0A_13_POS) /**< LPWKPOL0A_13_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_13_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0A_13_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_13_FALLING              (MXC_V_PWRSEQ_LPWKPOL0A_13_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_13_POS) /**< LPWKPOL0A_13_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_14_POS                  28 /**< LPWKPOL0A_14 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_14                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_14_POS)) /**< LPWKPOL0A_14 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_14_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0A_14_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_14_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0A_14_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_14_POS) /**< LPWKPOL0A_14_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_14_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0A_14_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_14_RISING               (MXC_V_PWRSEQ_LPWKPOL0A_14_RISING << MXC_F_PWRSEQ_LPWKPOL0A_14_POS) /**< LPWKPOL0A_14_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_14_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0A_14_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_14_FALLING              (MXC_V_PWRSEQ_LPWKPOL0A_14_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_14_POS) /**< LPWKPOL0A_14_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0A_15_POS                  30 /**< LPWKPOL0A_15 Position */
#define MXC_F_PWRSEQ_LPWKPOL0A_15                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0A_15_POS)) /**< LPWKPOL0A_15 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0A_15_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0A_15_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_15_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0A_15_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0A_15_POS) /**< LPWKPOL0A_15_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_15_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0A_15_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_15_RISING               (MXC_V_PWRSEQ_LPWKPOL0A_15_RISING << MXC_F_PWRSEQ_LPWKPOL0A_15_POS) /**< LPWKPOL0A_15_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0A_15_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0A_15_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0A_15_FALLING              (MXC_V_PWRSEQ_LPWKPOL0A_15_FALLING << MXC_F_PWRSEQ_LPWKPOL0A_15_POS) /**< LPWKPOL0A_15_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL0A_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL0B PWRSEQ_LPWKPOL0B
 * @brief    Low Power Wakeup Polarity Select for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL0B_16_POS                  0 /**< LPWKPOL0B_16 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_16                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_16_POS)) /**< LPWKPOL0B_16 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_16_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_16_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_16_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_16_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_16_POS) /**< LPWKPOL0B_16_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_16_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_16_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_16_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_16_RISING << MXC_F_PWRSEQ_LPWKPOL0B_16_POS) /**< LPWKPOL0B_16_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_16_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_16_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_16_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_16_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_16_POS) /**< LPWKPOL0B_16_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_17_POS                  2 /**< LPWKPOL0B_17 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_17                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_17_POS)) /**< LPWKPOL0B_17 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_17_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_17_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_17_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_17_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_17_POS) /**< LPWKPOL0B_17_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_17_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_17_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_17_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_17_RISING << MXC_F_PWRSEQ_LPWKPOL0B_17_POS) /**< LPWKPOL0B_17_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_17_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_17_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_17_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_17_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_17_POS) /**< LPWKPOL0B_17_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_18_POS                  4 /**< LPWKPOL0B_18 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_18                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_18_POS)) /**< LPWKPOL0B_18 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_18_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_18_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_18_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_18_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_18_POS) /**< LPWKPOL0B_18_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_18_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_18_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_18_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_18_RISING << MXC_F_PWRSEQ_LPWKPOL0B_18_POS) /**< LPWKPOL0B_18_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_18_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_18_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_18_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_18_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_18_POS) /**< LPWKPOL0B_18_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_19_POS                  6 /**< LPWKPOL0B_19 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_19                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_19_POS)) /**< LPWKPOL0B_19 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_19_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_19_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_19_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_19_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_19_POS) /**< LPWKPOL0B_19_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_19_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_19_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_19_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_19_RISING << MXC_F_PWRSEQ_LPWKPOL0B_19_POS) /**< LPWKPOL0B_19_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_19_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_19_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_19_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_19_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_19_POS) /**< LPWKPOL0B_19_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_20_POS                  8 /**< LPWKPOL0B_20 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_20                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_20_POS)) /**< LPWKPOL0B_20 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_20_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_20_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_20_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_20_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_20_POS) /**< LPWKPOL0B_20_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_20_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_20_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_20_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_20_RISING << MXC_F_PWRSEQ_LPWKPOL0B_20_POS) /**< LPWKPOL0B_20_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_20_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_20_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_20_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_20_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_20_POS) /**< LPWKPOL0B_20_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_21_POS                  10 /**< LPWKPOL0B_21 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_21                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_21_POS)) /**< LPWKPOL0B_21 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_21_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_21_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_21_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_21_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_21_POS) /**< LPWKPOL0B_21_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_21_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_21_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_21_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_21_RISING << MXC_F_PWRSEQ_LPWKPOL0B_21_POS) /**< LPWKPOL0B_21_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_21_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_21_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_21_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_21_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_21_POS) /**< LPWKPOL0B_21_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_22_POS                  12 /**< LPWKPOL0B_22 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_22                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_22_POS)) /**< LPWKPOL0B_22 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_22_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_22_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_22_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_22_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_22_POS) /**< LPWKPOL0B_22_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_22_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_22_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_22_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_22_RISING << MXC_F_PWRSEQ_LPWKPOL0B_22_POS) /**< LPWKPOL0B_22_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_22_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_22_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_22_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_22_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_22_POS) /**< LPWKPOL0B_22_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_23_POS                  14 /**< LPWKPOL0B_23 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_23                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_23_POS)) /**< LPWKPOL0B_23 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_23_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_23_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_23_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_23_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_23_POS) /**< LPWKPOL0B_23_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_23_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_23_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_23_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_23_RISING << MXC_F_PWRSEQ_LPWKPOL0B_23_POS) /**< LPWKPOL0B_23_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_23_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_23_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_23_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_23_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_23_POS) /**< LPWKPOL0B_23_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_24_POS                  16 /**< LPWKPOL0B_24 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_24                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_24_POS)) /**< LPWKPOL0B_24 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_24_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_24_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_24_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_24_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_24_POS) /**< LPWKPOL0B_24_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_24_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_24_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_24_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_24_RISING << MXC_F_PWRSEQ_LPWKPOL0B_24_POS) /**< LPWKPOL0B_24_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_24_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_24_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_24_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_24_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_24_POS) /**< LPWKPOL0B_24_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_25_POS                  18 /**< LPWKPOL0B_25 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_25                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_25_POS)) /**< LPWKPOL0B_25 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_25_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_25_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_25_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_25_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_25_POS) /**< LPWKPOL0B_25_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_25_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_25_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_25_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_25_RISING << MXC_F_PWRSEQ_LPWKPOL0B_25_POS) /**< LPWKPOL0B_25_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_25_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_25_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_25_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_25_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_25_POS) /**< LPWKPOL0B_25_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_26_POS                  20 /**< LPWKPOL0B_26 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_26                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_26_POS)) /**< LPWKPOL0B_26 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_26_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_26_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_26_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_26_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_26_POS) /**< LPWKPOL0B_26_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_26_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_26_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_26_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_26_RISING << MXC_F_PWRSEQ_LPWKPOL0B_26_POS) /**< LPWKPOL0B_26_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_26_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_26_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_26_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_26_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_26_POS) /**< LPWKPOL0B_26_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_27_POS                  22 /**< LPWKPOL0B_27 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_27                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_27_POS)) /**< LPWKPOL0B_27 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_27_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_27_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_27_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_27_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_27_POS) /**< LPWKPOL0B_27_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_27_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_27_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_27_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_27_RISING << MXC_F_PWRSEQ_LPWKPOL0B_27_POS) /**< LPWKPOL0B_27_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_27_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_27_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_27_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_27_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_27_POS) /**< LPWKPOL0B_27_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_28_POS                  24 /**< LPWKPOL0B_28 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_28                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_28_POS)) /**< LPWKPOL0B_28 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_28_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_28_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_28_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_28_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_28_POS) /**< LPWKPOL0B_28_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_28_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_28_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_28_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_28_RISING << MXC_F_PWRSEQ_LPWKPOL0B_28_POS) /**< LPWKPOL0B_28_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_28_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_28_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_28_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_28_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_28_POS) /**< LPWKPOL0B_28_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_29_POS                  26 /**< LPWKPOL0B_29 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_29                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_29_POS)) /**< LPWKPOL0B_29 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_29_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_29_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_29_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_29_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_29_POS) /**< LPWKPOL0B_29_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_29_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_29_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_29_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_29_RISING << MXC_F_PWRSEQ_LPWKPOL0B_29_POS) /**< LPWKPOL0B_29_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_29_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_29_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_29_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_29_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_29_POS) /**< LPWKPOL0B_29_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_30_POS                  28 /**< LPWKPOL0B_30 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_30                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_30_POS)) /**< LPWKPOL0B_30 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_30_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_30_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_30_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_30_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_30_POS) /**< LPWKPOL0B_30_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_30_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_30_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_30_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_30_RISING << MXC_F_PWRSEQ_LPWKPOL0B_30_POS) /**< LPWKPOL0B_30_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_30_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_30_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_30_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_30_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_30_POS) /**< LPWKPOL0B_30_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL0B_31_POS                  30 /**< LPWKPOL0B_31 Position */
#define MXC_F_PWRSEQ_LPWKPOL0B_31                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL0B_31_POS)) /**< LPWKPOL0B_31 Mask */
#define MXC_V_PWRSEQ_LPWKPOL0B_31_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL0B_31_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_31_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL0B_31_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL0B_31_POS) /**< LPWKPOL0B_31_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_31_RISING               ((uint32_t)0x1UL) /**< LPWKPOL0B_31_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_31_RISING               (MXC_V_PWRSEQ_LPWKPOL0B_31_RISING << MXC_F_PWRSEQ_LPWKPOL0B_31_POS) /**< LPWKPOL0B_31_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL0B_31_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL0B_31_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL0B_31_FALLING              (MXC_V_PWRSEQ_LPWKPOL0B_31_FALLING << MXC_F_PWRSEQ_LPWKPOL0B_31_POS) /**< LPWKPOL0B_31_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL0B_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL1A PWRSEQ_LPWKPOL1A
 * @brief    Low Power Wakeup Polarity Select for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL1A_0_POS                   0 /**< LPWKPOL1A_0 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_0                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_0_POS)) /**< LPWKPOL1A_0 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_0_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_0_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_0_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_0_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_0_POS) /**< LPWKPOL1A_0_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_0_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_0_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_0_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_0_RISING << MXC_F_PWRSEQ_LPWKPOL1A_0_POS) /**< LPWKPOL1A_0_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_0_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_0_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_0_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_0_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_0_POS) /**< LPWKPOL1A_0_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_1_POS                   2 /**< LPWKPOL1A_1 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_1                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_1_POS)) /**< LPWKPOL1A_1 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_1_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_1_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_1_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_1_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_1_POS) /**< LPWKPOL1A_1_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_1_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_1_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_1_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_1_RISING << MXC_F_PWRSEQ_LPWKPOL1A_1_POS) /**< LPWKPOL1A_1_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_1_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_1_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_1_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_1_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_1_POS) /**< LPWKPOL1A_1_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_2_POS                   4 /**< LPWKPOL1A_2 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_2                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_2_POS)) /**< LPWKPOL1A_2 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_2_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_2_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_2_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_2_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_2_POS) /**< LPWKPOL1A_2_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_2_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_2_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_2_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_2_RISING << MXC_F_PWRSEQ_LPWKPOL1A_2_POS) /**< LPWKPOL1A_2_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_2_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_2_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_2_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_2_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_2_POS) /**< LPWKPOL1A_2_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_3_POS                   6 /**< LPWKPOL1A_3 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_3                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_3_POS)) /**< LPWKPOL1A_3 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_3_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_3_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_3_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_3_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_3_POS) /**< LPWKPOL1A_3_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_3_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_3_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_3_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_3_RISING << MXC_F_PWRSEQ_LPWKPOL1A_3_POS) /**< LPWKPOL1A_3_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_3_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_3_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_3_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_3_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_3_POS) /**< LPWKPOL1A_3_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_4_POS                   8 /**< LPWKPOL1A_4 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_4                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_4_POS)) /**< LPWKPOL1A_4 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_4_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_4_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_4_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_4_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_4_POS) /**< LPWKPOL1A_4_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_4_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_4_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_4_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_4_RISING << MXC_F_PWRSEQ_LPWKPOL1A_4_POS) /**< LPWKPOL1A_4_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_4_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_4_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_4_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_4_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_4_POS) /**< LPWKPOL1A_4_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_5_POS                   10 /**< LPWKPOL1A_5 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_5                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_5_POS)) /**< LPWKPOL1A_5 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_5_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_5_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_5_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_5_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_5_POS) /**< LPWKPOL1A_5_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_5_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_5_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_5_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_5_RISING << MXC_F_PWRSEQ_LPWKPOL1A_5_POS) /**< LPWKPOL1A_5_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_5_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_5_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_5_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_5_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_5_POS) /**< LPWKPOL1A_5_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_6_POS                   12 /**< LPWKPOL1A_6 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_6                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_6_POS)) /**< LPWKPOL1A_6 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_6_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_6_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_6_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_6_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_6_POS) /**< LPWKPOL1A_6_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_6_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_6_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_6_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_6_RISING << MXC_F_PWRSEQ_LPWKPOL1A_6_POS) /**< LPWKPOL1A_6_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_6_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_6_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_6_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_6_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_6_POS) /**< LPWKPOL1A_6_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_7_POS                   14 /**< LPWKPOL1A_7 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_7                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_7_POS)) /**< LPWKPOL1A_7 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_7_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_7_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_7_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_7_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_7_POS) /**< LPWKPOL1A_7_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_7_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_7_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_7_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_7_RISING << MXC_F_PWRSEQ_LPWKPOL1A_7_POS) /**< LPWKPOL1A_7_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_7_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_7_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_7_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_7_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_7_POS) /**< LPWKPOL1A_7_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_8_POS                   16 /**< LPWKPOL1A_8 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_8                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_8_POS)) /**< LPWKPOL1A_8 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_8_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_8_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_8_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_8_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_8_POS) /**< LPWKPOL1A_8_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_8_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_8_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_8_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_8_RISING << MXC_F_PWRSEQ_LPWKPOL1A_8_POS) /**< LPWKPOL1A_8_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_8_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_8_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_8_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_8_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_8_POS) /**< LPWKPOL1A_8_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_9_POS                   18 /**< LPWKPOL1A_9 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_9                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_9_POS)) /**< LPWKPOL1A_9 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_9_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL1A_9_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_9_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL1A_9_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_9_POS) /**< LPWKPOL1A_9_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_9_RISING                ((uint32_t)0x1UL) /**< LPWKPOL1A_9_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_9_RISING                (MXC_V_PWRSEQ_LPWKPOL1A_9_RISING << MXC_F_PWRSEQ_LPWKPOL1A_9_POS) /**< LPWKPOL1A_9_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_9_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL1A_9_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_9_FALLING               (MXC_V_PWRSEQ_LPWKPOL1A_9_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_9_POS) /**< LPWKPOL1A_9_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_10_POS                  20 /**< LPWKPOL1A_10 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_10                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_10_POS)) /**< LPWKPOL1A_10 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_10_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1A_10_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_10_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1A_10_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_10_POS) /**< LPWKPOL1A_10_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_10_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1A_10_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_10_RISING               (MXC_V_PWRSEQ_LPWKPOL1A_10_RISING << MXC_F_PWRSEQ_LPWKPOL1A_10_POS) /**< LPWKPOL1A_10_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_10_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1A_10_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_10_FALLING              (MXC_V_PWRSEQ_LPWKPOL1A_10_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_10_POS) /**< LPWKPOL1A_10_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_11_POS                  22 /**< LPWKPOL1A_11 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_11                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_11_POS)) /**< LPWKPOL1A_11 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_11_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1A_11_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_11_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1A_11_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_11_POS) /**< LPWKPOL1A_11_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_11_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1A_11_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_11_RISING               (MXC_V_PWRSEQ_LPWKPOL1A_11_RISING << MXC_F_PWRSEQ_LPWKPOL1A_11_POS) /**< LPWKPOL1A_11_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_11_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1A_11_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_11_FALLING              (MXC_V_PWRSEQ_LPWKPOL1A_11_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_11_POS) /**< LPWKPOL1A_11_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_12_POS                  24 /**< LPWKPOL1A_12 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_12                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_12_POS)) /**< LPWKPOL1A_12 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_12_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1A_12_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_12_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1A_12_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_12_POS) /**< LPWKPOL1A_12_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_12_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1A_12_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_12_RISING               (MXC_V_PWRSEQ_LPWKPOL1A_12_RISING << MXC_F_PWRSEQ_LPWKPOL1A_12_POS) /**< LPWKPOL1A_12_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_12_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1A_12_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_12_FALLING              (MXC_V_PWRSEQ_LPWKPOL1A_12_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_12_POS) /**< LPWKPOL1A_12_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_13_POS                  26 /**< LPWKPOL1A_13 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_13                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_13_POS)) /**< LPWKPOL1A_13 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_13_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1A_13_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_13_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1A_13_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_13_POS) /**< LPWKPOL1A_13_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_13_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1A_13_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_13_RISING               (MXC_V_PWRSEQ_LPWKPOL1A_13_RISING << MXC_F_PWRSEQ_LPWKPOL1A_13_POS) /**< LPWKPOL1A_13_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_13_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1A_13_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_13_FALLING              (MXC_V_PWRSEQ_LPWKPOL1A_13_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_13_POS) /**< LPWKPOL1A_13_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_14_POS                  28 /**< LPWKPOL1A_14 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_14                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_14_POS)) /**< LPWKPOL1A_14 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_14_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1A_14_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_14_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1A_14_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_14_POS) /**< LPWKPOL1A_14_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_14_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1A_14_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_14_RISING               (MXC_V_PWRSEQ_LPWKPOL1A_14_RISING << MXC_F_PWRSEQ_LPWKPOL1A_14_POS) /**< LPWKPOL1A_14_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_14_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1A_14_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_14_FALLING              (MXC_V_PWRSEQ_LPWKPOL1A_14_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_14_POS) /**< LPWKPOL1A_14_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1A_15_POS                  30 /**< LPWKPOL1A_15 Position */
#define MXC_F_PWRSEQ_LPWKPOL1A_15                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1A_15_POS)) /**< LPWKPOL1A_15 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1A_15_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1A_15_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_15_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1A_15_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1A_15_POS) /**< LPWKPOL1A_15_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_15_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1A_15_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_15_RISING               (MXC_V_PWRSEQ_LPWKPOL1A_15_RISING << MXC_F_PWRSEQ_LPWKPOL1A_15_POS) /**< LPWKPOL1A_15_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1A_15_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1A_15_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1A_15_FALLING              (MXC_V_PWRSEQ_LPWKPOL1A_15_FALLING << MXC_F_PWRSEQ_LPWKPOL1A_15_POS) /**< LPWKPOL1A_15_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL1A_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL1B PWRSEQ_LPWKPOL1B
 * @brief    Low Power Wakeup Polarity Select for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL1B_16_POS                  0 /**< LPWKPOL1B_16 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_16                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_16_POS)) /**< LPWKPOL1B_16 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_16_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_16_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_16_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_16_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_16_POS) /**< LPWKPOL1B_16_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_16_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_16_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_16_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_16_RISING << MXC_F_PWRSEQ_LPWKPOL1B_16_POS) /**< LPWKPOL1B_16_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_16_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_16_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_16_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_16_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_16_POS) /**< LPWKPOL1B_16_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_17_POS                  2 /**< LPWKPOL1B_17 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_17                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_17_POS)) /**< LPWKPOL1B_17 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_17_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_17_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_17_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_17_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_17_POS) /**< LPWKPOL1B_17_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_17_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_17_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_17_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_17_RISING << MXC_F_PWRSEQ_LPWKPOL1B_17_POS) /**< LPWKPOL1B_17_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_17_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_17_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_17_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_17_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_17_POS) /**< LPWKPOL1B_17_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_18_POS                  4 /**< LPWKPOL1B_18 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_18                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_18_POS)) /**< LPWKPOL1B_18 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_18_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_18_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_18_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_18_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_18_POS) /**< LPWKPOL1B_18_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_18_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_18_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_18_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_18_RISING << MXC_F_PWRSEQ_LPWKPOL1B_18_POS) /**< LPWKPOL1B_18_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_18_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_18_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_18_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_18_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_18_POS) /**< LPWKPOL1B_18_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_19_POS                  6 /**< LPWKPOL1B_19 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_19                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_19_POS)) /**< LPWKPOL1B_19 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_19_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_19_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_19_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_19_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_19_POS) /**< LPWKPOL1B_19_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_19_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_19_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_19_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_19_RISING << MXC_F_PWRSEQ_LPWKPOL1B_19_POS) /**< LPWKPOL1B_19_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_19_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_19_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_19_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_19_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_19_POS) /**< LPWKPOL1B_19_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_20_POS                  8 /**< LPWKPOL1B_20 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_20                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_20_POS)) /**< LPWKPOL1B_20 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_20_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_20_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_20_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_20_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_20_POS) /**< LPWKPOL1B_20_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_20_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_20_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_20_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_20_RISING << MXC_F_PWRSEQ_LPWKPOL1B_20_POS) /**< LPWKPOL1B_20_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_20_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_20_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_20_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_20_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_20_POS) /**< LPWKPOL1B_20_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_21_POS                  10 /**< LPWKPOL1B_21 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_21                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_21_POS)) /**< LPWKPOL1B_21 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_21_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_21_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_21_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_21_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_21_POS) /**< LPWKPOL1B_21_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_21_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_21_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_21_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_21_RISING << MXC_F_PWRSEQ_LPWKPOL1B_21_POS) /**< LPWKPOL1B_21_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_21_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_21_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_21_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_21_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_21_POS) /**< LPWKPOL1B_21_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_22_POS                  12 /**< LPWKPOL1B_22 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_22                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_22_POS)) /**< LPWKPOL1B_22 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_22_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_22_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_22_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_22_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_22_POS) /**< LPWKPOL1B_22_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_22_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_22_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_22_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_22_RISING << MXC_F_PWRSEQ_LPWKPOL1B_22_POS) /**< LPWKPOL1B_22_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_22_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_22_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_22_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_22_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_22_POS) /**< LPWKPOL1B_22_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_23_POS                  14 /**< LPWKPOL1B_23 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_23                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_23_POS)) /**< LPWKPOL1B_23 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_23_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_23_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_23_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_23_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_23_POS) /**< LPWKPOL1B_23_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_23_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_23_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_23_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_23_RISING << MXC_F_PWRSEQ_LPWKPOL1B_23_POS) /**< LPWKPOL1B_23_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_23_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_23_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_23_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_23_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_23_POS) /**< LPWKPOL1B_23_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_24_POS                  16 /**< LPWKPOL1B_24 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_24                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_24_POS)) /**< LPWKPOL1B_24 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_24_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_24_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_24_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_24_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_24_POS) /**< LPWKPOL1B_24_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_24_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_24_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_24_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_24_RISING << MXC_F_PWRSEQ_LPWKPOL1B_24_POS) /**< LPWKPOL1B_24_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_24_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_24_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_24_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_24_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_24_POS) /**< LPWKPOL1B_24_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_25_POS                  18 /**< LPWKPOL1B_25 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_25                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_25_POS)) /**< LPWKPOL1B_25 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_25_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_25_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_25_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_25_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_25_POS) /**< LPWKPOL1B_25_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_25_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_25_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_25_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_25_RISING << MXC_F_PWRSEQ_LPWKPOL1B_25_POS) /**< LPWKPOL1B_25_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_25_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_25_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_25_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_25_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_25_POS) /**< LPWKPOL1B_25_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_26_POS                  20 /**< LPWKPOL1B_26 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_26                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_26_POS)) /**< LPWKPOL1B_26 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_26_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_26_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_26_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_26_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_26_POS) /**< LPWKPOL1B_26_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_26_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_26_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_26_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_26_RISING << MXC_F_PWRSEQ_LPWKPOL1B_26_POS) /**< LPWKPOL1B_26_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_26_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_26_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_26_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_26_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_26_POS) /**< LPWKPOL1B_26_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_27_POS                  22 /**< LPWKPOL1B_27 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_27                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_27_POS)) /**< LPWKPOL1B_27 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_27_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_27_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_27_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_27_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_27_POS) /**< LPWKPOL1B_27_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_27_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_27_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_27_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_27_RISING << MXC_F_PWRSEQ_LPWKPOL1B_27_POS) /**< LPWKPOL1B_27_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_27_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_27_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_27_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_27_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_27_POS) /**< LPWKPOL1B_27_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_28_POS                  24 /**< LPWKPOL1B_28 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_28                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_28_POS)) /**< LPWKPOL1B_28 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_28_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_28_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_28_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_28_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_28_POS) /**< LPWKPOL1B_28_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_28_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_28_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_28_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_28_RISING << MXC_F_PWRSEQ_LPWKPOL1B_28_POS) /**< LPWKPOL1B_28_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_28_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_28_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_28_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_28_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_28_POS) /**< LPWKPOL1B_28_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_29_POS                  26 /**< LPWKPOL1B_29 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_29                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_29_POS)) /**< LPWKPOL1B_29 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_29_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_29_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_29_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_29_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_29_POS) /**< LPWKPOL1B_29_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_29_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_29_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_29_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_29_RISING << MXC_F_PWRSEQ_LPWKPOL1B_29_POS) /**< LPWKPOL1B_29_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_29_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_29_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_29_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_29_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_29_POS) /**< LPWKPOL1B_29_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_30_POS                  28 /**< LPWKPOL1B_30 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_30                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_30_POS)) /**< LPWKPOL1B_30 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_30_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_30_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_30_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_30_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_30_POS) /**< LPWKPOL1B_30_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_30_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_30_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_30_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_30_RISING << MXC_F_PWRSEQ_LPWKPOL1B_30_POS) /**< LPWKPOL1B_30_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_30_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_30_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_30_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_30_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_30_POS) /**< LPWKPOL1B_30_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL1B_31_POS                  30 /**< LPWKPOL1B_31 Position */
#define MXC_F_PWRSEQ_LPWKPOL1B_31                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL1B_31_POS)) /**< LPWKPOL1B_31 Mask */
#define MXC_V_PWRSEQ_LPWKPOL1B_31_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL1B_31_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_31_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL1B_31_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL1B_31_POS) /**< LPWKPOL1B_31_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_31_RISING               ((uint32_t)0x1UL) /**< LPWKPOL1B_31_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_31_RISING               (MXC_V_PWRSEQ_LPWKPOL1B_31_RISING << MXC_F_PWRSEQ_LPWKPOL1B_31_POS) /**< LPWKPOL1B_31_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL1B_31_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL1B_31_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL1B_31_FALLING              (MXC_V_PWRSEQ_LPWKPOL1B_31_FALLING << MXC_F_PWRSEQ_LPWKPOL1B_31_POS) /**< LPWKPOL1B_31_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL1B_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL2A PWRSEQ_LPWKPOL2A
 * @brief    Low Power Wakeup Polarity Select for GPIO2.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL2A_0_POS                   0 /**< LPWKPOL2A_0 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_0                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_0_POS)) /**< LPWKPOL2A_0 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_0_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_0_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_0_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_0_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_0_POS) /**< LPWKPOL2A_0_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_0_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_0_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_0_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_0_RISING << MXC_F_PWRSEQ_LPWKPOL2A_0_POS) /**< LPWKPOL2A_0_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_0_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_0_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_0_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_0_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_0_POS) /**< LPWKPOL2A_0_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_1_POS                   2 /**< LPWKPOL2A_1 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_1                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_1_POS)) /**< LPWKPOL2A_1 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_1_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_1_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_1_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_1_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_1_POS) /**< LPWKPOL2A_1_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_1_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_1_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_1_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_1_RISING << MXC_F_PWRSEQ_LPWKPOL2A_1_POS) /**< LPWKPOL2A_1_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_1_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_1_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_1_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_1_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_1_POS) /**< LPWKPOL2A_1_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_2_POS                   4 /**< LPWKPOL2A_2 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_2                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_2_POS)) /**< LPWKPOL2A_2 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_2_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_2_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_2_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_2_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_2_POS) /**< LPWKPOL2A_2_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_2_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_2_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_2_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_2_RISING << MXC_F_PWRSEQ_LPWKPOL2A_2_POS) /**< LPWKPOL2A_2_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_2_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_2_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_2_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_2_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_2_POS) /**< LPWKPOL2A_2_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_3_POS                   6 /**< LPWKPOL2A_3 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_3                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_3_POS)) /**< LPWKPOL2A_3 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_3_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_3_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_3_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_3_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_3_POS) /**< LPWKPOL2A_3_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_3_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_3_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_3_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_3_RISING << MXC_F_PWRSEQ_LPWKPOL2A_3_POS) /**< LPWKPOL2A_3_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_3_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_3_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_3_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_3_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_3_POS) /**< LPWKPOL2A_3_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_4_POS                   8 /**< LPWKPOL2A_4 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_4                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_4_POS)) /**< LPWKPOL2A_4 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_4_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_4_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_4_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_4_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_4_POS) /**< LPWKPOL2A_4_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_4_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_4_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_4_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_4_RISING << MXC_F_PWRSEQ_LPWKPOL2A_4_POS) /**< LPWKPOL2A_4_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_4_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_4_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_4_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_4_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_4_POS) /**< LPWKPOL2A_4_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_5_POS                   10 /**< LPWKPOL2A_5 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_5                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_5_POS)) /**< LPWKPOL2A_5 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_5_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_5_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_5_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_5_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_5_POS) /**< LPWKPOL2A_5_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_5_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_5_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_5_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_5_RISING << MXC_F_PWRSEQ_LPWKPOL2A_5_POS) /**< LPWKPOL2A_5_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_5_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_5_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_5_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_5_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_5_POS) /**< LPWKPOL2A_5_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_6_POS                   12 /**< LPWKPOL2A_6 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_6                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_6_POS)) /**< LPWKPOL2A_6 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_6_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_6_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_6_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_6_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_6_POS) /**< LPWKPOL2A_6_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_6_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_6_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_6_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_6_RISING << MXC_F_PWRSEQ_LPWKPOL2A_6_POS) /**< LPWKPOL2A_6_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_6_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_6_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_6_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_6_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_6_POS) /**< LPWKPOL2A_6_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_7_POS                   14 /**< LPWKPOL2A_7 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_7                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_7_POS)) /**< LPWKPOL2A_7 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_7_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_7_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_7_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_7_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_7_POS) /**< LPWKPOL2A_7_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_7_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_7_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_7_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_7_RISING << MXC_F_PWRSEQ_LPWKPOL2A_7_POS) /**< LPWKPOL2A_7_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_7_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_7_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_7_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_7_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_7_POS) /**< LPWKPOL2A_7_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_8_POS                   16 /**< LPWKPOL2A_8 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_8                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_8_POS)) /**< LPWKPOL2A_8 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_8_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_8_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_8_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_8_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_8_POS) /**< LPWKPOL2A_8_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_8_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_8_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_8_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_8_RISING << MXC_F_PWRSEQ_LPWKPOL2A_8_POS) /**< LPWKPOL2A_8_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_8_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_8_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_8_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_8_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_8_POS) /**< LPWKPOL2A_8_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_9_POS                   18 /**< LPWKPOL2A_9 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_9                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_9_POS)) /**< LPWKPOL2A_9 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_9_BOTH_EDGES            ((uint32_t)0x0UL) /**< LPWKPOL2A_9_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_9_BOTH_EDGES            (MXC_V_PWRSEQ_LPWKPOL2A_9_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_9_POS) /**< LPWKPOL2A_9_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_9_RISING                ((uint32_t)0x1UL) /**< LPWKPOL2A_9_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_9_RISING                (MXC_V_PWRSEQ_LPWKPOL2A_9_RISING << MXC_F_PWRSEQ_LPWKPOL2A_9_POS) /**< LPWKPOL2A_9_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_9_FALLING               ((uint32_t)0x2UL) /**< LPWKPOL2A_9_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_9_FALLING               (MXC_V_PWRSEQ_LPWKPOL2A_9_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_9_POS) /**< LPWKPOL2A_9_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_10_POS                  20 /**< LPWKPOL2A_10 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_10                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_10_POS)) /**< LPWKPOL2A_10 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_10_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2A_10_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_10_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2A_10_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_10_POS) /**< LPWKPOL2A_10_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_10_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2A_10_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_10_RISING               (MXC_V_PWRSEQ_LPWKPOL2A_10_RISING << MXC_F_PWRSEQ_LPWKPOL2A_10_POS) /**< LPWKPOL2A_10_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_10_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2A_10_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_10_FALLING              (MXC_V_PWRSEQ_LPWKPOL2A_10_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_10_POS) /**< LPWKPOL2A_10_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_11_POS                  22 /**< LPWKPOL2A_11 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_11                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_11_POS)) /**< LPWKPOL2A_11 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_11_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2A_11_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_11_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2A_11_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_11_POS) /**< LPWKPOL2A_11_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_11_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2A_11_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_11_RISING               (MXC_V_PWRSEQ_LPWKPOL2A_11_RISING << MXC_F_PWRSEQ_LPWKPOL2A_11_POS) /**< LPWKPOL2A_11_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_11_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2A_11_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_11_FALLING              (MXC_V_PWRSEQ_LPWKPOL2A_11_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_11_POS) /**< LPWKPOL2A_11_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_12_POS                  24 /**< LPWKPOL2A_12 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_12                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_12_POS)) /**< LPWKPOL2A_12 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_12_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2A_12_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_12_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2A_12_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_12_POS) /**< LPWKPOL2A_12_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_12_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2A_12_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_12_RISING               (MXC_V_PWRSEQ_LPWKPOL2A_12_RISING << MXC_F_PWRSEQ_LPWKPOL2A_12_POS) /**< LPWKPOL2A_12_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_12_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2A_12_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_12_FALLING              (MXC_V_PWRSEQ_LPWKPOL2A_12_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_12_POS) /**< LPWKPOL2A_12_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_13_POS                  26 /**< LPWKPOL2A_13 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_13                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_13_POS)) /**< LPWKPOL2A_13 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_13_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2A_13_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_13_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2A_13_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_13_POS) /**< LPWKPOL2A_13_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_13_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2A_13_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_13_RISING               (MXC_V_PWRSEQ_LPWKPOL2A_13_RISING << MXC_F_PWRSEQ_LPWKPOL2A_13_POS) /**< LPWKPOL2A_13_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_13_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2A_13_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_13_FALLING              (MXC_V_PWRSEQ_LPWKPOL2A_13_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_13_POS) /**< LPWKPOL2A_13_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_14_POS                  28 /**< LPWKPOL2A_14 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_14                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_14_POS)) /**< LPWKPOL2A_14 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_14_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2A_14_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_14_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2A_14_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_14_POS) /**< LPWKPOL2A_14_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_14_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2A_14_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_14_RISING               (MXC_V_PWRSEQ_LPWKPOL2A_14_RISING << MXC_F_PWRSEQ_LPWKPOL2A_14_POS) /**< LPWKPOL2A_14_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_14_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2A_14_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_14_FALLING              (MXC_V_PWRSEQ_LPWKPOL2A_14_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_14_POS) /**< LPWKPOL2A_14_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2A_15_POS                  30 /**< LPWKPOL2A_15 Position */
#define MXC_F_PWRSEQ_LPWKPOL2A_15                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2A_15_POS)) /**< LPWKPOL2A_15 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2A_15_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2A_15_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_15_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2A_15_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2A_15_POS) /**< LPWKPOL2A_15_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_15_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2A_15_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_15_RISING               (MXC_V_PWRSEQ_LPWKPOL2A_15_RISING << MXC_F_PWRSEQ_LPWKPOL2A_15_POS) /**< LPWKPOL2A_15_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2A_15_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2A_15_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2A_15_FALLING              (MXC_V_PWRSEQ_LPWKPOL2A_15_FALLING << MXC_F_PWRSEQ_LPWKPOL2A_15_POS) /**< LPWKPOL2A_15_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL2A_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL2B PWRSEQ_LPWKPOL2B
 * @brief    Low Power Wakeup Polarity Select for GPIO2.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL2B_16_POS                  0 /**< LPWKPOL2B_16 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_16                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_16_POS)) /**< LPWKPOL2B_16 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_16_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_16_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_16_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_16_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_16_POS) /**< LPWKPOL2B_16_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_16_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_16_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_16_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_16_RISING << MXC_F_PWRSEQ_LPWKPOL2B_16_POS) /**< LPWKPOL2B_16_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_16_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_16_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_16_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_16_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_16_POS) /**< LPWKPOL2B_16_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_17_POS                  2 /**< LPWKPOL2B_17 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_17                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_17_POS)) /**< LPWKPOL2B_17 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_17_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_17_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_17_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_17_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_17_POS) /**< LPWKPOL2B_17_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_17_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_17_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_17_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_17_RISING << MXC_F_PWRSEQ_LPWKPOL2B_17_POS) /**< LPWKPOL2B_17_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_17_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_17_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_17_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_17_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_17_POS) /**< LPWKPOL2B_17_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_18_POS                  4 /**< LPWKPOL2B_18 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_18                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_18_POS)) /**< LPWKPOL2B_18 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_18_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_18_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_18_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_18_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_18_POS) /**< LPWKPOL2B_18_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_18_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_18_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_18_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_18_RISING << MXC_F_PWRSEQ_LPWKPOL2B_18_POS) /**< LPWKPOL2B_18_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_18_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_18_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_18_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_18_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_18_POS) /**< LPWKPOL2B_18_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_19_POS                  6 /**< LPWKPOL2B_19 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_19                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_19_POS)) /**< LPWKPOL2B_19 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_19_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_19_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_19_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_19_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_19_POS) /**< LPWKPOL2B_19_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_19_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_19_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_19_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_19_RISING << MXC_F_PWRSEQ_LPWKPOL2B_19_POS) /**< LPWKPOL2B_19_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_19_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_19_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_19_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_19_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_19_POS) /**< LPWKPOL2B_19_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_20_POS                  8 /**< LPWKPOL2B_20 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_20                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_20_POS)) /**< LPWKPOL2B_20 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_20_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_20_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_20_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_20_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_20_POS) /**< LPWKPOL2B_20_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_20_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_20_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_20_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_20_RISING << MXC_F_PWRSEQ_LPWKPOL2B_20_POS) /**< LPWKPOL2B_20_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_20_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_20_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_20_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_20_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_20_POS) /**< LPWKPOL2B_20_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_21_POS                  10 /**< LPWKPOL2B_21 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_21                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_21_POS)) /**< LPWKPOL2B_21 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_21_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_21_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_21_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_21_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_21_POS) /**< LPWKPOL2B_21_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_21_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_21_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_21_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_21_RISING << MXC_F_PWRSEQ_LPWKPOL2B_21_POS) /**< LPWKPOL2B_21_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_21_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_21_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_21_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_21_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_21_POS) /**< LPWKPOL2B_21_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_22_POS                  12 /**< LPWKPOL2B_22 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_22                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_22_POS)) /**< LPWKPOL2B_22 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_22_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_22_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_22_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_22_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_22_POS) /**< LPWKPOL2B_22_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_22_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_22_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_22_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_22_RISING << MXC_F_PWRSEQ_LPWKPOL2B_22_POS) /**< LPWKPOL2B_22_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_22_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_22_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_22_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_22_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_22_POS) /**< LPWKPOL2B_22_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_23_POS                  14 /**< LPWKPOL2B_23 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_23                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_23_POS)) /**< LPWKPOL2B_23 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_23_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_23_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_23_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_23_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_23_POS) /**< LPWKPOL2B_23_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_23_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_23_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_23_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_23_RISING << MXC_F_PWRSEQ_LPWKPOL2B_23_POS) /**< LPWKPOL2B_23_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_23_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_23_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_23_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_23_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_23_POS) /**< LPWKPOL2B_23_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_24_POS                  16 /**< LPWKPOL2B_24 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_24                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_24_POS)) /**< LPWKPOL2B_24 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_24_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_24_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_24_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_24_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_24_POS) /**< LPWKPOL2B_24_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_24_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_24_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_24_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_24_RISING << MXC_F_PWRSEQ_LPWKPOL2B_24_POS) /**< LPWKPOL2B_24_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_24_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_24_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_24_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_24_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_24_POS) /**< LPWKPOL2B_24_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_25_POS                  18 /**< LPWKPOL2B_25 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_25                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_25_POS)) /**< LPWKPOL2B_25 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_25_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_25_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_25_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_25_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_25_POS) /**< LPWKPOL2B_25_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_25_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_25_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_25_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_25_RISING << MXC_F_PWRSEQ_LPWKPOL2B_25_POS) /**< LPWKPOL2B_25_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_25_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_25_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_25_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_25_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_25_POS) /**< LPWKPOL2B_25_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_26_POS                  20 /**< LPWKPOL2B_26 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_26                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_26_POS)) /**< LPWKPOL2B_26 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_26_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_26_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_26_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_26_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_26_POS) /**< LPWKPOL2B_26_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_26_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_26_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_26_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_26_RISING << MXC_F_PWRSEQ_LPWKPOL2B_26_POS) /**< LPWKPOL2B_26_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_26_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_26_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_26_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_26_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_26_POS) /**< LPWKPOL2B_26_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_27_POS                  22 /**< LPWKPOL2B_27 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_27                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_27_POS)) /**< LPWKPOL2B_27 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_27_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_27_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_27_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_27_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_27_POS) /**< LPWKPOL2B_27_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_27_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_27_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_27_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_27_RISING << MXC_F_PWRSEQ_LPWKPOL2B_27_POS) /**< LPWKPOL2B_27_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_27_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_27_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_27_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_27_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_27_POS) /**< LPWKPOL2B_27_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_28_POS                  24 /**< LPWKPOL2B_28 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_28                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_28_POS)) /**< LPWKPOL2B_28 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_28_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_28_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_28_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_28_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_28_POS) /**< LPWKPOL2B_28_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_28_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_28_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_28_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_28_RISING << MXC_F_PWRSEQ_LPWKPOL2B_28_POS) /**< LPWKPOL2B_28_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_28_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_28_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_28_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_28_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_28_POS) /**< LPWKPOL2B_28_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_29_POS                  26 /**< LPWKPOL2B_29 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_29                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_29_POS)) /**< LPWKPOL2B_29 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_29_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_29_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_29_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_29_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_29_POS) /**< LPWKPOL2B_29_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_29_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_29_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_29_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_29_RISING << MXC_F_PWRSEQ_LPWKPOL2B_29_POS) /**< LPWKPOL2B_29_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_29_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_29_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_29_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_29_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_29_POS) /**< LPWKPOL2B_29_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_30_POS                  28 /**< LPWKPOL2B_30 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_30                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_30_POS)) /**< LPWKPOL2B_30 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_30_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_30_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_30_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_30_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_30_POS) /**< LPWKPOL2B_30_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_30_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_30_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_30_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_30_RISING << MXC_F_PWRSEQ_LPWKPOL2B_30_POS) /**< LPWKPOL2B_30_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_30_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_30_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_30_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_30_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_30_POS) /**< LPWKPOL2B_30_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL2B_31_POS                  30 /**< LPWKPOL2B_31 Position */
#define MXC_F_PWRSEQ_LPWKPOL2B_31                      ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL2B_31_POS)) /**< LPWKPOL2B_31 Mask */
#define MXC_V_PWRSEQ_LPWKPOL2B_31_BOTH_EDGES           ((uint32_t)0x0UL) /**< LPWKPOL2B_31_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_31_BOTH_EDGES           (MXC_V_PWRSEQ_LPWKPOL2B_31_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL2B_31_POS) /**< LPWKPOL2B_31_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_31_RISING               ((uint32_t)0x1UL) /**< LPWKPOL2B_31_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_31_RISING               (MXC_V_PWRSEQ_LPWKPOL2B_31_RISING << MXC_F_PWRSEQ_LPWKPOL2B_31_POS) /**< LPWKPOL2B_31_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL2B_31_FALLING              ((uint32_t)0x2UL) /**< LPWKPOL2B_31_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL2B_31_FALLING              (MXC_V_PWRSEQ_LPWKPOL2B_31_FALLING << MXC_F_PWRSEQ_LPWKPOL2B_31_POS) /**< LPWKPOL2B_31_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL2B_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKPOL3 PWRSEQ_LPWKPOL3
 * @brief    Low Power Wakeup Polarity Select for GPIO3.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKPOL3_0_POS                    0 /**< LPWKPOL3_0 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_0                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_0_POS)) /**< LPWKPOL3_0 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_0_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_0_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_0_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_0_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_0_POS) /**< LPWKPOL3_0_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_0_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_0_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_0_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_0_RISING << MXC_F_PWRSEQ_LPWKPOL3_0_POS) /**< LPWKPOL3_0_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_0_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_0_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_0_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_0_FALLING << MXC_F_PWRSEQ_LPWKPOL3_0_POS) /**< LPWKPOL3_0_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_1_POS                    2 /**< LPWKPOL3_1 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_1                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_1_POS)) /**< LPWKPOL3_1 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_1_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_1_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_1_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_1_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_1_POS) /**< LPWKPOL3_1_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_1_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_1_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_1_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_1_RISING << MXC_F_PWRSEQ_LPWKPOL3_1_POS) /**< LPWKPOL3_1_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_1_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_1_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_1_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_1_FALLING << MXC_F_PWRSEQ_LPWKPOL3_1_POS) /**< LPWKPOL3_1_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_2_POS                    4 /**< LPWKPOL3_2 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_2                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_2_POS)) /**< LPWKPOL3_2 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_2_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_2_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_2_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_2_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_2_POS) /**< LPWKPOL3_2_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_2_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_2_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_2_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_2_RISING << MXC_F_PWRSEQ_LPWKPOL3_2_POS) /**< LPWKPOL3_2_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_2_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_2_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_2_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_2_FALLING << MXC_F_PWRSEQ_LPWKPOL3_2_POS) /**< LPWKPOL3_2_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_3_POS                    6 /**< LPWKPOL3_3 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_3                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_3_POS)) /**< LPWKPOL3_3 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_3_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_3_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_3_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_3_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_3_POS) /**< LPWKPOL3_3_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_3_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_3_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_3_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_3_RISING << MXC_F_PWRSEQ_LPWKPOL3_3_POS) /**< LPWKPOL3_3_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_3_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_3_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_3_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_3_FALLING << MXC_F_PWRSEQ_LPWKPOL3_3_POS) /**< LPWKPOL3_3_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_4_POS                    8 /**< LPWKPOL3_4 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_4                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_4_POS)) /**< LPWKPOL3_4 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_4_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_4_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_4_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_4_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_4_POS) /**< LPWKPOL3_4_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_4_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_4_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_4_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_4_RISING << MXC_F_PWRSEQ_LPWKPOL3_4_POS) /**< LPWKPOL3_4_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_4_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_4_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_4_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_4_FALLING << MXC_F_PWRSEQ_LPWKPOL3_4_POS) /**< LPWKPOL3_4_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_5_POS                    10 /**< LPWKPOL3_5 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_5                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_5_POS)) /**< LPWKPOL3_5 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_5_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_5_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_5_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_5_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_5_POS) /**< LPWKPOL3_5_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_5_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_5_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_5_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_5_RISING << MXC_F_PWRSEQ_LPWKPOL3_5_POS) /**< LPWKPOL3_5_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_5_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_5_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_5_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_5_FALLING << MXC_F_PWRSEQ_LPWKPOL3_5_POS) /**< LPWKPOL3_5_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_6_POS                    12 /**< LPWKPOL3_6 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_6                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_6_POS)) /**< LPWKPOL3_6 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_6_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_6_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_6_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_6_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_6_POS) /**< LPWKPOL3_6_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_6_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_6_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_6_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_6_RISING << MXC_F_PWRSEQ_LPWKPOL3_6_POS) /**< LPWKPOL3_6_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_6_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_6_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_6_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_6_FALLING << MXC_F_PWRSEQ_LPWKPOL3_6_POS) /**< LPWKPOL3_6_FALLING Setting */

#define MXC_F_PWRSEQ_LPWKPOL3_7_POS                    14 /**< LPWKPOL3_7 Position */
#define MXC_F_PWRSEQ_LPWKPOL3_7                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPWKPOL3_7_POS)) /**< LPWKPOL3_7 Mask */
#define MXC_V_PWRSEQ_LPWKPOL3_7_BOTH_EDGES             ((uint32_t)0x0UL) /**< LPWKPOL3_7_BOTH_EDGES Value */
#define MXC_S_PWRSEQ_LPWKPOL3_7_BOTH_EDGES             (MXC_V_PWRSEQ_LPWKPOL3_7_BOTH_EDGES << MXC_F_PWRSEQ_LPWKPOL3_7_POS) /**< LPWKPOL3_7_BOTH_EDGES Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_7_RISING                 ((uint32_t)0x1UL) /**< LPWKPOL3_7_RISING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_7_RISING                 (MXC_V_PWRSEQ_LPWKPOL3_7_RISING << MXC_F_PWRSEQ_LPWKPOL3_7_POS) /**< LPWKPOL3_7_RISING Setting */
#define MXC_V_PWRSEQ_LPWKPOL3_7_FALLING                ((uint32_t)0x2UL) /**< LPWKPOL3_7_FALLING Value */
#define MXC_S_PWRSEQ_LPWKPOL3_7_FALLING                (MXC_V_PWRSEQ_LPWKPOL3_7_FALLING << MXC_F_PWRSEQ_LPWKPOL3_7_POS) /**< LPWKPOL3_7_FALLING Setting */

/**@} end of group PWRSEQ_LPWKPOL3_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_PWRSEQ_REGS_H_
