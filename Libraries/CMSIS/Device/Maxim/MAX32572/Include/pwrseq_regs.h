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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_PWRSEQ_REGS_H_

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

#define MXC_F_PWRSEQ_LPCTRL_BGOFF_POS                  11 /**< LPCTRL_BGOFF Position */
#define MXC_F_PWRSEQ_LPCTRL_BGOFF                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_BGOFF_POS)) /**< LPCTRL_BGOFF Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS_POS           12 /**< LPCTRL_VCOREPOR_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS_POS)) /**< LPCTRL_VCOREPOR_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOHHVMON_DIS_POS        17 /**< LPCTRL_VDDIOHHVMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOHHVMON_DIS            ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOHHVMON_DIS_POS)) /**< LPCTRL_VDDIOHHVMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDIOHVMON_DIS_POS         18 /**< LPCTRL_VDDIOHVMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDIOHVMON_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDIOHVMON_DIS_POS)) /**< LPCTRL_VDDIOHVMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREHVMON_DIS_POS         19 /**< LPCTRL_VCOREHVMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREHVMON_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREHVMON_DIS_POS)) /**< LPCTRL_VCOREHVMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS_POS           20 /**< LPCTRL_VCOREMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS_POS)) /**< LPCTRL_VCOREMON_DIS Mask */

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

#define MXC_F_PWRSEQ_LPCTRL_DEEPSLEEP_PDOUT_DIS_POS    30 /**< LPCTRL_DEEPSLEEP_PDOUT_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_DEEPSLEEP_PDOUT_DIS        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_DEEPSLEEP_PDOUT_DIS_POS)) /**< LPCTRL_DEEPSLEEP_PDOUT_DIS Mask */

/**@} end of group PWRSEQ_LPCTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKFL0 PWRSEQ_LPWKFL0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKFL0_WAKEST_POS                0 /**< LPWKFL0_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKFL0_WAKEST                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPWKFL0_WAKEST_POS)) /**< LPWKFL0_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKFL0_Register */

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
 * @defgroup PWRSEQ_LPPWKFL PWRSEQ_LPPWKFL
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKFL_USBLS_POS                 0 /**< LPPWKFL_USBLS Position */
#define MXC_F_PWRSEQ_LPPWKFL_USBLS                     ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWKFL_USBLS_POS)) /**< LPPWKFL_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWKFL_USBVBUS_POS               2 /**< LPPWKFL_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWKFL_USBVBUS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_USBVBUS_POS)) /**< LPPWKFL_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWKFL_CPU1_POS                  3 /**< LPPWKFL_CPU1 Position */
#define MXC_F_PWRSEQ_LPPWKFL_CPU1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_CPU1_POS)) /**< LPPWKFL_CPU1 Mask */

#define MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS                16 /**< LPPWKFL_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWKFL_BACKUP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS)) /**< LPPWKFL_BACKUP Mask */

#define MXC_F_PWRSEQ_LPPWKFL_RESET_POS                 17 /**< LPPWKFL_RESET Position */
#define MXC_F_PWRSEQ_LPPWKFL_RESET                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_RESET_POS)) /**< LPPWKFL_RESET Mask */

#define MXC_F_PWRSEQ_LPPWKFL_DRS_EVT_POS               19 /**< LPPWKFL_DRS_EVT Position */
#define MXC_F_PWRSEQ_LPPWKFL_DRS_EVT                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_DRS_EVT_POS)) /**< LPPWKFL_DRS_EVT Mask */

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

#define MXC_F_PWRSEQ_LPPWKEN_CPU1_POS                  3 /**< LPPWKEN_CPU1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_CPU1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_CPU1_POS)) /**< LPPWKEN_CPU1 Mask */

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

#define MXC_F_PWRSEQ_LPMEMSD_RAM6_POS                  6 /**< LPMEMSD_RAM6 Position */
#define MXC_F_PWRSEQ_LPMEMSD_RAM6                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_RAM6_POS)) /**< LPMEMSD_RAM6 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICCXIP_POS                8 /**< LPMEMSD_ICCXIP Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICCXIP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICCXIP_POS)) /**< LPMEMSD_ICCXIP Mask */

#define MXC_F_PWRSEQ_LPMEMSD_CRYPTO_POS                10 /**< LPMEMSD_CRYPTO Position */
#define MXC_F_PWRSEQ_LPMEMSD_CRYPTO                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_CRYPTO_POS)) /**< LPMEMSD_CRYPTO Mask */

#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS               11 /**< LPMEMSD_USBFIFO Position */
#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS)) /**< LPMEMSD_USBFIFO Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM0_POS                  12 /**< LPMEMSD_ROM0 Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM0                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM0_POS)) /**< LPMEMSD_ROM0 Mask */

#define MXC_F_PWRSEQ_LPMEMSD_MEUMEM_POS                13 /**< LPMEMSD_MEUMEM Position */
#define MXC_F_PWRSEQ_LPMEMSD_MEUMEM                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_MEUMEM_POS)) /**< LPMEMSD_MEUMEM Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM1_POS                  15 /**< LPMEMSD_ROM1 Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM1                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM1_POS)) /**< LPMEMSD_ROM1 Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_PWRSEQ_REGS_H_
