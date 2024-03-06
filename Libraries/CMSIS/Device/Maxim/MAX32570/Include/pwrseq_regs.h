/**
 * @file    pwrseq_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PWRSEQ Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t lppwkst;              /**< <tt>\b 0x30:</tt> PWRSEQ LPPWKST Register */
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
#define MXC_R_PWRSEQ_LPCN                  ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_LPWKST0               ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_LPWKEN0               ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_LPWKST1               ((uint32_t)0x0000000CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x000C</tt> */
#define MXC_R_PWRSEQ_LPWKEN1               ((uint32_t)0x00000010UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0010</tt> */
#define MXC_R_PWRSEQ_LPWKST2               ((uint32_t)0x00000014UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0014</tt> */
#define MXC_R_PWRSEQ_LPWKEN2               ((uint32_t)0x00000018UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0018</tt> */
#define MXC_R_PWRSEQ_LPWKST3               ((uint32_t)0x0000001CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x001C</tt> */
#define MXC_R_PWRSEQ_LPWKEN3               ((uint32_t)0x00000020UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0020</tt> */
#define MXC_R_PWRSEQ_LPPWKST               ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPPWKEN               ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
#define MXC_R_PWRSEQ_LPVDDPD               ((uint32_t)0x00000044UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0044</tt> */
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

#define MXC_F_PWRSEQ_LPCN_OVR_POS                      4 /**< LPCN_OVR Position */
#define MXC_F_PWRSEQ_LPCN_OVR                          ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCN_OVR_POS)) /**< LPCN_OVR Mask */
#define MXC_V_PWRSEQ_LPCN_OVR_0_9V                     ((uint32_t)0x0UL) /**< LPCN_OVR_0_9V Value */
#define MXC_S_PWRSEQ_LPCN_OVR_0_9V                     (MXC_V_PWRSEQ_LPCN_OVR_0_9V << MXC_F_PWRSEQ_LPCN_OVR_POS) /**< LPCN_OVR_0_9V Setting */
#define MXC_V_PWRSEQ_LPCN_OVR_1_0V                     ((uint32_t)0x1UL) /**< LPCN_OVR_1_0V Value */
#define MXC_S_PWRSEQ_LPCN_OVR_1_0V                     (MXC_V_PWRSEQ_LPCN_OVR_1_0V << MXC_F_PWRSEQ_LPCN_OVR_POS) /**< LPCN_OVR_1_0V Setting */
#define MXC_V_PWRSEQ_LPCN_OVR_1_1V                     ((uint32_t)0x2UL) /**< LPCN_OVR_1_1V Value */
#define MXC_S_PWRSEQ_LPCN_OVR_1_1V                     (MXC_V_PWRSEQ_LPCN_OVR_1_1V << MXC_F_PWRSEQ_LPCN_OVR_POS) /**< LPCN_OVR_1_1V Setting */

#define MXC_F_PWRSEQ_LPCN_RETREG_EN_POS                8 /**< LPCN_RETREG_EN Position */
#define MXC_F_PWRSEQ_LPCN_RETREG_EN                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_RETREG_EN_POS)) /**< LPCN_RETREG_EN Mask */

#define MXC_F_PWRSEQ_LPCN_FASTWK_EN_POS                10 /**< LPCN_FASTWK_EN Position */
#define MXC_F_PWRSEQ_LPCN_FASTWK_EN                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_FASTWK_EN_POS)) /**< LPCN_FASTWK_EN Mask */

#define MXC_F_PWRSEQ_LPCN_BG_DIS_POS                   11 /**< LPCN_BG_DIS Position */
#define MXC_F_PWRSEQ_LPCN_BG_DIS                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_BG_DIS_POS)) /**< LPCN_BG_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VCOREPOR_DIS_POS             12 /**< LPCN_VCOREPOR_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VCOREPOR_DIS                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VCOREPOR_DIS_POS)) /**< LPCN_VCOREPOR_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_LDO_DIS_POS                  16 /**< LPCN_LDO_DIS Position */
#define MXC_F_PWRSEQ_LPCN_LDO_DIS                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_LDO_DIS_POS)) /**< LPCN_LDO_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VCOREMON_DIS_POS             20 /**< LPCN_VCOREMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VCOREMON_DIS                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VCOREMON_DIS_POS)) /**< LPCN_VCOREMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VRTCMON_DIS_POS              21 /**< LPCN_VRTCMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VRTCMON_DIS                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VRTCMON_DIS_POS)) /**< LPCN_VRTCMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VDDAMON_DIS_POS              22 /**< LPCN_VDDAMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VDDAMON_DIS                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDAMON_DIS_POS)) /**< LPCN_VDDAMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VDDIOMON_DIS_POS             23 /**< LPCN_VDDIOMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VDDIOMON_DIS                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDIOMON_DIS_POS)) /**< LPCN_VDDIOMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_VDDIOHMON_DIS_POS            24 /**< LPCN_VDDIOHMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_VDDIOHMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDIOHMON_DIS_POS)) /**< LPCN_VDDIOHMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCN_PORVDDBMON_DIS_POS           27 /**< LPCN_PORVDDBMON_DIS Position */
#define MXC_F_PWRSEQ_LPCN_PORVDDBMON_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_PORVDDBMON_DIS_POS)) /**< LPCN_PORVDDBMON_DIS Mask */

/**@} end of group PWRSEQ_LPCN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST0 PWRSEQ_LPWKST0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST0_ST_POS                    0 /**< LPWKST0_ST Position */
#define MXC_F_PWRSEQ_LPWKST0_ST                        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPWKST0_ST_POS)) /**< LPWKST0_ST Mask */

/**@} end of group PWRSEQ_LPWKST0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_EN_POS                    0 /**< LPWKEN0_EN Position */
#define MXC_F_PWRSEQ_LPWKEN0_EN                        ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKEN0_EN_POS)) /**< LPWKEN0_EN Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKST PWRSEQ_LPPWKST
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKST_USBLS_POS                 0 /**< LPPWKST_USBLS Position */
#define MXC_F_PWRSEQ_LPPWKST_USBLS                     ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWKST_USBLS_POS)) /**< LPPWKST_USBLS Mask */

#define MXC_F_PWRSEQ_LPPWKST_USBVBUS_POS               2 /**< LPPWKST_USBVBUS Position */
#define MXC_F_PWRSEQ_LPPWKST_USBVBUS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_USBVBUS_POS)) /**< LPPWKST_USBVBUS Mask */

#define MXC_F_PWRSEQ_LPPWKST_HA0_POS                   3 /**< LPPWKST_HA0 Position */
#define MXC_F_PWRSEQ_LPPWKST_HA0                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_HA0_POS)) /**< LPPWKST_HA0 Mask */

#define MXC_F_PWRSEQ_LPPWKST_BBMOD_POS                 16 /**< LPPWKST_BBMOD Position */
#define MXC_F_PWRSEQ_LPPWKST_BBMOD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_BBMOD_POS)) /**< LPPWKST_BBMOD Mask */

#define MXC_F_PWRSEQ_LPPWKST_RST_POS                   17 /**< LPPWKST_RST Position */
#define MXC_F_PWRSEQ_LPPWKST_RST                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_RST_POS)) /**< LPPWKST_RST Mask */

#define MXC_F_PWRSEQ_LPPWKST_SDMA1_POS                 18 /**< LPPWKST_SDMA1 Position */
#define MXC_F_PWRSEQ_LPPWKST_SDMA1                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKST_SDMA1_POS)) /**< LPPWKST_SDMA1 Mask */

/**@} end of group PWRSEQ_LPPWKST_Register */

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

#define MXC_F_PWRSEQ_LPPWKEN_SDMA0_POS                 3 /**< LPPWKEN_SDMA0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_SDMA0                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_SDMA0_POS)) /**< LPPWKEN_SDMA0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_SDMA1_POS                 18 /**< LPPWKEN_SDMA1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_SDMA1                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_SDMA1_POS)) /**< LPPWKEN_SDMA1 Mask */

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

#define MXC_F_PWRSEQ_LPMEMSD_ICACHE_POS                7 /**< LPMEMSD_ICACHE Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICACHE                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICACHE_POS)) /**< LPMEMSD_ICACHE Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICACHEXIP_POS             8 /**< LPMEMSD_ICACHEXIP Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICACHEXIP                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICACHEXIP_POS)) /**< LPMEMSD_ICACHEXIP Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRCC_POS                  9 /**< LPMEMSD_SRCC Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRCC                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRCC_POS)) /**< LPMEMSD_SRCC Mask */

#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS               11 /**< LPMEMSD_USBFIFO Position */
#define MXC_F_PWRSEQ_LPMEMSD_USBFIFO                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_USBFIFO_POS)) /**< LPMEMSD_USBFIFO Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM_POS                   12 /**< LPMEMSD_ROM Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM_POS)) /**< LPMEMSD_ROM Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_PWRSEQ_REGS_H_
