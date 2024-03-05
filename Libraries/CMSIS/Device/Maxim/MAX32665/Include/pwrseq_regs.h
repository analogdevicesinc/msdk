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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t lppwen;               /**< <tt>\b 0x34:</tt> PWRSEQ LPPWEN Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t lpmemsd;              /**< <tt>\b 0x40:</tt> PWRSEQ LPMEMSD Register */
    __IO uint32_t lpvddpd;              /**< <tt>\b 0x44:</tt> PWRSEQ LPVDDPD Register */
    __IO uint32_t buretvec;             /**< <tt>\b 0x48:</tt> PWRSEQ BURETVEC Register */
    __IO uint32_t buaod;                /**< <tt>\b 0x4C:</tt> PWRSEQ BUAOD Register */
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
#define MXC_R_PWRSEQ_LPPWEN                ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
#define MXC_R_PWRSEQ_LPVDDPD               ((uint32_t)0x00000044UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0044</tt> */
#define MXC_R_PWRSEQ_BURETVEC              ((uint32_t)0x00000048UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0048</tt> */
#define MXC_R_PWRSEQ_BUAOD                 ((uint32_t)0x0000004CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x004C</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCN PWRSEQ_LPCN
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCN_RAMRET_POS                   0 /**< LPCN_RAMRET Position */
#define MXC_F_PWRSEQ_LPCN_RAMRET                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCN_RAMRET_POS)) /**< LPCN_RAMRET Mask */
#define MXC_V_PWRSEQ_LPCN_RAMRET_DIS                   ((uint32_t)0x0UL) /**< LPCN_RAMRET_DIS Value */
#define MXC_S_PWRSEQ_LPCN_RAMRET_DIS                   (MXC_V_PWRSEQ_LPCN_RAMRET_DIS << MXC_F_PWRSEQ_LPCN_RAMRET_POS) /**< LPCN_RAMRET_DIS Setting */
#define MXC_V_PWRSEQ_LPCN_RAMRET_EN1                   ((uint32_t)0x1UL) /**< LPCN_RAMRET_EN1 Value */
#define MXC_S_PWRSEQ_LPCN_RAMRET_EN1                   (MXC_V_PWRSEQ_LPCN_RAMRET_EN1 << MXC_F_PWRSEQ_LPCN_RAMRET_POS) /**< LPCN_RAMRET_EN1 Setting */
#define MXC_V_PWRSEQ_LPCN_RAMRET_EN2                   ((uint32_t)0x2UL) /**< LPCN_RAMRET_EN2 Value */
#define MXC_S_PWRSEQ_LPCN_RAMRET_EN2                   (MXC_V_PWRSEQ_LPCN_RAMRET_EN2 << MXC_F_PWRSEQ_LPCN_RAMRET_POS) /**< LPCN_RAMRET_EN2 Setting */
#define MXC_V_PWRSEQ_LPCN_RAMRET_EN3                   ((uint32_t)0x3UL) /**< LPCN_RAMRET_EN3 Value */
#define MXC_S_PWRSEQ_LPCN_RAMRET_EN3                   (MXC_V_PWRSEQ_LPCN_RAMRET_EN3 << MXC_F_PWRSEQ_LPCN_RAMRET_POS) /**< LPCN_RAMRET_EN3 Setting */

#define MXC_F_PWRSEQ_LPCN_BCKGRND_POS                  9 /**< LPCN_BCKGRND Position */
#define MXC_F_PWRSEQ_LPCN_BCKGRND                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_BCKGRND_POS)) /**< LPCN_BCKGRND Mask */

#define MXC_F_PWRSEQ_LPCN_FWKM_POS                     10 /**< LPCN_FWKM Position */
#define MXC_F_PWRSEQ_LPCN_FWKM                         ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_FWKM_POS)) /**< LPCN_FWKM Mask */

#define MXC_F_PWRSEQ_LPCN_BGOFF_POS                    11 /**< LPCN_BGOFF Position */
#define MXC_F_PWRSEQ_LPCN_BGOFF                        ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_BGOFF_POS)) /**< LPCN_BGOFF Mask */

#define MXC_F_PWRSEQ_LPCN_VCOREMD_POS                  20 /**< LPCN_VCOREMD Position */
#define MXC_F_PWRSEQ_LPCN_VCOREMD                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VCOREMD_POS)) /**< LPCN_VCOREMD Mask */

#define MXC_F_PWRSEQ_LPCN_VREGIMD_POS                  21 /**< LPCN_VREGIMD Position */
#define MXC_F_PWRSEQ_LPCN_VREGIMD                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VREGIMD_POS)) /**< LPCN_VREGIMD Mask */

#define MXC_F_PWRSEQ_LPCN_VDDAMD_POS                   22 /**< LPCN_VDDAMD Position */
#define MXC_F_PWRSEQ_LPCN_VDDAMD                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDAMD_POS)) /**< LPCN_VDDAMD Mask */

#define MXC_F_PWRSEQ_LPCN_VDDIOMD_POS                  23 /**< LPCN_VDDIOMD Position */
#define MXC_F_PWRSEQ_LPCN_VDDIOMD                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDIOMD_POS)) /**< LPCN_VDDIOMD Mask */

#define MXC_F_PWRSEQ_LPCN_VDDIOHMD_POS                 24 /**< LPCN_VDDIOHMD Position */
#define MXC_F_PWRSEQ_LPCN_VDDIOHMD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDIOHMD_POS)) /**< LPCN_VDDIOHMD Mask */

#define MXC_F_PWRSEQ_LPCN_PORVDDIOMD_POS               25 /**< LPCN_PORVDDIOMD Position */
#define MXC_F_PWRSEQ_LPCN_PORVDDIOMD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_PORVDDIOMD_POS)) /**< LPCN_PORVDDIOMD Mask */

#define MXC_F_PWRSEQ_LPCN_PORVDDIOHMD_POS              26 /**< LPCN_PORVDDIOHMD Position */
#define MXC_F_PWRSEQ_LPCN_PORVDDIOHMD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_PORVDDIOHMD_POS)) /**< LPCN_PORVDDIOHMD Mask */

#define MXC_F_PWRSEQ_LPCN_VDDBMD_POS                   27 /**< LPCN_VDDBMD Position */
#define MXC_F_PWRSEQ_LPCN_VDDBMD                       ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VDDBMD_POS)) /**< LPCN_VDDBMD Mask */

#define MXC_F_PWRSEQ_LPCN_VRXOUTMD_POS                 28 /**< LPCN_VRXOUTMD Position */
#define MXC_F_PWRSEQ_LPCN_VRXOUTMD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VRXOUTMD_POS)) /**< LPCN_VRXOUTMD Mask */

#define MXC_F_PWRSEQ_LPCN_VTXOUTMD_POS                 29 /**< LPCN_VTXOUTMD Position */
#define MXC_F_PWRSEQ_LPCN_VTXOUTMD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_VTXOUTMD_POS)) /**< LPCN_VTXOUTMD Mask */

#define MXC_F_PWRSEQ_LPCN_PDOWNDSLEN_POS               30 /**< LPCN_PDOWNDSLEN Position */
#define MXC_F_PWRSEQ_LPCN_PDOWNDSLEN                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCN_PDOWNDSLEN_POS)) /**< LPCN_PDOWNDSLEN Mask */

/**@} end of group PWRSEQ_LPCN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKST0 PWRSEQ_LPWKST0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKST0_WAKEST_POS                0 /**< LPWKST0_WAKEST Position */
#define MXC_F_PWRSEQ_LPWKST0_WAKEST                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_LPWKST0_WAKEST_POS)) /**< LPWKST0_WAKEST Mask */

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
#define MXC_F_PWRSEQ_LPWKST1_WAKEST                    ((uint32_t)(0x3FFFFUL << MXC_F_PWRSEQ_LPWKST1_WAKEST_POS)) /**< LPWKST1_WAKEST Mask */

/**@} end of group PWRSEQ_LPWKST1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN1 PWRSEQ_LPWKEN1
 * @brief    Low Power I/O Wakeup Enable Register 1. This register enables low power wakeup
 *           functionality for GPIO1.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN1_WAKEEN_POS                0 /**< LPWKEN1_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWKEN1_WAKEEN                    ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKEN1_WAKEEN_POS)) /**< LPWKEN1_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWKEN1_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWST PWRSEQ_LPPWST
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWST_USBLSWKST_POS              0 /**< LPPWST_USBLSWKST Position */
#define MXC_F_PWRSEQ_LPPWST_USBLSWKST                  ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWST_USBLSWKST_POS)) /**< LPPWST_USBLSWKST Mask */

#define MXC_F_PWRSEQ_LPPWST_USBVBUSWKST_POS            2 /**< LPPWST_USBVBUSWKST Position */
#define MXC_F_PWRSEQ_LPPWST_USBVBUSWKST                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_USBVBUSWKST_POS)) /**< LPPWST_USBVBUSWKST Mask */

#define MXC_F_PWRSEQ_LPPWST_SDMAWKST_POS               3 /**< LPPWST_SDMAWKST Position */
#define MXC_F_PWRSEQ_LPPWST_SDMAWKST                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_SDMAWKST_POS)) /**< LPPWST_SDMAWKST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP0WKST_POS           4 /**< LPPWST_AINCOMP0WKST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP0WKST               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP0WKST_POS)) /**< LPPWST_AINCOMP0WKST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP1WKST_POS           5 /**< LPPWST_AINCOMP1WKST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP1WKST               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP1WKST_POS)) /**< LPPWST_AINCOMP1WKST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP2WKST_POS           6 /**< LPPWST_AINCOMP2WKST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP2WKST               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP2WKST_POS)) /**< LPPWST_AINCOMP2WKST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP3WKST_POS           7 /**< LPPWST_AINCOMP3WKST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP3WKST               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP3WKST_POS)) /**< LPPWST_AINCOMP3WKST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP0ST_POS             8 /**< LPPWST_AINCOMP0ST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP0ST                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP0ST_POS)) /**< LPPWST_AINCOMP0ST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP1ST_POS             9 /**< LPPWST_AINCOMP1ST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP1ST                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP1ST_POS)) /**< LPPWST_AINCOMP1ST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP2ST_POS             10 /**< LPPWST_AINCOMP2ST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP2ST                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP2ST_POS)) /**< LPPWST_AINCOMP2ST Mask */

#define MXC_F_PWRSEQ_LPPWST_AINCOMP3ST_POS             11 /**< LPPWST_AINCOMP3ST Position */
#define MXC_F_PWRSEQ_LPPWST_AINCOMP3ST                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_AINCOMP3ST_POS)) /**< LPPWST_AINCOMP3ST Mask */

#define MXC_F_PWRSEQ_LPPWST_BBMODEST_POS               16 /**< LPPWST_BBMODEST Position */
#define MXC_F_PWRSEQ_LPPWST_BBMODEST                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_BBMODEST_POS)) /**< LPPWST_BBMODEST Mask */

#define MXC_F_PWRSEQ_LPPWST_RSTWKST_POS                17 /**< LPPWST_RSTWKST Position */
#define MXC_F_PWRSEQ_LPPWST_RSTWKST                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_RSTWKST_POS)) /**< LPPWST_RSTWKST Mask */

/**@} end of group PWRSEQ_LPPWST_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWEN PWRSEQ_LPPWEN
 * @brief    Low Power Peripheral Wakeup Enable Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWEN_USBLSWKEN_POS              0 /**< LPPWEN_USBLSWKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_USBLSWKEN                  ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPPWEN_USBLSWKEN_POS)) /**< LPPWEN_USBLSWKEN Mask */

#define MXC_F_PWRSEQ_LPPWEN_USBVBUSWKEN_POS            2 /**< LPPWEN_USBVBUSWKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_USBVBUSWKEN                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_USBVBUSWKEN_POS)) /**< LPPWEN_USBVBUSWKEN Mask */

#define MXC_F_PWRSEQ_LPPWEN_SDMAWKEN_POS               3 /**< LPPWEN_SDMAWKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_SDMAWKEN                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_SDMAWKEN_POS)) /**< LPPWEN_SDMAWKEN Mask */

#define MXC_F_PWRSEQ_LPPWEN_AINCOMP0WKEN_POS           4 /**< LPPWEN_AINCOMP0WKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_AINCOMP0WKEN               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_AINCOMP0WKEN_POS)) /**< LPPWEN_AINCOMP0WKEN Mask */

#define MXC_F_PWRSEQ_LPPWEN_AINCOMP1WKEN_POS           5 /**< LPPWEN_AINCOMP1WKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_AINCOMP1WKEN               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_AINCOMP1WKEN_POS)) /**< LPPWEN_AINCOMP1WKEN Mask */

#define MXC_F_PWRSEQ_LPPWEN_AINCOMP2WKEN_POS           6 /**< LPPWEN_AINCOMP2WKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_AINCOMP2WKEN               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_AINCOMP2WKEN_POS)) /**< LPPWEN_AINCOMP2WKEN Mask */

#define MXC_F_PWRSEQ_LPPWEN_AINCOMP3WKEN_POS           7 /**< LPPWEN_AINCOMP3WKEN Position */
#define MXC_F_PWRSEQ_LPPWEN_AINCOMP3WKEN               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWEN_AINCOMP3WKEN_POS)) /**< LPPWEN_AINCOMP3WKEN Mask */

/**@} end of group PWRSEQ_LPPWEN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPMEMSD PWRSEQ_LPMEMSD
 * @brief    Low Power Memory Shutdown Control.
 * @{
 */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM0SD_POS               0 /**< LPMEMSD_SRAM0SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM0SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM0SD_POS)) /**< LPMEMSD_SRAM0SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM1SD_POS               1 /**< LPMEMSD_SRAM1SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM1SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM1SD_POS)) /**< LPMEMSD_SRAM1SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM2SD_POS               2 /**< LPMEMSD_SRAM2SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM2SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM2SD_POS)) /**< LPMEMSD_SRAM2SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM3SD_POS               3 /**< LPMEMSD_SRAM3SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM3SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM3SD_POS)) /**< LPMEMSD_SRAM3SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM4SD_POS               4 /**< LPMEMSD_SRAM4SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM4SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM4SD_POS)) /**< LPMEMSD_SRAM4SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM5SD_POS               5 /**< LPMEMSD_SRAM5SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM5SD                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM5SD_POS)) /**< LPMEMSD_SRAM5SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICACHESD_POS              7 /**< LPMEMSD_ICACHESD Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICACHESD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICACHESD_POS)) /**< LPMEMSD_ICACHESD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ICACHEXIPSD_POS           8 /**< LPMEMSD_ICACHEXIPSD Position */
#define MXC_F_PWRSEQ_LPMEMSD_ICACHEXIPSD               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ICACHEXIPSD_POS)) /**< LPMEMSD_ICACHEXIPSD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRCCSD_POS                9 /**< LPMEMSD_SRCCSD Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRCCSD                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRCCSD_POS)) /**< LPMEMSD_SRCCSD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_CRYPTOSD_POS              10 /**< LPMEMSD_CRYPTOSD Position */
#define MXC_F_PWRSEQ_LPMEMSD_CRYPTOSD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_CRYPTOSD_POS)) /**< LPMEMSD_CRYPTOSD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_USBFIFOSD_POS             11 /**< LPMEMSD_USBFIFOSD Position */
#define MXC_F_PWRSEQ_LPMEMSD_USBFIFOSD                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_USBFIFOSD_POS)) /**< LPMEMSD_USBFIFOSD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROMSD_POS                 12 /**< LPMEMSD_ROMSD Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROMSD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROMSD_POS)) /**< LPMEMSD_ROMSD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_ROM1SD_POS                13 /**< LPMEMSD_ROM1SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_ROM1SD                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_ROM1SD_POS)) /**< LPMEMSD_ROM1SD Mask */

#define MXC_F_PWRSEQ_LPMEMSD_IC1SD_POS                 14 /**< LPMEMSD_IC1SD Position */
#define MXC_F_PWRSEQ_LPMEMSD_IC1SD                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_IC1SD_POS)) /**< LPMEMSD_IC1SD Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPVDDPD PWRSEQ_LPVDDPD
 * @brief    Low Power VDD Domain Power Down Control.
 * @{
 */
#define MXC_F_PWRSEQ_LPVDDPD_VREGOBPD_POS              0 /**< LPVDDPD_VREGOBPD Position */
#define MXC_F_PWRSEQ_LPVDDPD_VREGOBPD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_VREGOBPD_POS)) /**< LPVDDPD_VREGOBPD Mask */

#define MXC_F_PWRSEQ_LPVDDPD_VREGODPD_POS              1 /**< LPVDDPD_VREGODPD Position */
#define MXC_F_PWRSEQ_LPVDDPD_VREGODPD                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_VREGODPD_POS)) /**< LPVDDPD_VREGODPD Mask */

#define MXC_F_PWRSEQ_LPVDDPD_VDD2PD_POS                8 /**< LPVDDPD_VDD2PD Position */
#define MXC_F_PWRSEQ_LPVDDPD_VDD2PD                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_VDD2PD_POS)) /**< LPVDDPD_VDD2PD Mask */

#define MXC_F_PWRSEQ_LPVDDPD_VDD3PD_POS                9 /**< LPVDDPD_VDD3PD Position */
#define MXC_F_PWRSEQ_LPVDDPD_VDD3PD                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_VDD3PD_POS)) /**< LPVDDPD_VDD3PD Mask */

#define MXC_F_PWRSEQ_LPVDDPD_VDD4PD_POS                10 /**< LPVDDPD_VDD4PD Position */
#define MXC_F_PWRSEQ_LPVDDPD_VDD4PD                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_VDD4PD_POS)) /**< LPVDDPD_VDD4PD Mask */

#define MXC_F_PWRSEQ_LPVDDPD_VDD5PD_POS                11 /**< LPVDDPD_VDD5PD Position */
#define MXC_F_PWRSEQ_LPVDDPD_VDD5PD                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPVDDPD_VDD5PD_POS)) /**< LPVDDPD_VDD5PD Mask */

/**@} end of group PWRSEQ_LPVDDPD_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_BURETVEC PWRSEQ_BURETVEC
 * @brief    BACKUP Return Vector Register
 * @{
 */
#define MXC_F_PWRSEQ_BURETVEC_GPR0_POS                 0 /**< BURETVEC_GPR0 Position */
#define MXC_F_PWRSEQ_BURETVEC_GPR0                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_BURETVEC_GPR0_POS)) /**< BURETVEC_GPR0 Mask */

/**@} end of group PWRSEQ_BURETVEC_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_BUAOD PWRSEQ_BUAOD
 * @brief    BACKUP AoD Register
 * @{
 */
#define MXC_F_PWRSEQ_BUAOD_GPR1_POS                    0 /**< BUAOD_GPR1 Position */
#define MXC_F_PWRSEQ_BUAOD_GPR1                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_PWRSEQ_BUAOD_GPR1_POS)) /**< BUAOD_GPR1 Mask */

/**@} end of group PWRSEQ_BUAOD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_PWRSEQ_REGS_H_
