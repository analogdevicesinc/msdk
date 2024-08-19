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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_PWRSEQ_REGS_H_

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
    __R  uint32_t rsv_0xc_0x2f[9];
    __IO uint32_t lppwkfl;              /**< <tt>\b 0x30:</tt> PWRSEQ LPPWKFL Register */
    __IO uint32_t lppwken;              /**< <tt>\b 0x34:</tt> PWRSEQ LPPWKEN Register */
    __R  uint32_t rsv_0x38_0x3f[2];
    __IO uint32_t lpmemsd;              /**< <tt>\b 0x40:</tt> PWRSEQ LPMEMSD Register */
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
#define MXC_R_PWRSEQ_LPPWKFL               ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_LPPWKEN               ((uint32_t)0x00000034UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0034</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCTRL PWRSEQ_LPCTRL
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCTRL_RAM0RET_EN_POS             0 /**< LPCTRL_RAM0RET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RAM0RET_EN                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RAM0RET_EN_POS)) /**< LPCTRL_RAM0RET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_RAM1RET_EN_POS             1 /**< LPCTRL_RAM1RET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RAM1RET_EN                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RAM1RET_EN_POS)) /**< LPCTRL_RAM1RET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_RAM2RET_EN_POS             2 /**< LPCTRL_RAM2RET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RAM2RET_EN                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RAM2RET_EN_POS)) /**< LPCTRL_RAM2RET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_RAM3RET_EN_POS             3 /**< LPCTRL_RAM3RET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RAM3RET_EN                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RAM3RET_EN_POS)) /**< LPCTRL_RAM3RET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_OVR_POS                    4 /**< LPCTRL_OVR Position */
#define MXC_F_PWRSEQ_LPCTRL_OVR                        ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LPCTRL_OVR_POS)) /**< LPCTRL_OVR Mask */
#define MXC_V_PWRSEQ_LPCTRL_OVR_0_9V                   ((uint32_t)0x0UL) /**< LPCTRL_OVR_0_9V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_0_9V                   (MXC_V_PWRSEQ_LPCTRL_OVR_0_9V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_0_9V Setting */
#define MXC_V_PWRSEQ_LPCTRL_OVR_1_0V                   ((uint32_t)0x1UL) /**< LPCTRL_OVR_1_0V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_1_0V                   (MXC_V_PWRSEQ_LPCTRL_OVR_1_0V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_1_0V Setting */
#define MXC_V_PWRSEQ_LPCTRL_OVR_1_1V                   ((uint32_t)0x2UL) /**< LPCTRL_OVR_1_1V Value */
#define MXC_S_PWRSEQ_LPCTRL_OVR_1_1V                   (MXC_V_PWRSEQ_LPCTRL_OVR_1_1V << MXC_F_PWRSEQ_LPCTRL_OVR_POS) /**< LPCTRL_OVR_1_1V Setting */

#define MXC_F_PWRSEQ_LPCTRL_VCORE_DET_BYPASS_POS       6 /**< LPCTRL_VCORE_DET_BYPASS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCORE_DET_BYPASS           ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCORE_DET_BYPASS_POS)) /**< LPCTRL_VCORE_DET_BYPASS Mask */

#define MXC_F_PWRSEQ_LPCTRL_FVDD_EN_POS                7 /**< LPCTRL_FVDD_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_FVDD_EN                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_FVDD_EN_POS)) /**< LPCTRL_FVDD_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_RETREG_EN_POS              8 /**< LPCTRL_RETREG_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RETREG_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RETREG_EN_POS)) /**< LPCTRL_RETREG_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_STORAGE_EN_POS             9 /**< LPCTRL_STORAGE_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_STORAGE_EN                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_STORAGE_EN_POS)) /**< LPCTRL_STORAGE_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_FASTWK_EN_POS              10 /**< LPCTRL_FASTWK_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_FASTWK_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_FASTWK_EN_POS)) /**< LPCTRL_FASTWK_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS                 11 /**< LPCTRL_BG_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_BG_DIS                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS)) /**< LPCTRL_BG_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS_POS           12 /**< LPCTRL_VCOREPOR_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS_POS)) /**< LPCTRL_VCOREPOR_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_LDO_DIS_POS                16 /**< LPCTRL_LDO_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_LDO_DIS                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_LDO_DIS_POS)) /**< LPCTRL_LDO_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCORE_EXT_POS              17 /**< LPCTRL_VCORE_EXT Position */
#define MXC_F_PWRSEQ_LPCTRL_VCORE_EXT                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCORE_EXT_POS)) /**< LPCTRL_VCORE_EXT Mask */

#define MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS_POS           20 /**< LPCTRL_VCOREMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VCOREMON_DIS_POS)) /**< LPCTRL_VCOREMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS_POS            22 /**< LPCTRL_VDDAMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VDDAMON_DIS_POS)) /**< LPCTRL_VDDAMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_PORVDDMON_DIS_POS          25 /**< LPCTRL_PORVDDMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_PORVDDMON_DIS              ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_PORVDDMON_DIS_POS)) /**< LPCTRL_PORVDDMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_VBBMON_DIS_POS             27 /**< LPCTRL_VBBMON_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_VBBMON_DIS                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_VBBMON_DIS_POS)) /**< LPCTRL_VBBMON_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_INRO_EN_POS                28 /**< LPCTRL_INRO_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_INRO_EN                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_INRO_EN_POS)) /**< LPCTRL_INRO_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_ERTCO_EN_POS               29 /**< LPCTRL_ERTCO_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_ERTCO_EN                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_ERTCO_EN_POS)) /**< LPCTRL_ERTCO_EN Mask */

/**@} end of group PWRSEQ_LPCTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKFL0 PWRSEQ_LPWKFL0
 * @brief    Low Power I/O Wakeup Status Flag Register 0. This register indicates the low
 *           power wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKFL0_WAKEFL_POS                0 /**< LPWKFL0_WAKEFL Position */
#define MXC_F_PWRSEQ_LPWKFL0_WAKEFL                    ((uint32_t)(0x7FFFFFFFUL << MXC_F_PWRSEQ_LPWKFL0_WAKEFL_POS)) /**< LPWKFL0_WAKEFL Mask */

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
 * @brief    Low Power Peripheral Wakeup Status Flag Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKFL_TMR3_POS                  0 /**< LPPWKFL_TMR3 Position */
#define MXC_F_PWRSEQ_LPPWKFL_TMR3                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_TMR3_POS)) /**< LPPWKFL_TMR3 Mask */

#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_POS              3 /**< LPPWKFL_AINCOMP0 Position */
#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP0                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_POS)) /**< LPPWKFL_AINCOMP0 Mask */

#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP1_POS              4 /**< LPPWKFL_AINCOMP1 Position */
#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP1                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_AINCOMP1_POS)) /**< LPPWKFL_AINCOMP1 Mask */

#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_ST_POS           5 /**< LPPWKFL_AINCOMP0_ST Position */
#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_ST               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_AINCOMP0_ST_POS)) /**< LPPWKFL_AINCOMP0_ST Mask */

#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP1_ST_POS           6 /**< LPPWKFL_AINCOMP1_ST Position */
#define MXC_F_PWRSEQ_LPPWKFL_AINCOMP1_ST               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_AINCOMP1_ST_POS)) /**< LPPWKFL_AINCOMP1_ST Mask */

#define MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS                16 /**< LPPWKFL_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWKFL_BACKUP                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKFL_BACKUP_POS)) /**< LPPWKFL_BACKUP Mask */

/**@} end of group PWRSEQ_LPPWKFL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWKEN PWRSEQ_LPPWKEN
 * @brief    Low Power Peripheral Wakeup Enable Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWKEN_LPTMR0_POS                0 /**< LPPWKEN_LPTMR0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_LPTMR0                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_LPTMR0_POS)) /**< LPPWKEN_LPTMR0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_AINCOMP0_POS              3 /**< LPPWKEN_AINCOMP0 Position */
#define MXC_F_PWRSEQ_LPPWKEN_AINCOMP0                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_AINCOMP0_POS)) /**< LPPWKEN_AINCOMP0 Mask */

#define MXC_F_PWRSEQ_LPPWKEN_AINCOMP1_POS              4 /**< LPPWKEN_AINCOMP1 Position */
#define MXC_F_PWRSEQ_LPPWKEN_AINCOMP1                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWKEN_AINCOMP1_POS)) /**< LPPWKEN_AINCOMP1 Mask */

/**@} end of group PWRSEQ_LPPWKEN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPMEMSD PWRSEQ_LPMEMSD
 * @brief    Low Power Memory Shutdown Register.
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

/**@} end of group PWRSEQ_LPMEMSD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_PWRSEQ_REGS_H_
