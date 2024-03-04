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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32660_INCLUDE_PWRSEQ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32660_INCLUDE_PWRSEQ_REGS_H_

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
    __IO uint32_t lp_ctrl;              /**< <tt>\b 0x00:</tt> PWRSEQ LP_CTRL Register */
    __IO uint32_t lp_wakefl;            /**< <tt>\b 0x04:</tt> PWRSEQ LP_WAKEFL Register */
    __IO uint32_t lpwk_en;              /**< <tt>\b 0x08:</tt> PWRSEQ LPWK_EN Register */
    __R  uint32_t rsv_0xc_0x3f[13];
    __IO uint32_t lpmemsd;              /**< <tt>\b 0x40:</tt> PWRSEQ LPMEMSD Register */
} mxc_pwrseq_regs_t;

/* Register offsets for module PWRSEQ */
/**
 * @ingroup    pwrseq_registers
 * @defgroup   PWRSEQ_Register_Offsets Register Offsets
 * @brief      PWRSEQ Peripheral Register Offsets from the PWRSEQ Base Peripheral Address.
 * @{
 */
#define MXC_R_PWRSEQ_LP_CTRL               ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_LP_WAKEFL             ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_LPWK_EN               ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_LPMEMSD               ((uint32_t)0x00000040UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0040</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LP_CTRL PWRSEQ_LP_CTRL
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL0_POS           0 /**< LP_CTRL_RAMRET_SEL0 Position */
#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL0               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL0_POS)) /**< LP_CTRL_RAMRET_SEL0 Mask */

#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL1_POS           1 /**< LP_CTRL_RAMRET_SEL1 Position */
#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL1               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL1_POS)) /**< LP_CTRL_RAMRET_SEL1 Mask */

#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL2_POS           2 /**< LP_CTRL_RAMRET_SEL2 Position */
#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL2               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL2_POS)) /**< LP_CTRL_RAMRET_SEL2 Mask */

#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL3_POS           3 /**< LP_CTRL_RAMRET_SEL3 Position */
#define MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL3               ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_RAMRET_SEL3_POS)) /**< LP_CTRL_RAMRET_SEL3 Mask */

#define MXC_F_PWRSEQ_LP_CTRL_OVR_POS                   4 /**< LP_CTRL_OVR Position */
#define MXC_F_PWRSEQ_LP_CTRL_OVR                       ((uint32_t)(0x3UL << MXC_F_PWRSEQ_LP_CTRL_OVR_POS)) /**< LP_CTRL_OVR Mask */
#define MXC_V_PWRSEQ_LP_CTRL_OVR_0_9V                  ((uint32_t)0x0UL) /**< LP_CTRL_OVR_0_9V Value */
#define MXC_S_PWRSEQ_LP_CTRL_OVR_0_9V                  (MXC_V_PWRSEQ_LP_CTRL_OVR_0_9V << MXC_F_PWRSEQ_LP_CTRL_OVR_POS) /**< LP_CTRL_OVR_0_9V Setting */
#define MXC_V_PWRSEQ_LP_CTRL_OVR_1_0V                  ((uint32_t)0x1UL) /**< LP_CTRL_OVR_1_0V Value */
#define MXC_S_PWRSEQ_LP_CTRL_OVR_1_0V                  (MXC_V_PWRSEQ_LP_CTRL_OVR_1_0V << MXC_F_PWRSEQ_LP_CTRL_OVR_POS) /**< LP_CTRL_OVR_1_0V Setting */
#define MXC_V_PWRSEQ_LP_CTRL_OVR_1_1V                  ((uint32_t)0x2UL) /**< LP_CTRL_OVR_1_1V Value */
#define MXC_S_PWRSEQ_LP_CTRL_OVR_1_1V                  (MXC_V_PWRSEQ_LP_CTRL_OVR_1_1V << MXC_F_PWRSEQ_LP_CTRL_OVR_POS) /**< LP_CTRL_OVR_1_1V Setting */

#define MXC_F_PWRSEQ_LP_CTRL_VCORE_DET_BYPASS_POS      6 /**< LP_CTRL_VCORE_DET_BYPASS Position */
#define MXC_F_PWRSEQ_LP_CTRL_VCORE_DET_BYPASS          ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_VCORE_DET_BYPASS_POS)) /**< LP_CTRL_VCORE_DET_BYPASS Mask */

#define MXC_F_PWRSEQ_LP_CTRL_RETREG_EN_POS             8 /**< LP_CTRL_RETREG_EN Position */
#define MXC_F_PWRSEQ_LP_CTRL_RETREG_EN                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_RETREG_EN_POS)) /**< LP_CTRL_RETREG_EN Mask */

#define MXC_F_PWRSEQ_LP_CTRL_FAST_WK_EN_POS            10 /**< LP_CTRL_FAST_WK_EN Position */
#define MXC_F_PWRSEQ_LP_CTRL_FAST_WK_EN                ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_FAST_WK_EN_POS)) /**< LP_CTRL_FAST_WK_EN Mask */

#define MXC_F_PWRSEQ_LP_CTRL_BG_OFF_POS                11 /**< LP_CTRL_BG_OFF Position */
#define MXC_F_PWRSEQ_LP_CTRL_BG_OFF                    ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_BG_OFF_POS)) /**< LP_CTRL_BG_OFF Mask */

#define MXC_F_PWRSEQ_LP_CTRL_VCORE_POR_DIS_POS         12 /**< LP_CTRL_VCORE_POR_DIS Position */
#define MXC_F_PWRSEQ_LP_CTRL_VCORE_POR_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_VCORE_POR_DIS_POS)) /**< LP_CTRL_VCORE_POR_DIS Mask */

#define MXC_F_PWRSEQ_LP_CTRL_LDO_DIS_POS               16 /**< LP_CTRL_LDO_DIS Position */
#define MXC_F_PWRSEQ_LP_CTRL_LDO_DIS                   ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_LDO_DIS_POS)) /**< LP_CTRL_LDO_DIS Mask */

#define MXC_F_PWRSEQ_LP_CTRL_VCORE_SVM_DIS_POS         20 /**< LP_CTRL_VCORE_SVM_DIS Position */
#define MXC_F_PWRSEQ_LP_CTRL_VCORE_SVM_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_VCORE_SVM_DIS_POS)) /**< LP_CTRL_VCORE_SVM_DIS Mask */

#define MXC_F_PWRSEQ_LP_CTRL_VDDIO_POR_DIS_POS         25 /**< LP_CTRL_VDDIO_POR_DIS Position */
#define MXC_F_PWRSEQ_LP_CTRL_VDDIO_POR_DIS             ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LP_CTRL_VDDIO_POR_DIS_POS)) /**< LP_CTRL_VDDIO_POR_DIS Mask */

/**@} end of group PWRSEQ_LP_CTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LP_WAKEFL PWRSEQ_LP_WAKEFL
 * @brief    Low Power Mode Wakeup Flags for GPIO0
 * @{
 */
#define MXC_F_PWRSEQ_LP_WAKEFL_WAKEST_POS              0 /**< LP_WAKEFL_WAKEST Position */
#define MXC_F_PWRSEQ_LP_WAKEFL_WAKEST                  ((uint32_t)(0x3FFFUL << MXC_F_PWRSEQ_LP_WAKEFL_WAKEST_POS)) /**< LP_WAKEFL_WAKEST Mask */

/**@} end of group PWRSEQ_LP_WAKEFL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWK_EN PWRSEQ_LPWK_EN
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWK_EN_WAKEEN_POS                0 /**< LPWK_EN_WAKEEN Position */
#define MXC_F_PWRSEQ_LPWK_EN_WAKEEN                    ((uint32_t)(0x3FFFUL << MXC_F_PWRSEQ_LPWK_EN_WAKEEN_POS)) /**< LPWK_EN_WAKEEN Mask */

/**@} end of group PWRSEQ_LPWK_EN_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPMEMSD PWRSEQ_LPMEMSD
 * @brief    Low Power Memory Shutdown Control.
 * @{
 */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM0_OFF_POS             0 /**< LPMEMSD_SRAM0_OFF Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM0_OFF                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM0_OFF_POS)) /**< LPMEMSD_SRAM0_OFF Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM1_OFF_POS             1 /**< LPMEMSD_SRAM1_OFF Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM1_OFF                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM1_OFF_POS)) /**< LPMEMSD_SRAM1_OFF Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM2_OFF_POS             2 /**< LPMEMSD_SRAM2_OFF Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM2_OFF                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM2_OFF_POS)) /**< LPMEMSD_SRAM2_OFF Mask */

#define MXC_F_PWRSEQ_LPMEMSD_SRAM3_OFF_POS             3 /**< LPMEMSD_SRAM3_OFF Position */
#define MXC_F_PWRSEQ_LPMEMSD_SRAM3_OFF                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPMEMSD_SRAM3_OFF_POS)) /**< LPMEMSD_SRAM3_OFF Mask */

/**@} end of group PWRSEQ_LPMEMSD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32660_INCLUDE_PWRSEQ_REGS_H_
