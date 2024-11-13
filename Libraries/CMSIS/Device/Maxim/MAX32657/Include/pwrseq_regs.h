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
    __IO uint32_t lpctrl;               /**< <tt>\b 0x00:</tt> PWRSEQ LPCTRL Register */
    __IO uint32_t lpwkfl0;              /**< <tt>\b 0x04:</tt> PWRSEQ LPWKFL0 Register */
    __IO uint32_t lpwken0;              /**< <tt>\b 0x08:</tt> PWRSEQ LPWKEN0 Register */
    __R  uint32_t rsv_0xc_0x2f[9];
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
#define MXC_R_PWRSEQ_LPCTRL                ((uint32_t)0x00000000UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0000</tt> */
#define MXC_R_PWRSEQ_LPWKFL0               ((uint32_t)0x00000004UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0004</tt> */
#define MXC_R_PWRSEQ_LPWKEN0               ((uint32_t)0x00000008UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0008</tt> */
#define MXC_R_PWRSEQ_LPPWST                ((uint32_t)0x00000030UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0030</tt> */
#define MXC_R_PWRSEQ_GP0                   ((uint32_t)0x00000048UL) /**< Offset from PWRSEQ Base Address: <tt> 0x0048</tt> */
#define MXC_R_PWRSEQ_GP1                   ((uint32_t)0x0000004CUL) /**< Offset from PWRSEQ Base Address: <tt> 0x004C</tt> */
/**@} end of group pwrseq_registers */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPCTRL PWRSEQ_LPCTRL
 * @brief    Low Power Control Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPCTRL_SRAMRET_EN_POS             0 /**< LPCTRL_SRAMRET_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_SRAMRET_EN                 ((uint32_t)(0x1FUL << MXC_F_PWRSEQ_LPCTRL_SRAMRET_EN_POS)) /**< LPCTRL_SRAMRET_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS                 11 /**< LPCTRL_BG_DIS Position */
#define MXC_F_PWRSEQ_LPCTRL_BG_DIS                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_BG_DIS_POS)) /**< LPCTRL_BG_DIS Mask */

#define MXC_F_PWRSEQ_LPCTRL_RETLDO_EN_POS              12 /**< LPCTRL_RETLDO_EN Position */
#define MXC_F_PWRSEQ_LPCTRL_RETLDO_EN                  ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_RETLDO_EN_POS)) /**< LPCTRL_RETLDO_EN Mask */

#define MXC_F_PWRSEQ_LPCTRL_LDO_EN_DLY_POS             13 /**< LPCTRL_LDO_EN_DLY Position */
#define MXC_F_PWRSEQ_LPCTRL_LDO_EN_DLY                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_LDO_EN_DLY_POS)) /**< LPCTRL_LDO_EN_DLY Mask */

#define MXC_F_PWRSEQ_LPCTRL_LPWKFL_CLR_POS             31 /**< LPCTRL_LPWKFL_CLR Position */
#define MXC_F_PWRSEQ_LPCTRL_LPWKFL_CLR                 ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPCTRL_LPWKFL_CLR_POS)) /**< LPCTRL_LPWKFL_CLR Mask */

/**@} end of group PWRSEQ_LPCTRL_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKFL0 PWRSEQ_LPWKFL0
 * @brief    Low Power I/O Wakeup Status Register 0. This register indicates the low power
 *           wakeup status for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKFL0_PINS_POS                  0 /**< LPWKFL0_PINS Position */
#define MXC_F_PWRSEQ_LPWKFL0_PINS                      ((uint32_t)(0xFFFUL << MXC_F_PWRSEQ_LPWKFL0_PINS_POS)) /**< LPWKFL0_PINS Mask */

/**@} end of group PWRSEQ_LPWKFL0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPWKEN0 PWRSEQ_LPWKEN0
 * @brief    Low Power I/O Wakeup Enable Register 0. This register enables low power wakeup
 *           functionality for GPIO0.
 * @{
 */
#define MXC_F_PWRSEQ_LPWKEN0_PINS_POS                  0 /**< LPWKEN0_PINS Position */
#define MXC_F_PWRSEQ_LPWKEN0_PINS                      ((uint32_t)(0xFFFUL << MXC_F_PWRSEQ_LPWKEN0_PINS_POS)) /**< LPWKEN0_PINS Mask */

/**@} end of group PWRSEQ_LPWKEN0_Register */

/**
 * @ingroup  pwrseq_registers
 * @defgroup PWRSEQ_LPPWST PWRSEQ_LPPWST
 * @brief    Low Power Peripheral Wakeup Status Register.
 * @{
 */
#define MXC_F_PWRSEQ_LPPWST_BACKUP_POS                 16 /**< LPPWST_BACKUP Position */
#define MXC_F_PWRSEQ_LPPWST_BACKUP                     ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_BACKUP_POS)) /**< LPPWST_BACKUP Mask */

#define MXC_F_PWRSEQ_LPPWST_RESET_POS                  17 /**< LPPWST_RESET Position */
#define MXC_F_PWRSEQ_LPPWST_RESET                      ((uint32_t)(0x1UL << MXC_F_PWRSEQ_LPPWST_RESET_POS)) /**< LPPWST_RESET Mask */

/**@} end of group PWRSEQ_LPPWST_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_PWRSEQ_REGS_H_
