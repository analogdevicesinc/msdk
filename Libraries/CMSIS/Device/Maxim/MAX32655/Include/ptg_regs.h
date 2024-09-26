/**
 * @file    ptg_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PTG Peripheral Module.
 * @note    This file is @generated.
 * @ingroup ptg_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_PTG_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_PTG_REGS_H_

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
 * @ingroup     ptg
 * @defgroup    ptg_registers PTG_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the PTG Peripheral Module.
 * @details     Pulse Train Generation
 */

/**
 * @ingroup ptg_registers
 * Structure type to access the PTG Registers.
 */
typedef struct {
    __IO uint32_t enable;               /**< <tt>\b 0x0000:</tt> PTG ENABLE Register */
    __IO uint32_t resync;               /**< <tt>\b 0x0004:</tt> PTG RESYNC Register */
    __IO uint32_t stop_intfl;           /**< <tt>\b 0x0008:</tt> PTG STOP_INTFL Register */
    __IO uint32_t stop_inten;           /**< <tt>\b 0x000C:</tt> PTG STOP_INTEN Register */
    __O  uint32_t safe_en;              /**< <tt>\b 0x0010:</tt> PTG SAFE_EN Register */
    __O  uint32_t safe_dis;             /**< <tt>\b 0x0014:</tt> PTG SAFE_DIS Register */
    __IO uint32_t ready_intfl;          /**< <tt>\b 0x0018:</tt> PTG READY_INTFL Register */
    __IO uint32_t ready_inten;          /**< <tt>\b 0x001C:</tt> PTG READY_INTEN Register */
} mxc_ptg_regs_t;

/* Register offsets for module PTG */
/**
 * @ingroup    ptg_registers
 * @defgroup   PTG_Register_Offsets Register Offsets
 * @brief      PTG Peripheral Register Offsets from the PTG Base Peripheral Address.
 * @{
 */
#define MXC_R_PTG_ENABLE                   ((uint32_t)0x00000000UL) /**< Offset from PTG Base Address: <tt> 0x0000</tt> */
#define MXC_R_PTG_RESYNC                   ((uint32_t)0x00000004UL) /**< Offset from PTG Base Address: <tt> 0x0004</tt> */
#define MXC_R_PTG_STOP_INTFL               ((uint32_t)0x00000008UL) /**< Offset from PTG Base Address: <tt> 0x0008</tt> */
#define MXC_R_PTG_STOP_INTEN               ((uint32_t)0x0000000CUL) /**< Offset from PTG Base Address: <tt> 0x000C</tt> */
#define MXC_R_PTG_SAFE_EN                  ((uint32_t)0x00000010UL) /**< Offset from PTG Base Address: <tt> 0x0010</tt> */
#define MXC_R_PTG_SAFE_DIS                 ((uint32_t)0x00000014UL) /**< Offset from PTG Base Address: <tt> 0x0014</tt> */
#define MXC_R_PTG_READY_INTFL              ((uint32_t)0x00000018UL) /**< Offset from PTG Base Address: <tt> 0x0018</tt> */
#define MXC_R_PTG_READY_INTEN              ((uint32_t)0x0000001CUL) /**< Offset from PTG Base Address: <tt> 0x001C</tt> */
/**@} end of group ptg_registers */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_ENABLE PTG_ENABLE
 * @brief    Global Enable/Disable Controls for All Pulse Trains
 * @{
 */
#define MXC_F_PTG_ENABLE_PT0_POS                       0 /**< ENABLE_PT0 Position */
#define MXC_F_PTG_ENABLE_PT0                           ((uint32_t)(0x1UL << MXC_F_PTG_ENABLE_PT0_POS)) /**< ENABLE_PT0 Mask */

#define MXC_F_PTG_ENABLE_PT1_POS                       1 /**< ENABLE_PT1 Position */
#define MXC_F_PTG_ENABLE_PT1                           ((uint32_t)(0x1UL << MXC_F_PTG_ENABLE_PT1_POS)) /**< ENABLE_PT1 Mask */

#define MXC_F_PTG_ENABLE_PT2_POS                       2 /**< ENABLE_PT2 Position */
#define MXC_F_PTG_ENABLE_PT2                           ((uint32_t)(0x1UL << MXC_F_PTG_ENABLE_PT2_POS)) /**< ENABLE_PT2 Mask */

#define MXC_F_PTG_ENABLE_PT3_POS                       3 /**< ENABLE_PT3 Position */
#define MXC_F_PTG_ENABLE_PT3                           ((uint32_t)(0x1UL << MXC_F_PTG_ENABLE_PT3_POS)) /**< ENABLE_PT3 Mask */

/**@} end of group PTG_ENABLE_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_RESYNC PTG_RESYNC
 * @brief    Global Resync (All Pulse Trains) Control
 * @{
 */
#define MXC_F_PTG_RESYNC_PT0_POS                       0 /**< RESYNC_PT0 Position */
#define MXC_F_PTG_RESYNC_PT0                           ((uint32_t)(0x1UL << MXC_F_PTG_RESYNC_PT0_POS)) /**< RESYNC_PT0 Mask */

#define MXC_F_PTG_RESYNC_PT1_POS                       1 /**< RESYNC_PT1 Position */
#define MXC_F_PTG_RESYNC_PT1                           ((uint32_t)(0x1UL << MXC_F_PTG_RESYNC_PT1_POS)) /**< RESYNC_PT1 Mask */

#define MXC_F_PTG_RESYNC_PT2_POS                       2 /**< RESYNC_PT2 Position */
#define MXC_F_PTG_RESYNC_PT2                           ((uint32_t)(0x1UL << MXC_F_PTG_RESYNC_PT2_POS)) /**< RESYNC_PT2 Mask */

#define MXC_F_PTG_RESYNC_PT3_POS                       3 /**< RESYNC_PT3 Position */
#define MXC_F_PTG_RESYNC_PT3                           ((uint32_t)(0x1UL << MXC_F_PTG_RESYNC_PT3_POS)) /**< RESYNC_PT3 Mask */

/**@} end of group PTG_RESYNC_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_STOP_INTFL PTG_STOP_INTFL
 * @brief    Pulse Train Stop Interrupt Flags
 * @{
 */
#define MXC_F_PTG_STOP_INTFL_PT0_POS                   0 /**< STOP_INTFL_PT0 Position */
#define MXC_F_PTG_STOP_INTFL_PT0                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTFL_PT0_POS)) /**< STOP_INTFL_PT0 Mask */

#define MXC_F_PTG_STOP_INTFL_PT1_POS                   1 /**< STOP_INTFL_PT1 Position */
#define MXC_F_PTG_STOP_INTFL_PT1                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTFL_PT1_POS)) /**< STOP_INTFL_PT1 Mask */

#define MXC_F_PTG_STOP_INTFL_PT2_POS                   2 /**< STOP_INTFL_PT2 Position */
#define MXC_F_PTG_STOP_INTFL_PT2                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTFL_PT2_POS)) /**< STOP_INTFL_PT2 Mask */

#define MXC_F_PTG_STOP_INTFL_PT3_POS                   3 /**< STOP_INTFL_PT3 Position */
#define MXC_F_PTG_STOP_INTFL_PT3                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTFL_PT3_POS)) /**< STOP_INTFL_PT3 Mask */

/**@} end of group PTG_STOP_INTFL_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_STOP_INTEN PTG_STOP_INTEN
 * @brief    Pulse Train Stop Interrupt Enable/Disable
 * @{
 */
#define MXC_F_PTG_STOP_INTEN_PT0_POS                   0 /**< STOP_INTEN_PT0 Position */
#define MXC_F_PTG_STOP_INTEN_PT0                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTEN_PT0_POS)) /**< STOP_INTEN_PT0 Mask */

#define MXC_F_PTG_STOP_INTEN_PT1_POS                   1 /**< STOP_INTEN_PT1 Position */
#define MXC_F_PTG_STOP_INTEN_PT1                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTEN_PT1_POS)) /**< STOP_INTEN_PT1 Mask */

#define MXC_F_PTG_STOP_INTEN_PT2_POS                   2 /**< STOP_INTEN_PT2 Position */
#define MXC_F_PTG_STOP_INTEN_PT2                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTEN_PT2_POS)) /**< STOP_INTEN_PT2 Mask */

#define MXC_F_PTG_STOP_INTEN_PT3_POS                   3 /**< STOP_INTEN_PT3 Position */
#define MXC_F_PTG_STOP_INTEN_PT3                       ((uint32_t)(0x1UL << MXC_F_PTG_STOP_INTEN_PT3_POS)) /**< STOP_INTEN_PT3 Mask */

/**@} end of group PTG_STOP_INTEN_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_SAFE_EN PTG_SAFE_EN
 * @brief    Pulse Train Global Safe Enable.
 * @{
 */
#define MXC_F_PTG_SAFE_EN_PT0_POS                      0 /**< SAFE_EN_PT0 Position */
#define MXC_F_PTG_SAFE_EN_PT0                          ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_EN_PT0_POS)) /**< SAFE_EN_PT0 Mask */

#define MXC_F_PTG_SAFE_EN_PT1_POS                      1 /**< SAFE_EN_PT1 Position */
#define MXC_F_PTG_SAFE_EN_PT1                          ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_EN_PT1_POS)) /**< SAFE_EN_PT1 Mask */

#define MXC_F_PTG_SAFE_EN_PT2_POS                      2 /**< SAFE_EN_PT2 Position */
#define MXC_F_PTG_SAFE_EN_PT2                          ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_EN_PT2_POS)) /**< SAFE_EN_PT2 Mask */

#define MXC_F_PTG_SAFE_EN_PT3_POS                      3 /**< SAFE_EN_PT3 Position */
#define MXC_F_PTG_SAFE_EN_PT3                          ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_EN_PT3_POS)) /**< SAFE_EN_PT3 Mask */

/**@} end of group PTG_SAFE_EN_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_SAFE_DIS PTG_SAFE_DIS
 * @brief    Pulse Train Global Safe Disable.
 * @{
 */
#define MXC_F_PTG_SAFE_DIS_PT0_POS                     0 /**< SAFE_DIS_PT0 Position */
#define MXC_F_PTG_SAFE_DIS_PT0                         ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_DIS_PT0_POS)) /**< SAFE_DIS_PT0 Mask */

#define MXC_F_PTG_SAFE_DIS_PT1_POS                     1 /**< SAFE_DIS_PT1 Position */
#define MXC_F_PTG_SAFE_DIS_PT1                         ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_DIS_PT1_POS)) /**< SAFE_DIS_PT1 Mask */

#define MXC_F_PTG_SAFE_DIS_PT2_POS                     2 /**< SAFE_DIS_PT2 Position */
#define MXC_F_PTG_SAFE_DIS_PT2                         ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_DIS_PT2_POS)) /**< SAFE_DIS_PT2 Mask */

#define MXC_F_PTG_SAFE_DIS_PT3_POS                     3 /**< SAFE_DIS_PT3 Position */
#define MXC_F_PTG_SAFE_DIS_PT3                         ((uint32_t)(0x1UL << MXC_F_PTG_SAFE_DIS_PT3_POS)) /**< SAFE_DIS_PT3 Mask */

/**@} end of group PTG_SAFE_DIS_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_READY_INTFL PTG_READY_INTFL
 * @brief    Pulse Train Ready Interrupt Flags
 * @{
 */
#define MXC_F_PTG_READY_INTFL_PT0_POS                  0 /**< READY_INTFL_PT0 Position */
#define MXC_F_PTG_READY_INTFL_PT0                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTFL_PT0_POS)) /**< READY_INTFL_PT0 Mask */

#define MXC_F_PTG_READY_INTFL_PT1_POS                  1 /**< READY_INTFL_PT1 Position */
#define MXC_F_PTG_READY_INTFL_PT1                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTFL_PT1_POS)) /**< READY_INTFL_PT1 Mask */

#define MXC_F_PTG_READY_INTFL_PT2_POS                  2 /**< READY_INTFL_PT2 Position */
#define MXC_F_PTG_READY_INTFL_PT2                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTFL_PT2_POS)) /**< READY_INTFL_PT2 Mask */

#define MXC_F_PTG_READY_INTFL_PT3_POS                  3 /**< READY_INTFL_PT3 Position */
#define MXC_F_PTG_READY_INTFL_PT3                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTFL_PT3_POS)) /**< READY_INTFL_PT3 Mask */

/**@} end of group PTG_READY_INTFL_Register */

/**
 * @ingroup  ptg_registers
 * @defgroup PTG_READY_INTEN PTG_READY_INTEN
 * @brief    Pulse Train Ready Interrupt Enable/Disable
 * @{
 */
#define MXC_F_PTG_READY_INTEN_PT0_POS                  0 /**< READY_INTEN_PT0 Position */
#define MXC_F_PTG_READY_INTEN_PT0                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTEN_PT0_POS)) /**< READY_INTEN_PT0 Mask */

#define MXC_F_PTG_READY_INTEN_PT1_POS                  1 /**< READY_INTEN_PT1 Position */
#define MXC_F_PTG_READY_INTEN_PT1                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTEN_PT1_POS)) /**< READY_INTEN_PT1 Mask */

#define MXC_F_PTG_READY_INTEN_PT2_POS                  2 /**< READY_INTEN_PT2 Position */
#define MXC_F_PTG_READY_INTEN_PT2                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTEN_PT2_POS)) /**< READY_INTEN_PT2 Mask */

#define MXC_F_PTG_READY_INTEN_PT3_POS                  3 /**< READY_INTEN_PT3 Position */
#define MXC_F_PTG_READY_INTEN_PT3                      ((uint32_t)(0x1UL << MXC_F_PTG_READY_INTEN_PT3_POS)) /**< READY_INTEN_PT3 Mask */

/**@} end of group PTG_READY_INTEN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_PTG_REGS_H_
