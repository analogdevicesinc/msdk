/**
 * @file    pt_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PT Peripheral Module.
 * @note    This file is @generated.
 * @ingroup pt_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_PT_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_PT_REGS_H_

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
 * @ingroup     pt
 * @defgroup    pt_registers PT_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the PT Peripheral Module.
 * @details     Pulse Train
 */

/**
 * @ingroup pt_registers
 * Structure type to access the PT Registers.
 */
typedef struct {
    __IO uint32_t rate_length;          /**< <tt>\b 0x0000:</tt> PT RATE_LENGTH Register */
    __IO uint32_t train;                /**< <tt>\b 0x0004:</tt> PT TRAIN Register */
    __IO uint32_t loop;                 /**< <tt>\b 0x0008:</tt> PT LOOP Register */
    __IO uint32_t restart;              /**< <tt>\b 0x000C:</tt> PT RESTART Register */
} mxc_pt_regs_t;

/* Register offsets for module PT */
/**
 * @ingroup    pt_registers
 * @defgroup   PT_Register_Offsets Register Offsets
 * @brief      PT Peripheral Register Offsets from the PT Base Peripheral Address.
 * @{
 */
#define MXC_R_PT_RATE_LENGTH               ((uint32_t)0x00000000UL) /**< Offset from PT Base Address: <tt> 0x0000</tt> */
#define MXC_R_PT_TRAIN                     ((uint32_t)0x00000004UL) /**< Offset from PT Base Address: <tt> 0x0004</tt> */
#define MXC_R_PT_LOOP                      ((uint32_t)0x00000008UL) /**< Offset from PT Base Address: <tt> 0x0008</tt> */
#define MXC_R_PT_RESTART                   ((uint32_t)0x0000000CUL) /**< Offset from PT Base Address: <tt> 0x000C</tt> */
/**@} end of group pt_registers */

/**
 * @ingroup  pt_registers
 * @defgroup PT_RATE_LENGTH PT_RATE_LENGTH
 * @brief    Pulse Train Configuration
 * @{
 */
#define MXC_F_PT_RATE_LENGTH_RATE_CONTROL_POS          0 /**< RATE_LENGTH_RATE_CONTROL Position */
#define MXC_F_PT_RATE_LENGTH_RATE_CONTROL              ((uint32_t)(0x7FFFFFFUL << MXC_F_PT_RATE_LENGTH_RATE_CONTROL_POS)) /**< RATE_LENGTH_RATE_CONTROL Mask */

#define MXC_F_PT_RATE_LENGTH_MODE_POS                  27 /**< RATE_LENGTH_MODE Position */
#define MXC_F_PT_RATE_LENGTH_MODE                      ((uint32_t)(0x1FUL << MXC_F_PT_RATE_LENGTH_MODE_POS)) /**< RATE_LENGTH_MODE Mask */
#define MXC_V_PT_RATE_LENGTH_MODE_32_BIT               ((uint32_t)0x0UL) /**< RATE_LENGTH_MODE_32_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_32_BIT               (MXC_V_PT_RATE_LENGTH_MODE_32_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_32_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_SQUARE_WAVE          ((uint32_t)0x1UL) /**< RATE_LENGTH_MODE_SQUARE_WAVE Value */
#define MXC_S_PT_RATE_LENGTH_MODE_SQUARE_WAVE          (MXC_V_PT_RATE_LENGTH_MODE_SQUARE_WAVE << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_SQUARE_WAVE Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_2_BIT                ((uint32_t)0x2UL) /**< RATE_LENGTH_MODE_2_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_2_BIT                (MXC_V_PT_RATE_LENGTH_MODE_2_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_2_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_3_BIT                ((uint32_t)0x3UL) /**< RATE_LENGTH_MODE_3_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_3_BIT                (MXC_V_PT_RATE_LENGTH_MODE_3_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_3_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_4_BIT                ((uint32_t)0x4UL) /**< RATE_LENGTH_MODE_4_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_4_BIT                (MXC_V_PT_RATE_LENGTH_MODE_4_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_4_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_5_BIT                ((uint32_t)0x5UL) /**< RATE_LENGTH_MODE_5_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_5_BIT                (MXC_V_PT_RATE_LENGTH_MODE_5_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_5_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_6_BIT                ((uint32_t)0x6UL) /**< RATE_LENGTH_MODE_6_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_6_BIT                (MXC_V_PT_RATE_LENGTH_MODE_6_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_6_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_7_BIT                ((uint32_t)0x7UL) /**< RATE_LENGTH_MODE_7_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_7_BIT                (MXC_V_PT_RATE_LENGTH_MODE_7_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_7_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_8_BIT                ((uint32_t)0x8UL) /**< RATE_LENGTH_MODE_8_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_8_BIT                (MXC_V_PT_RATE_LENGTH_MODE_8_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_8_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_9_BIT                ((uint32_t)0x9UL) /**< RATE_LENGTH_MODE_9_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_9_BIT                (MXC_V_PT_RATE_LENGTH_MODE_9_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_9_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_10_BIT               ((uint32_t)0xAUL) /**< RATE_LENGTH_MODE_10_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_10_BIT               (MXC_V_PT_RATE_LENGTH_MODE_10_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_10_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_11_BIT               ((uint32_t)0xBUL) /**< RATE_LENGTH_MODE_11_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_11_BIT               (MXC_V_PT_RATE_LENGTH_MODE_11_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_11_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_12_BIT               ((uint32_t)0xCUL) /**< RATE_LENGTH_MODE_12_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_12_BIT               (MXC_V_PT_RATE_LENGTH_MODE_12_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_12_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_13_BIT               ((uint32_t)0xDUL) /**< RATE_LENGTH_MODE_13_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_13_BIT               (MXC_V_PT_RATE_LENGTH_MODE_13_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_13_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_14_BIT               ((uint32_t)0xEUL) /**< RATE_LENGTH_MODE_14_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_14_BIT               (MXC_V_PT_RATE_LENGTH_MODE_14_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_14_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_15_BIT               ((uint32_t)0xFUL) /**< RATE_LENGTH_MODE_15_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_15_BIT               (MXC_V_PT_RATE_LENGTH_MODE_15_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_15_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_16_BIT               ((uint32_t)0x10UL) /**< RATE_LENGTH_MODE_16_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_16_BIT               (MXC_V_PT_RATE_LENGTH_MODE_16_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_16_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_17_BIT               ((uint32_t)0x11UL) /**< RATE_LENGTH_MODE_17_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_17_BIT               (MXC_V_PT_RATE_LENGTH_MODE_17_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_17_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_18_BIT               ((uint32_t)0x12UL) /**< RATE_LENGTH_MODE_18_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_18_BIT               (MXC_V_PT_RATE_LENGTH_MODE_18_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_18_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_19_BIT               ((uint32_t)0x13UL) /**< RATE_LENGTH_MODE_19_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_19_BIT               (MXC_V_PT_RATE_LENGTH_MODE_19_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_19_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_20_BIT               ((uint32_t)0x14UL) /**< RATE_LENGTH_MODE_20_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_20_BIT               (MXC_V_PT_RATE_LENGTH_MODE_20_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_20_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_21_BIT               ((uint32_t)0x15UL) /**< RATE_LENGTH_MODE_21_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_21_BIT               (MXC_V_PT_RATE_LENGTH_MODE_21_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_21_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_22_BIT               ((uint32_t)0x16UL) /**< RATE_LENGTH_MODE_22_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_22_BIT               (MXC_V_PT_RATE_LENGTH_MODE_22_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_22_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_23_BIT               ((uint32_t)0x17UL) /**< RATE_LENGTH_MODE_23_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_23_BIT               (MXC_V_PT_RATE_LENGTH_MODE_23_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_23_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_24_BIT               ((uint32_t)0x18UL) /**< RATE_LENGTH_MODE_24_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_24_BIT               (MXC_V_PT_RATE_LENGTH_MODE_24_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_24_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_25_BIT               ((uint32_t)0x19UL) /**< RATE_LENGTH_MODE_25_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_25_BIT               (MXC_V_PT_RATE_LENGTH_MODE_25_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_25_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_26_BIT               ((uint32_t)0x1AUL) /**< RATE_LENGTH_MODE_26_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_26_BIT               (MXC_V_PT_RATE_LENGTH_MODE_26_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_26_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_27_BIT               ((uint32_t)0x1BUL) /**< RATE_LENGTH_MODE_27_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_27_BIT               (MXC_V_PT_RATE_LENGTH_MODE_27_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_27_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_28_BIT               ((uint32_t)0x1CUL) /**< RATE_LENGTH_MODE_28_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_28_BIT               (MXC_V_PT_RATE_LENGTH_MODE_28_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_28_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_29_BIT               ((uint32_t)0x1DUL) /**< RATE_LENGTH_MODE_29_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_29_BIT               (MXC_V_PT_RATE_LENGTH_MODE_29_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_29_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_30_BIT               ((uint32_t)0x1EUL) /**< RATE_LENGTH_MODE_30_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_30_BIT               (MXC_V_PT_RATE_LENGTH_MODE_30_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_30_BIT Setting */
#define MXC_V_PT_RATE_LENGTH_MODE_31_BIT               ((uint32_t)0x1FUL) /**< RATE_LENGTH_MODE_31_BIT Value */
#define MXC_S_PT_RATE_LENGTH_MODE_31_BIT               (MXC_V_PT_RATE_LENGTH_MODE_31_BIT << MXC_F_PT_RATE_LENGTH_MODE_POS) /**< RATE_LENGTH_MODE_31_BIT Setting */

/**@} end of group PT_RATE_LENGTH_Register */

/**
 * @ingroup  pt_registers
 * @defgroup PT_LOOP PT_LOOP
 * @brief    Pulse Train Loop Count
 * @{
 */
#define MXC_F_PT_LOOP_COUNT_POS                        0 /**< LOOP_COUNT Position */
#define MXC_F_PT_LOOP_COUNT                            ((uint32_t)(0xFFFFUL << MXC_F_PT_LOOP_COUNT_POS)) /**< LOOP_COUNT Mask */

#define MXC_F_PT_LOOP_DELAY_POS                        16 /**< LOOP_DELAY Position */
#define MXC_F_PT_LOOP_DELAY                            ((uint32_t)(0xFFFUL << MXC_F_PT_LOOP_DELAY_POS)) /**< LOOP_DELAY Mask */

/**@} end of group PT_LOOP_Register */

/**
 * @ingroup  pt_registers
 * @defgroup PT_RESTART PT_RESTART
 * @brief     Pulse Train Auto-Restart Configuration.
 * @{
 */
#define MXC_F_PT_RESTART_PT_X_SELECT_POS               0 /**< RESTART_PT_X_SELECT Position */
#define MXC_F_PT_RESTART_PT_X_SELECT                   ((uint32_t)(0x1FUL << MXC_F_PT_RESTART_PT_X_SELECT_POS)) /**< RESTART_PT_X_SELECT Mask */

#define MXC_F_PT_RESTART_ON_PT_X_LOOP_EXIT_POS         7 /**< RESTART_ON_PT_X_LOOP_EXIT Position */
#define MXC_F_PT_RESTART_ON_PT_X_LOOP_EXIT             ((uint32_t)(0x1UL << MXC_F_PT_RESTART_ON_PT_X_LOOP_EXIT_POS)) /**< RESTART_ON_PT_X_LOOP_EXIT Mask */

#define MXC_F_PT_RESTART_PT_Y_SELECT_POS               8 /**< RESTART_PT_Y_SELECT Position */
#define MXC_F_PT_RESTART_PT_Y_SELECT                   ((uint32_t)(0x1FUL << MXC_F_PT_RESTART_PT_Y_SELECT_POS)) /**< RESTART_PT_Y_SELECT Mask */

#define MXC_F_PT_RESTART_ON_PT_Y_LOOP_EXIT_POS         15 /**< RESTART_ON_PT_Y_LOOP_EXIT Position */
#define MXC_F_PT_RESTART_ON_PT_Y_LOOP_EXIT             ((uint32_t)(0x1UL << MXC_F_PT_RESTART_ON_PT_Y_LOOP_EXIT_POS)) /**< RESTART_ON_PT_Y_LOOP_EXIT Mask */

/**@} end of group PT_RESTART_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32655_INCLUDE_PT_REGS_H_
