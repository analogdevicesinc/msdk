/**
 * @file    qdec_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the QDEC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup qdec_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_QDEC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_QDEC_REGS_H_

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
 * @ingroup     qdec
 * @defgroup    qdec_registers QDEC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the QDEC Peripheral Module.
 * @details     Quadrature Encoder Interface
 */

/**
 * @ingroup qdec_registers
 * Structure type to access the QDEC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> QDEC CTRL Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0004:</tt> QDEC INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x0008:</tt> QDEC INTEN Register */
    __IO uint32_t maxcnt;               /**< <tt>\b 0x000C:</tt> QDEC MAXCNT Register */
    __IO uint32_t initial;              /**< <tt>\b 0x0010:</tt> QDEC INITIAL Register */
    __IO uint32_t compare;              /**< <tt>\b 0x0014:</tt> QDEC COMPARE Register */
    __I  uint32_t index;                /**< <tt>\b 0x0018:</tt> QDEC INDEX Register */
    __I  uint32_t capture;              /**< <tt>\b 0x001C:</tt> QDEC CAPTURE Register */
    __I  uint32_t status;               /**< <tt>\b 0x0020:</tt> QDEC STATUS Register */
    __IO uint32_t position;             /**< <tt>\b 0x0024:</tt> QDEC POSITION Register */
    __IO uint32_t capdly;               /**< <tt>\b 0x0028:</tt> QDEC CAPDLY Register */
} mxc_qdec_regs_t;

/* Register offsets for module QDEC */
/**
 * @ingroup    qdec_registers
 * @defgroup   QDEC_Register_Offsets Register Offsets
 * @brief      QDEC Peripheral Register Offsets from the QDEC Base Peripheral Address.
 * @{
 */
#define MXC_R_QDEC_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from QDEC Base Address: <tt> 0x0000</tt> */
#define MXC_R_QDEC_INTFL                   ((uint32_t)0x00000004UL) /**< Offset from QDEC Base Address: <tt> 0x0004</tt> */
#define MXC_R_QDEC_INTEN                   ((uint32_t)0x00000008UL) /**< Offset from QDEC Base Address: <tt> 0x0008</tt> */
#define MXC_R_QDEC_MAXCNT                  ((uint32_t)0x0000000CUL) /**< Offset from QDEC Base Address: <tt> 0x000C</tt> */
#define MXC_R_QDEC_INITIAL                 ((uint32_t)0x00000010UL) /**< Offset from QDEC Base Address: <tt> 0x0010</tt> */
#define MXC_R_QDEC_COMPARE                 ((uint32_t)0x00000014UL) /**< Offset from QDEC Base Address: <tt> 0x0014</tt> */
#define MXC_R_QDEC_INDEX                   ((uint32_t)0x00000018UL) /**< Offset from QDEC Base Address: <tt> 0x0018</tt> */
#define MXC_R_QDEC_CAPTURE                 ((uint32_t)0x0000001CUL) /**< Offset from QDEC Base Address: <tt> 0x001C</tt> */
#define MXC_R_QDEC_STATUS                  ((uint32_t)0x00000020UL) /**< Offset from QDEC Base Address: <tt> 0x0020</tt> */
#define MXC_R_QDEC_POSITION                ((uint32_t)0x00000024UL) /**< Offset from QDEC Base Address: <tt> 0x0024</tt> */
#define MXC_R_QDEC_CAPDLY                  ((uint32_t)0x00000028UL) /**< Offset from QDEC Base Address: <tt> 0x0028</tt> */
/**@} end of group qdec_registers */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_CTRL QDEC_CTRL
 * @brief    Control Register.
 * @{
 */
#define MXC_F_QDEC_CTRL_EN_POS                         0 /**< CTRL_EN Position */
#define MXC_F_QDEC_CTRL_EN                             ((uint32_t)(0x1UL << MXC_F_QDEC_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_QDEC_CTRL_MODE_POS                       1 /**< CTRL_MODE Position */
#define MXC_F_QDEC_CTRL_MODE                           ((uint32_t)(0x3UL << MXC_F_QDEC_CTRL_MODE_POS)) /**< CTRL_MODE Mask */
#define MXC_V_QDEC_CTRL_MODE_X1MODE                    ((uint32_t)0x0UL) /**< CTRL_MODE_X1MODE Value */
#define MXC_S_QDEC_CTRL_MODE_X1MODE                    (MXC_V_QDEC_CTRL_MODE_X1MODE << MXC_F_QDEC_CTRL_MODE_POS) /**< CTRL_MODE_X1MODE Setting */
#define MXC_V_QDEC_CTRL_MODE_X2MODE                    ((uint32_t)0x1UL) /**< CTRL_MODE_X2MODE Value */
#define MXC_S_QDEC_CTRL_MODE_X2MODE                    (MXC_V_QDEC_CTRL_MODE_X2MODE << MXC_F_QDEC_CTRL_MODE_POS) /**< CTRL_MODE_X2MODE Setting */
#define MXC_V_QDEC_CTRL_MODE_X4MODE                    ((uint32_t)0x2UL) /**< CTRL_MODE_X4MODE Value */
#define MXC_S_QDEC_CTRL_MODE_X4MODE                    (MXC_V_QDEC_CTRL_MODE_X4MODE << MXC_F_QDEC_CTRL_MODE_POS) /**< CTRL_MODE_X4MODE Setting */

#define MXC_F_QDEC_CTRL_SWAP_POS                       3 /**< CTRL_SWAP Position */
#define MXC_F_QDEC_CTRL_SWAP                           ((uint32_t)(0x1UL << MXC_F_QDEC_CTRL_SWAP_POS)) /**< CTRL_SWAP Mask */

#define MXC_F_QDEC_CTRL_FILTER_POS                     4 /**< CTRL_FILTER Position */
#define MXC_F_QDEC_CTRL_FILTER                         ((uint32_t)(0x3UL << MXC_F_QDEC_CTRL_FILTER_POS)) /**< CTRL_FILTER Mask */
#define MXC_V_QDEC_CTRL_FILTER_1_SAMPLE                ((uint32_t)0x0UL) /**< CTRL_FILTER_1_SAMPLE Value */
#define MXC_S_QDEC_CTRL_FILTER_1_SAMPLE                (MXC_V_QDEC_CTRL_FILTER_1_SAMPLE << MXC_F_QDEC_CTRL_FILTER_POS) /**< CTRL_FILTER_1_SAMPLE Setting */
#define MXC_V_QDEC_CTRL_FILTER_2_SAMPLES               ((uint32_t)0x1UL) /**< CTRL_FILTER_2_SAMPLES Value */
#define MXC_S_QDEC_CTRL_FILTER_2_SAMPLES               (MXC_V_QDEC_CTRL_FILTER_2_SAMPLES << MXC_F_QDEC_CTRL_FILTER_POS) /**< CTRL_FILTER_2_SAMPLES Setting */
#define MXC_V_QDEC_CTRL_FILTER_3_SAMPLES               ((uint32_t)0x2UL) /**< CTRL_FILTER_3_SAMPLES Value */
#define MXC_S_QDEC_CTRL_FILTER_3_SAMPLES               (MXC_V_QDEC_CTRL_FILTER_3_SAMPLES << MXC_F_QDEC_CTRL_FILTER_POS) /**< CTRL_FILTER_3_SAMPLES Setting */
#define MXC_V_QDEC_CTRL_FILTER_4_SAMPLES               ((uint32_t)0x3UL) /**< CTRL_FILTER_4_SAMPLES Value */
#define MXC_S_QDEC_CTRL_FILTER_4_SAMPLES               (MXC_V_QDEC_CTRL_FILTER_4_SAMPLES << MXC_F_QDEC_CTRL_FILTER_POS) /**< CTRL_FILTER_4_SAMPLES Setting */

#define MXC_F_QDEC_CTRL_RST_INDEX_POS                  6 /**< CTRL_RST_INDEX Position */
#define MXC_F_QDEC_CTRL_RST_INDEX                      ((uint32_t)(0x1UL << MXC_F_QDEC_CTRL_RST_INDEX_POS)) /**< CTRL_RST_INDEX Mask */

#define MXC_F_QDEC_CTRL_RST_MAXCNT_POS                 7 /**< CTRL_RST_MAXCNT Position */
#define MXC_F_QDEC_CTRL_RST_MAXCNT                     ((uint32_t)(0x1UL << MXC_F_QDEC_CTRL_RST_MAXCNT_POS)) /**< CTRL_RST_MAXCNT Mask */

#define MXC_F_QDEC_CTRL_STICKY_POS                     8 /**< CTRL_STICKY Position */
#define MXC_F_QDEC_CTRL_STICKY                         ((uint32_t)(0x1UL << MXC_F_QDEC_CTRL_STICKY_POS)) /**< CTRL_STICKY Mask */

#define MXC_F_QDEC_CTRL_PSC_POS                        16 /**< CTRL_PSC Position */
#define MXC_F_QDEC_CTRL_PSC                            ((uint32_t)(0x7UL << MXC_F_QDEC_CTRL_PSC_POS)) /**< CTRL_PSC Mask */
#define MXC_V_QDEC_CTRL_PSC_DIV1                       ((uint32_t)0x0UL) /**< CTRL_PSC_DIV1 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV1                       (MXC_V_QDEC_CTRL_PSC_DIV1 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV1 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV2                       ((uint32_t)0x1UL) /**< CTRL_PSC_DIV2 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV2                       (MXC_V_QDEC_CTRL_PSC_DIV2 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV2 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV4                       ((uint32_t)0x2UL) /**< CTRL_PSC_DIV4 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV4                       (MXC_V_QDEC_CTRL_PSC_DIV4 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV4 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV8                       ((uint32_t)0x3UL) /**< CTRL_PSC_DIV8 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV8                       (MXC_V_QDEC_CTRL_PSC_DIV8 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV8 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV16                      ((uint32_t)0x4UL) /**< CTRL_PSC_DIV16 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV16                      (MXC_V_QDEC_CTRL_PSC_DIV16 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV16 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV32                      ((uint32_t)0x5UL) /**< CTRL_PSC_DIV32 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV32                      (MXC_V_QDEC_CTRL_PSC_DIV32 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV32 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV64                      ((uint32_t)0x6UL) /**< CTRL_PSC_DIV64 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV64                      (MXC_V_QDEC_CTRL_PSC_DIV64 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV64 Setting */
#define MXC_V_QDEC_CTRL_PSC_DIV128                     ((uint32_t)0x7UL) /**< CTRL_PSC_DIV128 Value */
#define MXC_S_QDEC_CTRL_PSC_DIV128                     (MXC_V_QDEC_CTRL_PSC_DIV128 << MXC_F_QDEC_CTRL_PSC_POS) /**< CTRL_PSC_DIV128 Setting */

/**@} end of group QDEC_CTRL_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_INTFL QDEC_INTFL
 * @brief    Interrupt Flag Register.
 * @{
 */
#define MXC_F_QDEC_INTFL_INDEX_POS                     0 /**< INTFL_INDEX Position */
#define MXC_F_QDEC_INTFL_INDEX                         ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_INDEX_POS)) /**< INTFL_INDEX Mask */

#define MXC_F_QDEC_INTFL_QERR_POS                      1 /**< INTFL_QERR Position */
#define MXC_F_QDEC_INTFL_QERR                          ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_QERR_POS)) /**< INTFL_QERR Mask */

#define MXC_F_QDEC_INTFL_COMPARE_POS                   2 /**< INTFL_COMPARE Position */
#define MXC_F_QDEC_INTFL_COMPARE                       ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_COMPARE_POS)) /**< INTFL_COMPARE Mask */

#define MXC_F_QDEC_INTFL_MAXCNT_POS                    3 /**< INTFL_MAXCNT Position */
#define MXC_F_QDEC_INTFL_MAXCNT                        ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_MAXCNT_POS)) /**< INTFL_MAXCNT Mask */

#define MXC_F_QDEC_INTFL_CAPTURE_POS                   4 /**< INTFL_CAPTURE Position */
#define MXC_F_QDEC_INTFL_CAPTURE                       ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_CAPTURE_POS)) /**< INTFL_CAPTURE Mask */

#define MXC_F_QDEC_INTFL_DIR_POS                       5 /**< INTFL_DIR Position */
#define MXC_F_QDEC_INTFL_DIR                           ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_DIR_POS)) /**< INTFL_DIR Mask */

#define MXC_F_QDEC_INTFL_MOVE_POS                      6 /**< INTFL_MOVE Position */
#define MXC_F_QDEC_INTFL_MOVE                          ((uint32_t)(0x1UL << MXC_F_QDEC_INTFL_MOVE_POS)) /**< INTFL_MOVE Mask */

/**@} end of group QDEC_INTFL_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_INTEN QDEC_INTEN
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_QDEC_INTEN_INDEX_POS                     0 /**< INTEN_INDEX Position */
#define MXC_F_QDEC_INTEN_INDEX                         ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_INDEX_POS)) /**< INTEN_INDEX Mask */

#define MXC_F_QDEC_INTEN_QERR_POS                      1 /**< INTEN_QERR Position */
#define MXC_F_QDEC_INTEN_QERR                          ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_QERR_POS)) /**< INTEN_QERR Mask */

#define MXC_F_QDEC_INTEN_COMPARE_POS                   2 /**< INTEN_COMPARE Position */
#define MXC_F_QDEC_INTEN_COMPARE                       ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_COMPARE_POS)) /**< INTEN_COMPARE Mask */

#define MXC_F_QDEC_INTEN_MAXCNT_POS                    3 /**< INTEN_MAXCNT Position */
#define MXC_F_QDEC_INTEN_MAXCNT                        ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_MAXCNT_POS)) /**< INTEN_MAXCNT Mask */

#define MXC_F_QDEC_INTEN_CAPTURE_POS                   4 /**< INTEN_CAPTURE Position */
#define MXC_F_QDEC_INTEN_CAPTURE                       ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_CAPTURE_POS)) /**< INTEN_CAPTURE Mask */

#define MXC_F_QDEC_INTEN_DIR_POS                       5 /**< INTEN_DIR Position */
#define MXC_F_QDEC_INTEN_DIR                           ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_DIR_POS)) /**< INTEN_DIR Mask */

#define MXC_F_QDEC_INTEN_MOVE_POS                      6 /**< INTEN_MOVE Position */
#define MXC_F_QDEC_INTEN_MOVE                          ((uint32_t)(0x1UL << MXC_F_QDEC_INTEN_MOVE_POS)) /**< INTEN_MOVE Mask */

/**@} end of group QDEC_INTEN_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_MAXCNT QDEC_MAXCNT
 * @brief    Maximum Count Register.
 * @{
 */
#define MXC_F_QDEC_MAXCNT_MAXCNT_POS                   0 /**< MAXCNT_MAXCNT Position */
#define MXC_F_QDEC_MAXCNT_MAXCNT                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_MAXCNT_MAXCNT_POS)) /**< MAXCNT_MAXCNT Mask */

/**@} end of group QDEC_MAXCNT_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_INITIAL QDEC_INITIAL
 * @brief    Initial Count Register.
 * @{
 */
#define MXC_F_QDEC_INITIAL_INITIAL_POS                 0 /**< INITIAL_INITIAL Position */
#define MXC_F_QDEC_INITIAL_INITIAL                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_INITIAL_INITIAL_POS)) /**< INITIAL_INITIAL Mask */

/**@} end of group QDEC_INITIAL_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_COMPARE QDEC_COMPARE
 * @brief    Compare Register.
 * @{
 */
#define MXC_F_QDEC_COMPARE_COMPARE_POS                 0 /**< COMPARE_COMPARE Position */
#define MXC_F_QDEC_COMPARE_COMPARE                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_COMPARE_COMPARE_POS)) /**< COMPARE_COMPARE Mask */

/**@} end of group QDEC_COMPARE_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_INDEX QDEC_INDEX
 * @brief    Index Register. count captured when QEI fired
 * @{
 */
#define MXC_F_QDEC_INDEX_INDEX_POS                     0 /**< INDEX_INDEX Position */
#define MXC_F_QDEC_INDEX_INDEX                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_INDEX_INDEX_POS)) /**< INDEX_INDEX Mask */

/**@} end of group QDEC_INDEX_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_CAPTURE QDEC_CAPTURE
 * @brief    Capture Register. counter captured when QES fired
 * @{
 */
#define MXC_F_QDEC_CAPTURE_CAPTURE_POS                 0 /**< CAPTURE_CAPTURE Position */
#define MXC_F_QDEC_CAPTURE_CAPTURE                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_CAPTURE_CAPTURE_POS)) /**< CAPTURE_CAPTURE Mask */

/**@} end of group QDEC_CAPTURE_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_STATUS QDEC_STATUS
 * @brief    Status Register.
 * @{
 */
#define MXC_F_QDEC_STATUS_DIR_POS                      0 /**< STATUS_DIR Position */
#define MXC_F_QDEC_STATUS_DIR                          ((uint32_t)(0x1UL << MXC_F_QDEC_STATUS_DIR_POS)) /**< STATUS_DIR Mask */

/**@} end of group QDEC_STATUS_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_POSITION QDEC_POSITION
 * @brief    Count Register. raw counter value
 * @{
 */
#define MXC_F_QDEC_POSITION_POSITION_POS               0 /**< POSITION_POSITION Position */
#define MXC_F_QDEC_POSITION_POSITION                   ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_POSITION_POSITION_POS)) /**< POSITION_POSITION Mask */

/**@} end of group QDEC_POSITION_Register */

/**
 * @ingroup  qdec_registers
 * @defgroup QDEC_CAPDLY QDEC_CAPDLY
 * @brief    delay CAPTURE
 * @{
 */
#define MXC_F_QDEC_CAPDLY_CAPDLY_POS                   0 /**< CAPDLY_CAPDLY Position */
#define MXC_F_QDEC_CAPDLY_CAPDLY                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_QDEC_CAPDLY_CAPDLY_POS)) /**< CAPDLY_CAPDLY Mask */

/**@} end of group QDEC_CAPDLY_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_QDEC_REGS_H_
