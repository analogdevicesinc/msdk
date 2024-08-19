/**
 * @file    wut_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the WUT Peripheral Module.
 * @note    This file is @generated.
 * @ingroup wut_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_WUT_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_WUT_REGS_H_

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
 * @ingroup     wut
 * @defgroup    wut_registers WUT_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the WUT Peripheral Module.
 * @details     32-bit reloadable timer that can be used for timing and wakeup.
 */

/**
 * @ingroup wut_registers
 * Structure type to access the WUT Registers.
 */
typedef struct {
    __IO uint32_t cnt;                  /**< <tt>\b 0x00:</tt> WUT CNT Register */
    __IO uint32_t cmp;                  /**< <tt>\b 0x04:</tt> WUT CMP Register */
    __R  uint32_t rsv_0x8;
    __IO uint32_t intr;                 /**< <tt>\b 0x0C:</tt> WUT INTR Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> WUT CTRL Register */
    __IO uint32_t nolcmp;               /**< <tt>\b 0x14:</tt> WUT NOLCMP Register */
    __IO uint32_t preset;               /**< <tt>\b 0x18:</tt> WUT PRESET Register */
    __IO uint32_t reload;               /**< <tt>\b 0x1C:</tt> WUT RELOAD Register */
    __IO uint32_t snapshot;             /**< <tt>\b 0x20:</tt> WUT SNAPSHOT Register */
} mxc_wut_regs_t;

/* Register offsets for module WUT */
/**
 * @ingroup    wut_registers
 * @defgroup   WUT_Register_Offsets Register Offsets
 * @brief      WUT Peripheral Register Offsets from the WUT Base Peripheral Address.
 * @{
 */
#define MXC_R_WUT_CNT                      ((uint32_t)0x00000000UL) /**< Offset from WUT Base Address: <tt> 0x0000</tt> */
#define MXC_R_WUT_CMP                      ((uint32_t)0x00000004UL) /**< Offset from WUT Base Address: <tt> 0x0004</tt> */
#define MXC_R_WUT_INTR                     ((uint32_t)0x0000000CUL) /**< Offset from WUT Base Address: <tt> 0x000C</tt> */
#define MXC_R_WUT_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from WUT Base Address: <tt> 0x0010</tt> */
#define MXC_R_WUT_NOLCMP                   ((uint32_t)0x00000014UL) /**< Offset from WUT Base Address: <tt> 0x0014</tt> */
#define MXC_R_WUT_PRESET                   ((uint32_t)0x00000018UL) /**< Offset from WUT Base Address: <tt> 0x0018</tt> */
#define MXC_R_WUT_RELOAD                   ((uint32_t)0x0000001CUL) /**< Offset from WUT Base Address: <tt> 0x001C</tt> */
#define MXC_R_WUT_SNAPSHOT                 ((uint32_t)0x00000020UL) /**< Offset from WUT Base Address: <tt> 0x0020</tt> */
/**@} end of group wut_registers */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_CNT WUT_CNT
 * @brief    Count.  This register stores the current timer count.
 * @{
 */
#define MXC_F_WUT_CNT_COUNT_POS                        0 /**< CNT_COUNT Position */
#define MXC_F_WUT_CNT_COUNT                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_CNT_COUNT_POS)) /**< CNT_COUNT Mask */

/**@} end of group WUT_CNT_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_CMP WUT_CMP
 * @brief    Compare.  This register stores the compare value, which is used to set the
 *           maximum count value to initiate a reload of the timer to 0x0001.
 * @{
 */
#define MXC_F_WUT_CMP_COMPARE_POS                      0 /**< CMP_COMPARE Position */
#define MXC_F_WUT_CMP_COMPARE                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_CMP_COMPARE_POS)) /**< CMP_COMPARE Mask */

/**@} end of group WUT_CMP_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_INTR WUT_INTR
 * @brief    Clear Interrupt. Writing a value (0 or 1) to a bit in this register clears the
 *           associated interrupt.
 * @{
 */
#define MXC_F_WUT_INTR_IRQ_CLR_POS                     0 /**< INTR_IRQ_CLR Position */
#define MXC_F_WUT_INTR_IRQ_CLR                         ((uint32_t)(0x1UL << MXC_F_WUT_INTR_IRQ_CLR_POS)) /**< INTR_IRQ_CLR Mask */

/**@} end of group WUT_INTR_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_CTRL WUT_CTRL
 * @brief    Timer Control Register.
 * @{
 */
#define MXC_F_WUT_CTRL_TMODE_POS                       0 /**< CTRL_TMODE Position */
#define MXC_F_WUT_CTRL_TMODE                           ((uint32_t)(0x7UL << MXC_F_WUT_CTRL_TMODE_POS)) /**< CTRL_TMODE Mask */
#define MXC_V_WUT_CTRL_TMODE_ONESHOT                   ((uint32_t)0x0UL) /**< CTRL_TMODE_ONESHOT Value */
#define MXC_S_WUT_CTRL_TMODE_ONESHOT                   (MXC_V_WUT_CTRL_TMODE_ONESHOT << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_ONESHOT Setting */
#define MXC_V_WUT_CTRL_TMODE_CONTINUOUS                ((uint32_t)0x1UL) /**< CTRL_TMODE_CONTINUOUS Value */
#define MXC_S_WUT_CTRL_TMODE_CONTINUOUS                (MXC_V_WUT_CTRL_TMODE_CONTINUOUS << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_CONTINUOUS Setting */
#define MXC_V_WUT_CTRL_TMODE_COUNTER                   ((uint32_t)0x2UL) /**< CTRL_TMODE_COUNTER Value */
#define MXC_S_WUT_CTRL_TMODE_COUNTER                   (MXC_V_WUT_CTRL_TMODE_COUNTER << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_COUNTER Setting */
#define MXC_V_WUT_CTRL_TMODE_CAPTURE                   ((uint32_t)0x4UL) /**< CTRL_TMODE_CAPTURE Value */
#define MXC_S_WUT_CTRL_TMODE_CAPTURE                   (MXC_V_WUT_CTRL_TMODE_CAPTURE << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_CAPTURE Setting */
#define MXC_V_WUT_CTRL_TMODE_COMPARE                   ((uint32_t)0x5UL) /**< CTRL_TMODE_COMPARE Value */
#define MXC_S_WUT_CTRL_TMODE_COMPARE                   (MXC_V_WUT_CTRL_TMODE_COMPARE << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_COMPARE Setting */
#define MXC_V_WUT_CTRL_TMODE_GATED                     ((uint32_t)0x6UL) /**< CTRL_TMODE_GATED Value */
#define MXC_S_WUT_CTRL_TMODE_GATED                     (MXC_V_WUT_CTRL_TMODE_GATED << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_GATED Setting */
#define MXC_V_WUT_CTRL_TMODE_CAPTURECOMPARE            ((uint32_t)0x7UL) /**< CTRL_TMODE_CAPTURECOMPARE Value */
#define MXC_S_WUT_CTRL_TMODE_CAPTURECOMPARE            (MXC_V_WUT_CTRL_TMODE_CAPTURECOMPARE << MXC_F_WUT_CTRL_TMODE_POS) /**< CTRL_TMODE_CAPTURECOMPARE Setting */

#define MXC_F_WUT_CTRL_PRES_POS                        3 /**< CTRL_PRES Position */
#define MXC_F_WUT_CTRL_PRES                            ((uint32_t)(0x7UL << MXC_F_WUT_CTRL_PRES_POS)) /**< CTRL_PRES Mask */
#define MXC_V_WUT_CTRL_PRES_DIV1                       ((uint32_t)0x0UL) /**< CTRL_PRES_DIV1 Value */
#define MXC_S_WUT_CTRL_PRES_DIV1                       (MXC_V_WUT_CTRL_PRES_DIV1 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV1 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV2                       ((uint32_t)0x1UL) /**< CTRL_PRES_DIV2 Value */
#define MXC_S_WUT_CTRL_PRES_DIV2                       (MXC_V_WUT_CTRL_PRES_DIV2 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV2 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV4                       ((uint32_t)0x2UL) /**< CTRL_PRES_DIV4 Value */
#define MXC_S_WUT_CTRL_PRES_DIV4                       (MXC_V_WUT_CTRL_PRES_DIV4 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV4 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV8                       ((uint32_t)0x3UL) /**< CTRL_PRES_DIV8 Value */
#define MXC_S_WUT_CTRL_PRES_DIV8                       (MXC_V_WUT_CTRL_PRES_DIV8 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV8 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV16                      ((uint32_t)0x4UL) /**< CTRL_PRES_DIV16 Value */
#define MXC_S_WUT_CTRL_PRES_DIV16                      (MXC_V_WUT_CTRL_PRES_DIV16 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV16 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV32                      ((uint32_t)0x5UL) /**< CTRL_PRES_DIV32 Value */
#define MXC_S_WUT_CTRL_PRES_DIV32                      (MXC_V_WUT_CTRL_PRES_DIV32 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV32 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV64                      ((uint32_t)0x6UL) /**< CTRL_PRES_DIV64 Value */
#define MXC_S_WUT_CTRL_PRES_DIV64                      (MXC_V_WUT_CTRL_PRES_DIV64 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV64 Setting */
#define MXC_V_WUT_CTRL_PRES_DIV128                     ((uint32_t)0x7UL) /**< CTRL_PRES_DIV128 Value */
#define MXC_S_WUT_CTRL_PRES_DIV128                     (MXC_V_WUT_CTRL_PRES_DIV128 << MXC_F_WUT_CTRL_PRES_POS) /**< CTRL_PRES_DIV128 Setting */

#define MXC_F_WUT_CTRL_TPOL_POS                        6 /**< CTRL_TPOL Position */
#define MXC_F_WUT_CTRL_TPOL                            ((uint32_t)(0x1UL << MXC_F_WUT_CTRL_TPOL_POS)) /**< CTRL_TPOL Mask */

#define MXC_F_WUT_CTRL_TEN_POS                         7 /**< CTRL_TEN Position */
#define MXC_F_WUT_CTRL_TEN                             ((uint32_t)(0x1UL << MXC_F_WUT_CTRL_TEN_POS)) /**< CTRL_TEN Mask */

#define MXC_F_WUT_CTRL_PRES3_POS                       8 /**< CTRL_PRES3 Position */
#define MXC_F_WUT_CTRL_PRES3                           ((uint32_t)(0x1UL << MXC_F_WUT_CTRL_PRES3_POS)) /**< CTRL_PRES3 Mask */

/**@} end of group WUT_CTRL_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_NOLCMP WUT_NOLCMP
 * @brief    Timer Non-Overlapping Compare Register.
 * @{
 */
#define MXC_F_WUT_NOLCMP_NOLLCMP_POS                   0 /**< NOLCMP_NOLLCMP Position */
#define MXC_F_WUT_NOLCMP_NOLLCMP                       ((uint32_t)(0xFFUL << MXC_F_WUT_NOLCMP_NOLLCMP_POS)) /**< NOLCMP_NOLLCMP Mask */

#define MXC_F_WUT_NOLCMP_NOLHCMP_POS                   8 /**< NOLCMP_NOLHCMP Position */
#define MXC_F_WUT_NOLCMP_NOLHCMP                       ((uint32_t)(0xFFUL << MXC_F_WUT_NOLCMP_NOLHCMP_POS)) /**< NOLCMP_NOLHCMP Mask */

/**@} end of group WUT_NOLCMP_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_PRESET WUT_PRESET
 * @brief    Preset register.
 * @{
 */
#define MXC_F_WUT_PRESET_PRESET_POS                    0 /**< PRESET_PRESET Position */
#define MXC_F_WUT_PRESET_PRESET                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_PRESET_PRESET_POS)) /**< PRESET_PRESET Mask */

/**@} end of group WUT_PRESET_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_RELOAD WUT_RELOAD
 * @brief    Reload register.
 * @{
 */
#define MXC_F_WUT_RELOAD_RELOAD_POS                    0 /**< RELOAD_RELOAD Position */
#define MXC_F_WUT_RELOAD_RELOAD                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_RELOAD_RELOAD_POS)) /**< RELOAD_RELOAD Mask */

/**@} end of group WUT_RELOAD_Register */

/**
 * @ingroup  wut_registers
 * @defgroup WUT_SNAPSHOT WUT_SNAPSHOT
 * @brief    Snapshot register.
 * @{
 */
#define MXC_F_WUT_SNAPSHOT_SNAPSHOT_POS                0 /**< SNAPSHOT_SNAPSHOT Position */
#define MXC_F_WUT_SNAPSHOT_SNAPSHOT                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_SNAPSHOT_SNAPSHOT_POS)) /**< SNAPSHOT_SNAPSHOT Mask */

/**@} end of group WUT_SNAPSHOT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_WUT_REGS_H_
