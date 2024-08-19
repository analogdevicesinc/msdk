/**
 * @file    trimsir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup trimsir_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_TRIMSIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_TRIMSIR_REGS_H_

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
 * @ingroup     trimsir
 * @defgroup    trimsir_registers TRIMSIR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @details     Trim System Initilazation Registers
 */

/**
 * @ingroup trimsir_registers
 * Structure type to access the TRIMSIR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t rtc;                  /**< <tt>\b 0x08:</tt> TRIMSIR RTC Register */
    __R  uint32_t rsv_0xc_0x33[10];
    __IO uint32_t sir13;                /**< <tt>\b 0x34:</tt> TRIMSIR SIR13 Register */
    __R  uint32_t rsv_0x38_0x43[3];
    __IO uint32_t sir17;                /**< <tt>\b 0x44:</tt> TRIMSIR SIR17 Register */
} mxc_trimsir_regs_t;

/* Register offsets for module TRIMSIR */
/**
 * @ingroup    trimsir_registers
 * @defgroup   TRIMSIR_Register_Offsets Register Offsets
 * @brief      TRIMSIR Peripheral Register Offsets from the TRIMSIR Base Peripheral Address.
 * @{
 */
#define MXC_R_TRIMSIR_RTC                  ((uint32_t)0x00000008UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0008</tt> */
#define MXC_R_TRIMSIR_SIR13                ((uint32_t)0x00000034UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0034</tt> */
#define MXC_R_TRIMSIR_SIR17                ((uint32_t)0x00000044UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0044</tt> */
/**@} end of group trimsir_registers */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_RTC TRIMSIR_RTC
 * @brief    System Init. Configuration Register 2.
 * @{
 */
#define MXC_F_TRIMSIR_RTC_RTCX1_POS                    16 /**< RTC_RTCX1 Position */
#define MXC_F_TRIMSIR_RTC_RTCX1                        ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTC_RTCX1_POS)) /**< RTC_RTCX1 Mask */

#define MXC_F_TRIMSIR_RTC_RTCX2_POS                    21 /**< RTC_RTCX2 Position */
#define MXC_F_TRIMSIR_RTC_RTCX2                        ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTC_RTCX2_POS)) /**< RTC_RTCX2 Mask */

#define MXC_F_TRIMSIR_RTC_LOCK_POS                     31 /**< RTC_LOCK Position */
#define MXC_F_TRIMSIR_RTC_LOCK                         ((uint32_t)(0x1UL << MXC_F_TRIMSIR_RTC_LOCK_POS)) /**< RTC_LOCK Mask */

/**@} end of group TRIMSIR_RTC_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_SIR13 TRIMSIR_SIR13
 * @brief    System Init. Configuration Register 13.
 * @{
 */
#define MXC_F_TRIMSIR_SIR13_SIMOCLKDIV_POS             0 /**< SIR13_SIMOCLKDIV Position */
#define MXC_F_TRIMSIR_SIR13_SIMOCLKDIV                 ((uint32_t)(0x3UL << MXC_F_TRIMSIR_SIR13_SIMOCLKDIV_POS)) /**< SIR13_SIMOCLKDIV Mask */

/**@} end of group TRIMSIR_SIR13_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_SIR17 TRIMSIR_SIR17
 * @brief    System Init. Configuration Register 17.
 * @{
 */
#define MXC_F_TRIMSIR_SIR17_BUCKCLKSELLP_POS           6 /**< SIR17_BUCKCLKSELLP Position */
#define MXC_F_TRIMSIR_SIR17_BUCKCLKSELLP               ((uint32_t)(0x3UL << MXC_F_TRIMSIR_SIR17_BUCKCLKSELLP_POS)) /**< SIR17_BUCKCLKSELLP Mask */

/**@} end of group TRIMSIR_SIR17_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_TRIMSIR_REGS_H_
