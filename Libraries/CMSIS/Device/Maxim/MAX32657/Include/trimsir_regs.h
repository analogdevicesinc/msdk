/**
 * @file    trimsir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup trimsir_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TRIMSIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TRIMSIR_REGS_H_

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
 * @ingroup     trimsir
 * @defgroup    trimsir_registers TRIMSIR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @details     Trim System Initilazation Registers.
 */

/**
 * @ingroup trimsir_registers
 * Structure type to access the TRIMSIR Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x3b[15];
    __IO uint32_t rtcx1;                /**< <tt>\b 0x3C:</tt> TRIMSIR RTCX1 Register */
    __R  uint32_t rsv_0x40;
    __IO uint32_t rtcx2;                /**< <tt>\b 0x44:</tt> TRIMSIR RTCX2 Register */
} mxc_trimsir_regs_t;

/* Register offsets for module TRIMSIR */
/**
 * @ingroup    trimsir_registers
 * @defgroup   TRIMSIR_Register_Offsets Register Offsets
 * @brief      TRIMSIR Peripheral Register Offsets from the TRIMSIR Base Peripheral Address.
 * @{
 */
#define MXC_R_TRIMSIR_RTCX1                ((uint32_t)0x0000003CUL) /**< Offset from TRIMSIR Base Address: <tt> 0x003C</tt> */
#define MXC_R_TRIMSIR_RTCX2                ((uint32_t)0x00000044UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0044</tt> */
/**@} end of group trimsir_registers */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_RTCX1 TRIMSIR_RTCX1
 * @brief    RTC X1 Capacitor Setting.
 * @{
 */
#define MXC_F_TRIMSIR_RTCX1_CAP_POS                    0 /**< RTCX1_CAP Position */
#define MXC_F_TRIMSIR_RTCX1_CAP                        ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTCX1_CAP_POS)) /**< RTCX1_CAP Mask */

/**@} end of group TRIMSIR_RTCX1_Register */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_RTCX2 TRIMSIR_RTCX2
 * @brief    RTC X2 Capacitor Setting.
 * @{
 */
#define MXC_F_TRIMSIR_RTCX2_CAP_POS                    0 /**< RTCX2_CAP Position */
#define MXC_F_TRIMSIR_RTCX2_CAP                        ((uint32_t)(0x1FUL << MXC_F_TRIMSIR_RTCX2_CAP_POS)) /**< RTCX2_CAP Mask */

/**@} end of group TRIMSIR_RTCX2_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_TRIMSIR_REGS_H_
