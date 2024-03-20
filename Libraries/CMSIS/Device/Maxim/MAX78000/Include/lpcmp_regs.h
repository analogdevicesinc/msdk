/**
 * @file    lpcmp_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the LPCMP Peripheral Module.
 * @note    This file is @generated.
 * @ingroup lpcmp_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_LPCMP_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_LPCMP_REGS_H_

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
 * @ingroup     lpcmp
 * @defgroup    lpcmp_registers LPCMP_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the LPCMP Peripheral Module.
 * @details     Low Power Comparator
 */

/**
 * @ingroup lpcmp_registers
 * Structure type to access the LPCMP Registers.
 */
typedef struct {
    __IO uint32_t ctrl[3];              /**< <tt>\b 0x00:</tt> LPCMP CTRL Register */
} mxc_lpcmp_regs_t;

/* Register offsets for module LPCMP */
/**
 * @ingroup    lpcmp_registers
 * @defgroup   LPCMP_Register_Offsets Register Offsets
 * @brief      LPCMP Peripheral Register Offsets from the LPCMP Base Peripheral Address.
 * @{
 */
#define MXC_R_LPCMP_CTRL                   ((uint32_t)0x00000000UL) /**< Offset from LPCMP Base Address: <tt> 0x0000</tt> */
/**@} end of group lpcmp_registers */

/**
 * @ingroup  lpcmp_registers
 * @defgroup LPCMP_CTRL LPCMP_CTRL
 * @brief    Comparator Control Register.
 * @{
 */
#define MXC_F_LPCMP_CTRL_EN_POS                        0 /**< CTRL_EN Position */
#define MXC_F_LPCMP_CTRL_EN                            ((uint32_t)(0x1UL << MXC_F_LPCMP_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_LPCMP_CTRL_POL_POS                       5 /**< CTRL_POL Position */
#define MXC_F_LPCMP_CTRL_POL                           ((uint32_t)(0x1UL << MXC_F_LPCMP_CTRL_POL_POS)) /**< CTRL_POL Mask */

#define MXC_F_LPCMP_CTRL_INT_EN_POS                    6 /**< CTRL_INT_EN Position */
#define MXC_F_LPCMP_CTRL_INT_EN                        ((uint32_t)(0x1UL << MXC_F_LPCMP_CTRL_INT_EN_POS)) /**< CTRL_INT_EN Mask */

#define MXC_F_LPCMP_CTRL_OUT_POS                       14 /**< CTRL_OUT Position */
#define MXC_F_LPCMP_CTRL_OUT                           ((uint32_t)(0x1UL << MXC_F_LPCMP_CTRL_OUT_POS)) /**< CTRL_OUT Mask */

#define MXC_F_LPCMP_CTRL_INT_FL_POS                    15 /**< CTRL_INT_FL Position */
#define MXC_F_LPCMP_CTRL_INT_FL                        ((uint32_t)(0x1UL << MXC_F_LPCMP_CTRL_INT_FL_POS)) /**< CTRL_INT_FL Mask */

/**@} end of group LPCMP_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_LPCMP_REGS_H_
