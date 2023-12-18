/**
 * @file    trimsir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRIMSIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup trimsir_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_TRIMSIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_TRIMSIR_REGS_H_

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
    __I  uint32_t bb_sir2;              /**< <tt>\b 0x08:</tt> TRIMSIR BB_SIR2 Register */
} mxc_trimsir_regs_t;

/* Register offsets for module TRIMSIR */
/**
 * @ingroup    trimsir_registers
 * @defgroup   TRIMSIR_Register_Offsets Register Offsets
 * @brief      TRIMSIR Peripheral Register Offsets from the TRIMSIR Base Peripheral Address.
 * @{
 */
#define MXC_R_TRIMSIR_BB_SIR2              ((uint32_t)0x00000008UL) /**< Offset from TRIMSIR Base Address: <tt> 0x0008</tt> */
/**@} end of group trimsir_registers */

/**
 * @ingroup  trimsir_registers
 * @defgroup TRIMSIR_BB_SIR2 TRIMSIR_BB_SIR2
 * @brief    System Init. Configuration Register 2.
 * @{
 */
#define MXC_F_TRIMSIR_BB_SIR2_RAM0ECCEN_POS            0 /**< BB_SIR2_RAM0ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM0ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM0ECCEN_POS)) /**< BB_SIR2_RAM0ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM1ECCEN_POS            1 /**< BB_SIR2_RAM1ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM1ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM1ECCEN_POS)) /**< BB_SIR2_RAM1ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM2ECCEN_POS            2 /**< BB_SIR2_RAM2ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM2ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM2ECCEN_POS)) /**< BB_SIR2_RAM2ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM3ECCEN_POS            3 /**< BB_SIR2_RAM3ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM3ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM3ECCEN_POS)) /**< BB_SIR2_RAM3ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM4ECCEN_POS            4 /**< BB_SIR2_RAM4ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM4ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM4ECCEN_POS)) /**< BB_SIR2_RAM4ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_RAM5ECCEN_POS            5 /**< BB_SIR2_RAM5ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_RAM5ECCEN                ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_RAM5ECCEN_POS)) /**< BB_SIR2_RAM5ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_ICCECCEN_POS             8 /**< BB_SIR2_ICCECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_ICCECCEN                 ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_ICCECCEN_POS)) /**< BB_SIR2_ICCECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_ICCXIPECCEN_POS          10 /**< BB_SIR2_ICCXIPECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_ICCXIPECCEN              ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_ICCXIPECCEN_POS)) /**< BB_SIR2_ICCXIPECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN_POS             11 /**< BB_SIR2_FL0ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN                 ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN_POS)) /**< BB_SIR2_FL0ECCEN Mask */

#define MXC_F_TRIMSIR_BB_SIR2_FL1ECCEN_POS             12 /**< BB_SIR2_FL1ECCEN Position */
#define MXC_F_TRIMSIR_BB_SIR2_FL1ECCEN                 ((uint32_t)(0x1UL << MXC_F_TRIMSIR_BB_SIR2_FL1ECCEN_POS)) /**< BB_SIR2_FL1ECCEN Mask */

/**@} end of group TRIMSIR_BB_SIR2_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_TRIMSIR_REGS_H_
