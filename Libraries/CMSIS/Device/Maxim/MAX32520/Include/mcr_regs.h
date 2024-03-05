/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_MCR_REGS_H_

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
 * @ingroup     mcr
 * @defgroup    mcr_registers MCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @details     Misc Control.
 */

/**
 * @ingroup mcr_registers
 * Structure type to access the MCR Registers.
 */
typedef struct {
    __IO uint32_t eccen;                /**< <tt>\b 0x00:</tt> MCR ECCEN Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ECCEN MCR_ECCEN
 * @brief    ECC Enable Register
 * @{
 */
#define MXC_F_MCR_ECCEN_SYSRAM0ECCEN_POS               0 /**< ECCEN_SYSRAM0ECCEN Position */
#define MXC_F_MCR_ECCEN_SYSRAM0ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM0ECCEN_POS)) /**< ECCEN_SYSRAM0ECCEN Mask */

#define MXC_F_MCR_ECCEN_SYSRAM1ECCEN_POS               1 /**< ECCEN_SYSRAM1ECCEN Position */
#define MXC_F_MCR_ECCEN_SYSRAM1ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM1ECCEN_POS)) /**< ECCEN_SYSRAM1ECCEN Mask */

#define MXC_F_MCR_ECCEN_SYSRAM2ECCEN_POS               2 /**< ECCEN_SYSRAM2ECCEN Position */
#define MXC_F_MCR_ECCEN_SYSRAM2ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM2ECCEN_POS)) /**< ECCEN_SYSRAM2ECCEN Mask */

#define MXC_F_MCR_ECCEN_SYSRAM3ECCEN_POS               3 /**< ECCEN_SYSRAM3ECCEN Position */
#define MXC_F_MCR_ECCEN_SYSRAM3ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM3ECCEN_POS)) /**< ECCEN_SYSRAM3ECCEN Mask */

#define MXC_F_MCR_ECCEN_SYSRAM4ECCEN_POS               4 /**< ECCEN_SYSRAM4ECCEN Position */
#define MXC_F_MCR_ECCEN_SYSRAM4ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM4ECCEN_POS)) /**< ECCEN_SYSRAM4ECCEN Mask */

#define MXC_F_MCR_ECCEN_FL0ECCEN_POS                   11 /**< ECCEN_FL0ECCEN Position */
#define MXC_F_MCR_ECCEN_FL0ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL0ECCEN_POS)) /**< ECCEN_FL0ECCEN Mask */

#define MXC_F_MCR_ECCEN_FL1ECCEN_POS                   12 /**< ECCEN_FL1ECCEN Position */
#define MXC_F_MCR_ECCEN_FL1ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL1ECCEN_POS)) /**< ECCEN_FL1ECCEN Mask */

/**@} end of group MCR_ECCEN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_MCR_REGS_H_
