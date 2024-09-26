/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_

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
    __R  uint32_t rsv_0x4;
    __IO uint32_t pdown;                /**< <tt>\b 0x08:</tt> MCR PDOWN Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_MCR_PDOWN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
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

#define MXC_F_MCR_ECCEN_SYSRAM5ECCEN_POS               5 /**< ECCEN_SYSRAM5ECCEN Position */
#define MXC_F_MCR_ECCEN_SYSRAM5ECCEN                   ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_SYSRAM5ECCEN_POS)) /**< ECCEN_SYSRAM5ECCEN Mask */

#define MXC_F_MCR_ECCEN_IC0ECCEN_POS                   8 /**< ECCEN_IC0ECCEN Position */
#define MXC_F_MCR_ECCEN_IC0ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_IC0ECCEN_POS)) /**< ECCEN_IC0ECCEN Mask */

#define MXC_F_MCR_ECCEN_ICXIPECCEN_POS                 10 /**< ECCEN_ICXIPECCEN Position */
#define MXC_F_MCR_ECCEN_ICXIPECCEN                     ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_ICXIPECCEN_POS)) /**< ECCEN_ICXIPECCEN Mask */

#define MXC_F_MCR_ECCEN_FL0ECCEN_POS                   11 /**< ECCEN_FL0ECCEN Position */
#define MXC_F_MCR_ECCEN_FL0ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL0ECCEN_POS)) /**< ECCEN_FL0ECCEN Mask */

#define MXC_F_MCR_ECCEN_FL1ECCEN_POS                   12 /**< ECCEN_FL1ECCEN Position */
#define MXC_F_MCR_ECCEN_FL1ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL1ECCEN_POS)) /**< ECCEN_FL1ECCEN Mask */

/**@} end of group MCR_ECCEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PDOWN MCR_PDOWN
 * @brief    PDOWN Drive Strength
 * @{
 */
#define MXC_F_MCR_PDOWN_PDOWNDS_POS                    0 /**< PDOWN_PDOWNDS Position */
#define MXC_F_MCR_PDOWN_PDOWNDS                        ((uint32_t)(0x3UL << MXC_F_MCR_PDOWN_PDOWNDS_POS)) /**< PDOWN_PDOWNDS Mask */

#define MXC_F_MCR_PDOWN_PDOWNVS_POS                    2 /**< PDOWN_PDOWNVS Position */
#define MXC_F_MCR_PDOWN_PDOWNVS                        ((uint32_t)(0x1UL << MXC_F_MCR_PDOWN_PDOWNVS_POS)) /**< PDOWN_PDOWNVS Mask */

/**@} end of group MCR_PDOWN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Misc Power State Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_VDDCSW_POS                      1 /**< CTRL_VDDCSW Position */
#define MXC_F_MCR_CTRL_VDDCSW                          ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_VDDCSW_POS)) /**< CTRL_VDDCSW Mask */

#define MXC_F_MCR_CTRL_USBSWEN_N_POS                   3 /**< CTRL_USBSWEN_N Position */
#define MXC_F_MCR_CTRL_USBSWEN_N                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_USBSWEN_N_POS)) /**< CTRL_USBSWEN_N Mask */

#define MXC_F_MCR_CTRL_P1M_POS                         9 /**< CTRL_P1M Position */
#define MXC_F_MCR_CTRL_P1M                             ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_P1M_POS)) /**< CTRL_P1M Mask */

#define MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL_POS            10 /**< CTRL_RSTN_VOLTAGE_SEL Position */
#define MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL                ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL_POS)) /**< CTRL_RSTN_VOLTAGE_SEL Mask */

/**@} end of group MCR_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_MCR_REGS_H_
