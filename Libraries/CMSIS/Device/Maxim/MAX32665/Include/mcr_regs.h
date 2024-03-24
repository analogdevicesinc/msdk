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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_MCR_REGS_H_

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
    __IO uint32_t hirc96m;              /**< <tt>\b 0x04:</tt> MCR HIRC96M Register */
    __IO uint32_t outen;                /**< <tt>\b 0x08:</tt> MCR OUTEN Register */
    __IO uint32_t aincomp;              /**< <tt>\b 0x0C:</tt> MCR AINCOMP Register */
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
#define MXC_R_MCR_HIRC96M                  ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_OUTEN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_AINCOMP                  ((uint32_t)0x0000000CUL) /**< Offset from MCR Base Address: <tt> 0x000C</tt> */
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

#define MXC_F_MCR_ECCEN_IC1ECCEN_POS                   9 /**< ECCEN_IC1ECCEN Position */
#define MXC_F_MCR_ECCEN_IC1ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_IC1ECCEN_POS)) /**< ECCEN_IC1ECCEN Mask */

#define MXC_F_MCR_ECCEN_ICXIPFECCEN_POS                10 /**< ECCEN_ICXIPFECCEN Position */
#define MXC_F_MCR_ECCEN_ICXIPFECCEN                    ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_ICXIPFECCEN_POS)) /**< ECCEN_ICXIPFECCEN Mask */

#define MXC_F_MCR_ECCEN_FL0ECCEN_POS                   11 /**< ECCEN_FL0ECCEN Position */
#define MXC_F_MCR_ECCEN_FL0ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL0ECCEN_POS)) /**< ECCEN_FL0ECCEN Mask */

#define MXC_F_MCR_ECCEN_FL1ECCEN_POS                   12 /**< ECCEN_FL1ECCEN Position */
#define MXC_F_MCR_ECCEN_FL1ECCEN                       ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FL1ECCEN_POS)) /**< ECCEN_FL1ECCEN Mask */

/**@} end of group MCR_ECCEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_HIRC96M MCR_HIRC96M
 * @brief    96MHz High Frequency Clock Adjustment Register
 * @{
 */
#define MXC_F_MCR_HIRC96M_HIRC96MTR_POS                0 /**< HIRC96M_HIRC96MTR Position */
#define MXC_F_MCR_HIRC96M_HIRC96MTR                    ((uint32_t)(0x1FFUL << MXC_F_MCR_HIRC96M_HIRC96MTR_POS)) /**< HIRC96M_HIRC96MTR Mask */
#define MXC_V_MCR_HIRC96M_HIRC96MTR_DEFAULT            ((uint32_t)0x100UL) /**< HIRC96M_HIRC96MTR_DEFAULT Value */
#define MXC_S_MCR_HIRC96M_HIRC96MTR_DEFAULT            (MXC_V_MCR_HIRC96M_HIRC96MTR_DEFAULT << MXC_F_MCR_HIRC96M_HIRC96MTR_POS) /**< HIRC96M_HIRC96MTR_DEFAULT Setting */

/**@} end of group MCR_HIRC96M_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_OUTEN MCR_OUTEN
 * @brief    GPIOOUT_EN Function Enable Register
 * @{
 */
#define MXC_F_MCR_OUTEN_SQWOUT0EN_POS                  0 /**< OUTEN_SQWOUT0EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT0EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT0EN_POS)) /**< OUTEN_SQWOUT0EN Mask */

#define MXC_F_MCR_OUTEN_SQWOUT1EN_POS                  1 /**< OUTEN_SQWOUT1EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT1EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT1EN_POS)) /**< OUTEN_SQWOUT1EN Mask */

#define MXC_F_MCR_OUTEN_PDOWNOUT0EN_POS                2 /**< OUTEN_PDOWNOUT0EN Position */
#define MXC_F_MCR_OUTEN_PDOWNOUT0EN                    ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_PDOWNOUT0EN_POS)) /**< OUTEN_PDOWNOUT0EN Mask */

#define MXC_F_MCR_OUTEN_PDOWNOUT1EN_POS                3 /**< OUTEN_PDOWNOUT1EN Position */
#define MXC_F_MCR_OUTEN_PDOWNOUT1EN                    ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_PDOWNOUT1EN_POS)) /**< OUTEN_PDOWNOUT1EN Mask */

/**@} end of group MCR_OUTEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_AINCOMP MCR_AINCOMP
 * @brief    Comparator Power Control Register
 * @{
 */
#define MXC_F_MCR_AINCOMP_AINCOMP0PD_POS               0 /**< AINCOMP_AINCOMP0PD Position */
#define MXC_F_MCR_AINCOMP_AINCOMP0PD                   ((uint32_t)(0x1UL << MXC_F_MCR_AINCOMP_AINCOMP0PD_POS)) /**< AINCOMP_AINCOMP0PD Mask */

#define MXC_F_MCR_AINCOMP_AINCOMP1PD_POS               1 /**< AINCOMP_AINCOMP1PD Position */
#define MXC_F_MCR_AINCOMP_AINCOMP1PD                   ((uint32_t)(0x1UL << MXC_F_MCR_AINCOMP_AINCOMP1PD_POS)) /**< AINCOMP_AINCOMP1PD Mask */

#define MXC_F_MCR_AINCOMP_AINCOMP2PD_POS               2 /**< AINCOMP_AINCOMP2PD Position */
#define MXC_F_MCR_AINCOMP_AINCOMP2PD                   ((uint32_t)(0x1UL << MXC_F_MCR_AINCOMP_AINCOMP2PD_POS)) /**< AINCOMP_AINCOMP2PD Mask */

#define MXC_F_MCR_AINCOMP_AINCOMP3PD_POS               3 /**< AINCOMP_AINCOMP3PD Position */
#define MXC_F_MCR_AINCOMP_AINCOMP3PD                   ((uint32_t)(0x1UL << MXC_F_MCR_AINCOMP_AINCOMP3PD_POS)) /**< AINCOMP_AINCOMP3PD Mask */

#define MXC_F_MCR_AINCOMP_AINCOMPHYST_POS              4 /**< AINCOMP_AINCOMPHYST Position */
#define MXC_F_MCR_AINCOMP_AINCOMPHYST                  ((uint32_t)(0x3UL << MXC_F_MCR_AINCOMP_AINCOMPHYST_POS)) /**< AINCOMP_AINCOMPHYST Mask */

/**@} end of group MCR_AINCOMP_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Misc Power State Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_VDDCSWEN_POS                    0 /**< CTRL_VDDCSWEN Position */
#define MXC_F_MCR_CTRL_VDDCSWEN                        ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_VDDCSWEN_POS)) /**< CTRL_VDDCSWEN Mask */

#define MXC_F_MCR_CTRL_VDDCSW_POS                      1 /**< CTRL_VDDCSW Position */
#define MXC_F_MCR_CTRL_VDDCSW                          ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_VDDCSW_POS)) /**< CTRL_VDDCSW Mask */

#define MXC_F_MCR_CTRL_USBSWEN_N_POS                   3 /**< CTRL_USBSWEN_N Position */
#define MXC_F_MCR_CTRL_USBSWEN_N                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_USBSWEN_N_POS)) /**< CTRL_USBSWEN_N Mask */

#define MXC_F_MCR_CTRL_BUCKCLKSCALEN_POS               8 /**< CTRL_BUCKCLKSCALEN Position */
#define MXC_F_MCR_CTRL_BUCKCLKSCALEN                   ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_BUCKCLKSCALEN_POS)) /**< CTRL_BUCKCLKSCALEN Mask */

#define MXC_F_MCR_CTRL_P1M_POS                         9 /**< CTRL_P1M Position */
#define MXC_F_MCR_CTRL_P1M                             ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_P1M_POS)) /**< CTRL_P1M Mask */

#define MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL_POS            10 /**< CTRL_RSTN_VOLTAGE_SEL Position */
#define MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL                ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL_POS)) /**< CTRL_RSTN_VOLTAGE_SEL Mask */

/**@} end of group MCR_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_MCR_REGS_H_
