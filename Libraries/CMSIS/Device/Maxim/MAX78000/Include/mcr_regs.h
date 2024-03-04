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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_MCR_REGS_H_

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
    __IO uint32_t ipo_mtrim;            /**< <tt>\b 0x04:</tt> MCR IPO_MTRIM Register */
    __IO uint32_t outen;                /**< <tt>\b 0x08:</tt> MCR OUTEN Register */
    __IO uint32_t cmp_ctrl;             /**< <tt>\b 0x0C:</tt> MCR CMP_CTRL Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __R  uint32_t rsv_0x14_0x1f[3];
    __IO uint32_t gpio3_ctrl;           /**< <tt>\b 0x20:</tt> MCR GPIO3_CTRL Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_MCR_IPO_MTRIM                ((uint32_t)0x00000004UL) /**< Offset from MCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_MCR_OUTEN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CMP_CTRL                 ((uint32_t)0x0000000CUL) /**< Offset from MCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_GPIO3_CTRL               ((uint32_t)0x00000020UL) /**< Offset from MCR Base Address: <tt> 0x0020</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ECCEN MCR_ECCEN
 * @brief    ECC Enable Register
 * @{
 */
#define MXC_F_MCR_ECCEN_RAM0_POS                       0 /**< ECCEN_RAM0 Position */
#define MXC_F_MCR_ECCEN_RAM0                           ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_RAM0_POS)) /**< ECCEN_RAM0 Mask */

/**@} end of group MCR_ECCEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_IPO_MTRIM MCR_IPO_MTRIM
 * @brief    IPO Manual Register
 * @{
 */
#define MXC_F_MCR_IPO_MTRIM_MTRIM_POS                  0 /**< IPO_MTRIM_MTRIM Position */
#define MXC_F_MCR_IPO_MTRIM_MTRIM                      ((uint32_t)(0xFFUL << MXC_F_MCR_IPO_MTRIM_MTRIM_POS)) /**< IPO_MTRIM_MTRIM Mask */

#define MXC_F_MCR_IPO_MTRIM_TRIM_RANGE_POS             8 /**< IPO_MTRIM_TRIM_RANGE Position */
#define MXC_F_MCR_IPO_MTRIM_TRIM_RANGE                 ((uint32_t)(0x1UL << MXC_F_MCR_IPO_MTRIM_TRIM_RANGE_POS)) /**< IPO_MTRIM_TRIM_RANGE Mask */

/**@} end of group MCR_IPO_MTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_OUTEN MCR_OUTEN
 * @brief    Output Enable Register
 * @{
 */
#define MXC_F_MCR_OUTEN_SQWOUT_EN_POS                  0 /**< OUTEN_SQWOUT_EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT_EN_POS)) /**< OUTEN_SQWOUT_EN Mask */

#define MXC_F_MCR_OUTEN_PDOWN_OUT_EN_POS               1 /**< OUTEN_PDOWN_OUT_EN Position */
#define MXC_F_MCR_OUTEN_PDOWN_OUT_EN                   ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_PDOWN_OUT_EN_POS)) /**< OUTEN_PDOWN_OUT_EN Mask */

/**@} end of group MCR_OUTEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CMP_CTRL MCR_CMP_CTRL
 * @brief    Comparator Control Register.
 * @{
 */
#define MXC_F_MCR_CMP_CTRL_EN_POS                      0 /**< CMP_CTRL_EN Position */
#define MXC_F_MCR_CMP_CTRL_EN                          ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_EN_POS)) /**< CMP_CTRL_EN Mask */

#define MXC_F_MCR_CMP_CTRL_POL_POS                     5 /**< CMP_CTRL_POL Position */
#define MXC_F_MCR_CMP_CTRL_POL                         ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_POL_POS)) /**< CMP_CTRL_POL Mask */

#define MXC_F_MCR_CMP_CTRL_INT_EN_POS                  6 /**< CMP_CTRL_INT_EN Position */
#define MXC_F_MCR_CMP_CTRL_INT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_INT_EN_POS)) /**< CMP_CTRL_INT_EN Mask */

#define MXC_F_MCR_CMP_CTRL_OUT_POS                     14 /**< CMP_CTRL_OUT Position */
#define MXC_F_MCR_CMP_CTRL_OUT                         ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_OUT_POS)) /**< CMP_CTRL_OUT Mask */

#define MXC_F_MCR_CMP_CTRL_INT_FL_POS                  15 /**< CMP_CTRL_INT_FL Position */
#define MXC_F_MCR_CMP_CTRL_INT_FL                      ((uint32_t)(0x1UL << MXC_F_MCR_CMP_CTRL_INT_FL_POS)) /**< CMP_CTRL_INT_FL Mask */

/**@} end of group MCR_CMP_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Miscellaneous Control Register.
 * @{
 */
#define MXC_F_MCR_CTRL_INRO_EN_POS                     2 /**< CTRL_INRO_EN Position */
#define MXC_F_MCR_CTRL_INRO_EN                         ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_INRO_EN_POS)) /**< CTRL_INRO_EN Mask */

#define MXC_F_MCR_CTRL_ERTCO_EN_POS                    3 /**< CTRL_ERTCO_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_EN                        ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_EN_POS)) /**< CTRL_ERTCO_EN Mask */

#define MXC_F_MCR_CTRL_SIMO_CLKSCL_EN_POS              8 /**< CTRL_SIMO_CLKSCL_EN Position */
#define MXC_F_MCR_CTRL_SIMO_CLKSCL_EN                  ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_SIMO_CLKSCL_EN_POS)) /**< CTRL_SIMO_CLKSCL_EN Mask */

#define MXC_F_MCR_CTRL_SIMO_RSTD_POS                   9 /**< CTRL_SIMO_RSTD Position */
#define MXC_F_MCR_CTRL_SIMO_RSTD                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_SIMO_RSTD_POS)) /**< CTRL_SIMO_RSTD Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_GPIO3_CTRL MCR_GPIO3_CTRL
 * @brief    GPIO3 Pin Control Register.
 * @{
 */
#define MXC_F_MCR_GPIO3_CTRL_P30_DO_POS                0 /**< GPIO3_CTRL_P30_DO Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_DO                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_DO_POS)) /**< GPIO3_CTRL_P30_DO Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_OE_POS                1 /**< GPIO3_CTRL_P30_OE Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_OE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_OE_POS)) /**< GPIO3_CTRL_P30_OE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_PE_POS                2 /**< GPIO3_CTRL_P30_PE Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_PE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_PE_POS)) /**< GPIO3_CTRL_P30_PE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P30_IN_POS                3 /**< GPIO3_CTRL_P30_IN Position */
#define MXC_F_MCR_GPIO3_CTRL_P30_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P30_IN_POS)) /**< GPIO3_CTRL_P30_IN Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_DO_POS                4 /**< GPIO3_CTRL_P31_DO Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_DO                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_DO_POS)) /**< GPIO3_CTRL_P31_DO Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_OE_POS                5 /**< GPIO3_CTRL_P31_OE Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_OE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_OE_POS)) /**< GPIO3_CTRL_P31_OE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_PE_POS                6 /**< GPIO3_CTRL_P31_PE Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_PE                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_PE_POS)) /**< GPIO3_CTRL_P31_PE Mask */

#define MXC_F_MCR_GPIO3_CTRL_P31_IN_POS                7 /**< GPIO3_CTRL_P31_IN Position */
#define MXC_F_MCR_GPIO3_CTRL_P31_IN                    ((uint32_t)(0x1UL << MXC_F_MCR_GPIO3_CTRL_P31_IN_POS)) /**< GPIO3_CTRL_P31_IN Mask */

/**@} end of group MCR_GPIO3_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_MCR_REGS_H_
