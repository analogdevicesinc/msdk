/**
 * @file    fcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup fcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_FCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_FCR_REGS_H_

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
 * @ingroup     fcr
 * @defgroup    fcr_registers FCR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
 * @details     Function Control.
 */

/**
 * @ingroup fcr_registers
 * Structure type to access the FCR Registers.
 */
typedef struct {
    __IO uint32_t fcr;                  /**< <tt>\b 0x00:</tt> FCR FCR Register */
} mxc_fcr_regs_t;

/* Register offsets for module FCR */
/**
 * @ingroup    fcr_registers
 * @defgroup   FCR_Register_Offsets Register Offsets
 * @brief      FCR Peripheral Register Offsets from the FCR Base Peripheral Address.
 * @{
 */
#define MXC_R_FCR_FCR                      ((uint32_t)0x00000000UL) /**< Offset from FCR Base Address: <tt> 0x0000</tt> */
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCR FCR_FCR
 * @brief    Register 0.
 * @{
 */
#define MXC_F_FCR_FCR_USB_CLK_SEL_POS                  16 /**< FCR_USB_CLK_SEL Position */
#define MXC_F_FCR_FCR_USB_CLK_SEL                      ((uint32_t)(0x1UL << MXC_F_FCR_FCR_USB_CLK_SEL_POS)) /**< FCR_USB_CLK_SEL Mask */

#define MXC_F_FCR_FCR_QSPI0_FNC_SEL_POS                17 /**< FCR_QSPI0_FNC_SEL Position */
#define MXC_F_FCR_FCR_QSPI0_FNC_SEL                    ((uint32_t)(0x1UL << MXC_F_FCR_FCR_QSPI0_FNC_SEL_POS)) /**< FCR_QSPI0_FNC_SEL Mask */

#define MXC_F_FCR_FCR_I2C0_SDA_FILTER_EN_POS           20 /**< FCR_I2C0_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCR_I2C0_SDA_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C0_SDA_FILTER_EN_POS)) /**< FCR_I2C0_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCR_I2C0_SCL_FILTER_EN_POS           21 /**< FCR_I2C0_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCR_I2C0_SCL_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C0_SCL_FILTER_EN_POS)) /**< FCR_I2C0_SCL_FILTER_EN Mask */

#define MXC_F_FCR_FCR_I2C1_SDA_FILTER_EN_POS           22 /**< FCR_I2C1_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCR_I2C1_SDA_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C1_SDA_FILTER_EN_POS)) /**< FCR_I2C1_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCR_I2C1_SCL_FILTER_EN_POS           23 /**< FCR_I2C1_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCR_I2C1_SCL_FILTER_EN               ((uint32_t)(0x1UL << MXC_F_FCR_FCR_I2C1_SCL_FILTER_EN_POS)) /**< FCR_I2C1_SCL_FILTER_EN Mask */

/**@} end of group FCR_FCR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_FCR_REGS_H_
