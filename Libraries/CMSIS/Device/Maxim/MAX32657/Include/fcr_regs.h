/**
 * @file    fcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup fcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FCR_REGS_H_

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
 * @details     Function Control Register.
 */

/**
 * @ingroup fcr_registers
 * Structure type to access the FCR Registers.
 */
typedef struct {
    __IO uint32_t fctrl0;               /**< <tt>\b 0x00:</tt> FCR FCTRL0 Register */
    __IO uint32_t fctrl1;               /**< <tt>\b 0x04:</tt> FCR FCTRL1 Register */
    __IO uint32_t fctrl2;               /**< <tt>\b 0x08:</tt> FCR FCTRL2 Register */
    __IO uint32_t fctrl3;               /**< <tt>\b 0x0C:</tt> FCR FCTRL3 Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __IO uint32_t erfoks;               /**< <tt>\b 0x18:</tt> FCR ERFOKS Register */
} mxc_fcr_regs_t;

/* Register offsets for module FCR */
/**
 * @ingroup    fcr_registers
 * @defgroup   FCR_Register_Offsets Register Offsets
 * @brief      FCR Peripheral Register Offsets from the FCR Base Peripheral Address.
 * @{
 */
#define MXC_R_FCR_FCTRL0                   ((uint32_t)0x00000000UL) /**< Offset from FCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_FCR_FCTRL1                   ((uint32_t)0x00000004UL) /**< Offset from FCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_FCR_FCTRL2                   ((uint32_t)0x00000008UL) /**< Offset from FCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_FCR_FCTRL3                   ((uint32_t)0x0000000CUL) /**< Offset from FCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_FCR_ERFOKS                   ((uint32_t)0x00000018UL) /**< Offset from FCR Base Address: <tt> 0x0018</tt> */
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL0 FCR_FCTRL0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_FCR_FCTRL0_BTLELDO_TX_POS                0 /**< FCTRL0_BTLELDO_TX Position */
#define MXC_F_FCR_FCTRL0_BTLELDO_TX                    ((uint32_t)(0x1FUL << MXC_F_FCR_FCTRL0_BTLELDO_TX_POS)) /**< FCTRL0_BTLELDO_TX Mask */

#define MXC_F_FCR_FCTRL0_BTLELDO_RX_POS                8 /**< FCTRL0_BTLELDO_RX Position */
#define MXC_F_FCR_FCTRL0_BTLELDO_RX                    ((uint32_t)(0x1FUL << MXC_F_FCR_FCTRL0_BTLELDO_RX_POS)) /**< FCTRL0_BTLELDO_RX Mask */

#define MXC_F_FCR_FCTRL0_I3CDGEN0_POS                  20 /**< FCTRL0_I3CDGEN0 Position */
#define MXC_F_FCR_FCTRL0_I3CDGEN0                      ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I3CDGEN0_POS)) /**< FCTRL0_I3CDGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I3CDGEN1_POS                  21 /**< FCTRL0_I3CDGEN1 Position */
#define MXC_F_FCR_FCTRL0_I3CDGEN1                      ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I3CDGEN1_POS)) /**< FCTRL0_I3CDGEN1 Mask */

/**@} end of group FCR_FCTRL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL1 FCR_FCTRL1
 * @brief    Register 1.
 * @{
 */
#define MXC_F_FCR_FCTRL1_AC_EN_POS                     0 /**< FCTRL1_AC_EN Position */
#define MXC_F_FCR_FCTRL1_AC_EN                         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_AC_EN_POS)) /**< FCTRL1_AC_EN Mask */

#define MXC_F_FCR_FCTRL1_AC_RUN_POS                    1 /**< FCTRL1_AC_RUN Position */
#define MXC_F_FCR_FCTRL1_AC_RUN                        ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_AC_RUN_POS)) /**< FCTRL1_AC_RUN Mask */

#define MXC_F_FCR_FCTRL1_LOAD_TRIM_POS                 2 /**< FCTRL1_LOAD_TRIM Position */
#define MXC_F_FCR_FCTRL1_LOAD_TRIM                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_LOAD_TRIM_POS)) /**< FCTRL1_LOAD_TRIM Mask */

#define MXC_F_FCR_FCTRL1_GAIN_INV_POS                  3 /**< FCTRL1_GAIN_INV Position */
#define MXC_F_FCR_FCTRL1_GAIN_INV                      ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_GAIN_INV_POS)) /**< FCTRL1_GAIN_INV Mask */

#define MXC_F_FCR_FCTRL1_ATOMIC_POS                    4 /**< FCTRL1_ATOMIC Position */
#define MXC_F_FCR_FCTRL1_ATOMIC                        ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL1_ATOMIC_POS)) /**< FCTRL1_ATOMIC Mask */

#define MXC_F_FCR_FCTRL1_MU_POS                        8 /**< FCTRL1_MU Position */
#define MXC_F_FCR_FCTRL1_MU                            ((uint32_t)(0xFFFUL << MXC_F_FCR_FCTRL1_MU_POS)) /**< FCTRL1_MU Mask */

#define MXC_F_FCR_FCTRL1_AC_TRIM_OUT_POS               23 /**< FCTRL1_AC_TRIM_OUT Position */
#define MXC_F_FCR_FCTRL1_AC_TRIM_OUT                   ((uint32_t)(0x1FFUL << MXC_F_FCR_FCTRL1_AC_TRIM_OUT_POS)) /**< FCTRL1_AC_TRIM_OUT Mask */

/**@} end of group FCR_FCTRL1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL2 FCR_FCTRL2
 * @brief    Register 2.
 * @{
 */
#define MXC_F_FCR_FCTRL2_AC_INIT_TRIM_POS              0 /**< FCTRL2_AC_INIT_TRIM Position */
#define MXC_F_FCR_FCTRL2_AC_INIT_TRIM                  ((uint32_t)(0x1FFUL << MXC_F_FCR_FCTRL2_AC_INIT_TRIM_POS)) /**< FCTRL2_AC_INIT_TRIM Mask */

/**@} end of group FCR_FCTRL2_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL3 FCR_FCTRL3
 * @brief    Register 3.
 * @{
 */
#define MXC_F_FCR_FCTRL3_AC_RUNTIME_POS                0 /**< FCTRL3_AC_RUNTIME Position */
#define MXC_F_FCR_FCTRL3_AC_RUNTIME                    ((uint32_t)(0xFFUL << MXC_F_FCR_FCTRL3_AC_RUNTIME_POS)) /**< FCTRL3_AC_RUNTIME Mask */

#define MXC_F_FCR_FCTRL3_AC_DIV_POS                    8 /**< FCTRL3_AC_DIV Position */
#define MXC_F_FCR_FCTRL3_AC_DIV                        ((uint32_t)(0x1FFFUL << MXC_F_FCR_FCTRL3_AC_DIV_POS)) /**< FCTRL3_AC_DIV Mask */

/**@} end of group FCR_FCTRL3_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ERFOKS FCR_ERFOKS
 * @brief    ERFO Kick Start Register.
 * @{
 */
#define MXC_F_FCR_ERFOKS_CNT_POS                       0 /**< ERFOKS_CNT Position */
#define MXC_F_FCR_ERFOKS_CNT                           ((uint32_t)(0x7FUL << MXC_F_FCR_ERFOKS_CNT_POS)) /**< ERFOKS_CNT Mask */

#define MXC_F_FCR_ERFOKS_EN_POS                        7 /**< ERFOKS_EN Position */
#define MXC_F_FCR_ERFOKS_EN                            ((uint32_t)(0x1UL << MXC_F_FCR_ERFOKS_EN_POS)) /**< ERFOKS_EN Mask */

#define MXC_F_FCR_ERFOKS_DRVSTR_POS                    8 /**< ERFOKS_DRVSTR Position */
#define MXC_F_FCR_ERFOKS_DRVSTR                        ((uint32_t)(0x7UL << MXC_F_FCR_ERFOKS_DRVSTR_POS)) /**< ERFOKS_DRVSTR Mask */

#define MXC_F_FCR_ERFOKS_X2_PLEN_POS                   11 /**< ERFOKS_X2_PLEN Position */
#define MXC_F_FCR_ERFOKS_X2_PLEN                       ((uint32_t)(0x1UL << MXC_F_FCR_ERFOKS_X2_PLEN_POS)) /**< ERFOKS_X2_PLEN Mask */

#define MXC_F_FCR_ERFOKS_CLKSEL_POS                    12 /**< ERFOKS_CLKSEL Position */
#define MXC_F_FCR_ERFOKS_CLKSEL                        ((uint32_t)(0x3UL << MXC_F_FCR_ERFOKS_CLKSEL_POS)) /**< ERFOKS_CLKSEL Mask */
#define MXC_V_FCR_ERFOKS_CLKSEL_NO_CLK                 ((uint32_t)0x0UL) /**< ERFOKS_CLKSEL_NO_CLK Value */
#define MXC_S_FCR_ERFOKS_CLKSEL_NO_CLK                 (MXC_V_FCR_ERFOKS_CLKSEL_NO_CLK << MXC_F_FCR_ERFOKS_CLKSEL_POS) /**< ERFOKS_CLKSEL_NO_CLK Setting */
#define MXC_V_FCR_ERFOKS_CLKSEL_IPO                    ((uint32_t)0x3UL) /**< ERFOKS_CLKSEL_IPO Value */
#define MXC_S_FCR_ERFOKS_CLKSEL_IPO                    (MXC_V_FCR_ERFOKS_CLKSEL_IPO << MXC_F_FCR_ERFOKS_CLKSEL_POS) /**< ERFOKS_CLKSEL_IPO Setting */

/**@} end of group FCR_ERFOKS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FCR_REGS_H_
