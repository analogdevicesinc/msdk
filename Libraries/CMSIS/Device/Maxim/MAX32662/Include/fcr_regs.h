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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_FCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_FCR_REGS_H_

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
    __IO uint32_t autocal0;             /**< <tt>\b 0x04:</tt> FCR AUTOCAL0 Register */
    __IO uint32_t autocal1;             /**< <tt>\b 0x08:</tt> FCR AUTOCAL1 Register */
    __IO uint32_t autocal2;             /**< <tt>\b 0x0C:</tt> FCR AUTOCAL2 Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __IO uint32_t adcreftrim0;          /**< <tt>\b 0x18:</tt> FCR ADCREFTRIM0 Register */
    __IO uint32_t adcreftrim1;          /**< <tt>\b 0x1C:</tt> FCR ADCREFTRIM1 Register */
    __IO uint32_t adcreftrim2;          /**< <tt>\b 0x20:</tt> FCR ADCREFTRIM2 Register */
    __IO uint32_t erfoks;               /**< <tt>\b 0x24:</tt> FCR ERFOKS Register */
} mxc_fcr_regs_t;

/* Register offsets for module FCR */
/**
 * @ingroup    fcr_registers
 * @defgroup   FCR_Register_Offsets Register Offsets
 * @brief      FCR Peripheral Register Offsets from the FCR Base Peripheral Address.
 * @{
 */
#define MXC_R_FCR_FCTRL0                   ((uint32_t)0x00000000UL) /**< Offset from FCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_FCR_AUTOCAL0                 ((uint32_t)0x00000004UL) /**< Offset from FCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_FCR_AUTOCAL1                 ((uint32_t)0x00000008UL) /**< Offset from FCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_FCR_AUTOCAL2                 ((uint32_t)0x0000000CUL) /**< Offset from FCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_FCR_ADCREFTRIM0              ((uint32_t)0x00000018UL) /**< Offset from FCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_FCR_ADCREFTRIM1              ((uint32_t)0x0000001CUL) /**< Offset from FCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_FCR_ADCREFTRIM2              ((uint32_t)0x00000020UL) /**< Offset from FCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_FCR_ERFOKS                   ((uint32_t)0x00000024UL) /**< Offset from FCR Base Address: <tt> 0x0024</tt> */
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL0 FCR_FCTRL0
 * @brief    Function Control 0.
 * @{
 */
#define MXC_F_FCR_FCTRL0_ERFO_RANGE_SEL_POS            0 /**< FCTRL0_ERFO_RANGE_SEL Position */
#define MXC_F_FCR_FCTRL0_ERFO_RANGE_SEL                ((uint32_t)(0x7UL << MXC_F_FCR_FCTRL0_ERFO_RANGE_SEL_POS)) /**< FCTRL0_ERFO_RANGE_SEL Mask */

#define MXC_F_FCR_FCTRL0_KEYWIPE_SYS_POS               8 /**< FCTRL0_KEYWIPE_SYS Position */
#define MXC_F_FCR_FCTRL0_KEYWIPE_SYS                   ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_KEYWIPE_SYS_POS)) /**< FCTRL0_KEYWIPE_SYS Mask */

#define MXC_F_FCR_FCTRL0_I2C0DGEN0_POS                 20 /**< FCTRL0_I2C0DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C0DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN0_POS)) /**< FCTRL0_I2C0DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C0DGEN1_POS                 21 /**< FCTRL0_I2C0DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C0DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN1_POS)) /**< FCTRL0_I2C0DGEN1 Mask */

#define MXC_F_FCR_FCTRL0_I2C1DGEN0_POS                 22 /**< FCTRL0_I2C1DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C1DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN0_POS)) /**< FCTRL0_I2C1DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C1DGEN1_POS                 23 /**< FCTRL0_I2C1DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C1DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN1_POS)) /**< FCTRL0_I2C1DGEN1 Mask */

/**@} end of group FCR_FCTRL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL0 FCR_AUTOCAL0
 * @brief    Automatic Calibration 0.
 * @{
 */
#define MXC_F_FCR_AUTOCAL0_ACEN_POS                    0 /**< AUTOCAL0_ACEN Position */
#define MXC_F_FCR_AUTOCAL0_ACEN                        ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_ACEN_POS)) /**< AUTOCAL0_ACEN Mask */

#define MXC_F_FCR_AUTOCAL0_ACRUN_POS                   1 /**< AUTOCAL0_ACRUN Position */
#define MXC_F_FCR_AUTOCAL0_ACRUN                       ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_ACRUN_POS)) /**< AUTOCAL0_ACRUN Mask */

#define MXC_F_FCR_AUTOCAL0_LOAD_POS                    2 /**< AUTOCAL0_LOAD Position */
#define MXC_F_FCR_AUTOCAL0_LOAD                        ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_LOAD_POS)) /**< AUTOCAL0_LOAD Mask */

#define MXC_F_FCR_AUTOCAL0_GAIN_INV_POS                3 /**< AUTOCAL0_GAIN_INV Position */
#define MXC_F_FCR_AUTOCAL0_GAIN_INV                    ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_GAIN_INV_POS)) /**< AUTOCAL0_GAIN_INV Mask */

#define MXC_F_FCR_AUTOCAL0_ATOMIC_POS                  4 /**< AUTOCAL0_ATOMIC Position */
#define MXC_F_FCR_AUTOCAL0_ATOMIC                      ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_ATOMIC_POS)) /**< AUTOCAL0_ATOMIC Mask */

#define MXC_F_FCR_AUTOCAL0_MU_POS                      8 /**< AUTOCAL0_MU Position */
#define MXC_F_FCR_AUTOCAL0_MU                          ((uint32_t)(0xFFFUL << MXC_F_FCR_AUTOCAL0_MU_POS)) /**< AUTOCAL0_MU Mask */

#define MXC_F_FCR_AUTOCAL0_TRIM_POS                    23 /**< AUTOCAL0_TRIM Position */
#define MXC_F_FCR_AUTOCAL0_TRIM                        ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL0_TRIM_POS)) /**< AUTOCAL0_TRIM Mask */

/**@} end of group FCR_AUTOCAL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL1 FCR_AUTOCAL1
 * @brief    Automatic Calibration 1.
 * @{
 */
#define MXC_F_FCR_AUTOCAL1_INITTRIM_POS                0 /**< AUTOCAL1_INITTRIM Position */
#define MXC_F_FCR_AUTOCAL1_INITTRIM                    ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL1_INITTRIM_POS)) /**< AUTOCAL1_INITTRIM Mask */

/**@} end of group FCR_AUTOCAL1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL2 FCR_AUTOCAL2
 * @brief    Automatic Calibration 2
 * @{
 */
#define MXC_F_FCR_AUTOCAL2_DONECNT_POS                 0 /**< AUTOCAL2_DONECNT Position */
#define MXC_F_FCR_AUTOCAL2_DONECNT                     ((uint32_t)(0xFFUL << MXC_F_FCR_AUTOCAL2_DONECNT_POS)) /**< AUTOCAL2_DONECNT Mask */

#define MXC_F_FCR_AUTOCAL2_ACDIV_POS                   8 /**< AUTOCAL2_ACDIV Position */
#define MXC_F_FCR_AUTOCAL2_ACDIV                       ((uint32_t)(0x1FFFUL << MXC_F_FCR_AUTOCAL2_ACDIV_POS)) /**< AUTOCAL2_ACDIV Mask */

/**@} end of group FCR_AUTOCAL2_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ADCREFTRIM0 FCR_ADCREFTRIM0
 * @brief    ADC Reference Trim 0 Control Register.
 * @{
 */
#define MXC_F_FCR_ADCREFTRIM0_VREFP_POS                0 /**< ADCREFTRIM0_VREFP Position */
#define MXC_F_FCR_ADCREFTRIM0_VREFP                    ((uint32_t)(0x7FUL << MXC_F_FCR_ADCREFTRIM0_VREFP_POS)) /**< ADCREFTRIM0_VREFP Mask */

#define MXC_F_FCR_ADCREFTRIM0_VREFM_POS                8 /**< ADCREFTRIM0_VREFM Position */
#define MXC_F_FCR_ADCREFTRIM0_VREFM                    ((uint32_t)(0x7FUL << MXC_F_FCR_ADCREFTRIM0_VREFM_POS)) /**< ADCREFTRIM0_VREFM Mask */

#define MXC_F_FCR_ADCREFTRIM0_VCM_POS                  16 /**< ADCREFTRIM0_VCM Position */
#define MXC_F_FCR_ADCREFTRIM0_VCM                      ((uint32_t)(0x3UL << MXC_F_FCR_ADCREFTRIM0_VCM_POS)) /**< ADCREFTRIM0_VCM Mask */

#define MXC_F_FCR_ADCREFTRIM0_VX2_TUNE_POS             24 /**< ADCREFTRIM0_VX2_TUNE Position */
#define MXC_F_FCR_ADCREFTRIM0_VX2_TUNE                 ((uint32_t)(0x3FUL << MXC_F_FCR_ADCREFTRIM0_VX2_TUNE_POS)) /**< ADCREFTRIM0_VX2_TUNE Mask */

/**@} end of group FCR_ADCREFTRIM0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ADCREFTRIM1 FCR_ADCREFTRIM1
 * @brief    ADC Reference Trim 1 Control Register.
 * @{
 */
#define MXC_F_FCR_ADCREFTRIM1_VREFP_POS                0 /**< ADCREFTRIM1_VREFP Position */
#define MXC_F_FCR_ADCREFTRIM1_VREFP                    ((uint32_t)(0x7FUL << MXC_F_FCR_ADCREFTRIM1_VREFP_POS)) /**< ADCREFTRIM1_VREFP Mask */

#define MXC_F_FCR_ADCREFTRIM1_VREFM_POS                8 /**< ADCREFTRIM1_VREFM Position */
#define MXC_F_FCR_ADCREFTRIM1_VREFM                    ((uint32_t)(0x7FUL << MXC_F_FCR_ADCREFTRIM1_VREFM_POS)) /**< ADCREFTRIM1_VREFM Mask */

#define MXC_F_FCR_ADCREFTRIM1_VCM_POS                  16 /**< ADCREFTRIM1_VCM Position */
#define MXC_F_FCR_ADCREFTRIM1_VCM                      ((uint32_t)(0x3UL << MXC_F_FCR_ADCREFTRIM1_VCM_POS)) /**< ADCREFTRIM1_VCM Mask */

#define MXC_F_FCR_ADCREFTRIM1_VX2_TUNE_POS             24 /**< ADCREFTRIM1_VX2_TUNE Position */
#define MXC_F_FCR_ADCREFTRIM1_VX2_TUNE                 ((uint32_t)(0x3FUL << MXC_F_FCR_ADCREFTRIM1_VX2_TUNE_POS)) /**< ADCREFTRIM1_VX2_TUNE Mask */

/**@} end of group FCR_ADCREFTRIM1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ADCREFTRIM2 FCR_ADCREFTRIM2
 * @brief    ADC Reference Trim 2 Control Register.
 * @{
 */
#define MXC_F_FCR_ADCREFTRIM2_IDRV_1P25_POS            0 /**< ADCREFTRIM2_IDRV_1P25 Position */
#define MXC_F_FCR_ADCREFTRIM2_IDRV_1P25                ((uint32_t)(0xFUL << MXC_F_FCR_ADCREFTRIM2_IDRV_1P25_POS)) /**< ADCREFTRIM2_IDRV_1P25 Mask */

#define MXC_F_FCR_ADCREFTRIM2_IBOOST_1P25_POS          4 /**< ADCREFTRIM2_IBOOST_1P25 Position */
#define MXC_F_FCR_ADCREFTRIM2_IBOOST_1P25              ((uint32_t)(0x1UL << MXC_F_FCR_ADCREFTRIM2_IBOOST_1P25_POS)) /**< ADCREFTRIM2_IBOOST_1P25 Mask */

#define MXC_F_FCR_ADCREFTRIM2_IDRV_2P048_POS           8 /**< ADCREFTRIM2_IDRV_2P048 Position */
#define MXC_F_FCR_ADCREFTRIM2_IDRV_2P048               ((uint32_t)(0xFUL << MXC_F_FCR_ADCREFTRIM2_IDRV_2P048_POS)) /**< ADCREFTRIM2_IDRV_2P048 Mask */

#define MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048_POS         12 /**< ADCREFTRIM2_IBOOST_2P048 Position */
#define MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048             ((uint32_t)(0x1UL << MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048_POS)) /**< ADCREFTRIM2_IBOOST_2P048 Mask */

#define MXC_F_FCR_ADCREFTRIM2_VCM_POS                  16 /**< ADCREFTRIM2_VCM Position */
#define MXC_F_FCR_ADCREFTRIM2_VCM                      ((uint32_t)(0x3UL << MXC_F_FCR_ADCREFTRIM2_VCM_POS)) /**< ADCREFTRIM2_VCM Mask */

#define MXC_F_FCR_ADCREFTRIM2_VX2_TUNE_POS             24 /**< ADCREFTRIM2_VX2_TUNE Position */
#define MXC_F_FCR_ADCREFTRIM2_VX2_TUNE                 ((uint32_t)(0x7FUL << MXC_F_FCR_ADCREFTRIM2_VX2_TUNE_POS)) /**< ADCREFTRIM2_VX2_TUNE Mask */

/**@} end of group FCR_ADCREFTRIM2_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ERFOKS FCR_ERFOKS
 * @brief    ERFO Kick Start Control Register.
 * @{
 */
#define MXC_F_FCR_ERFOKS_CTRL_POS                      0 /**< ERFOKS_CTRL Position */
#define MXC_F_FCR_ERFOKS_CTRL                          ((uint32_t)(0xFFFFUL << MXC_F_FCR_ERFOKS_CTRL_POS)) /**< ERFOKS_CTRL Mask */

/**@} end of group FCR_ERFOKS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_FCR_REGS_H_
