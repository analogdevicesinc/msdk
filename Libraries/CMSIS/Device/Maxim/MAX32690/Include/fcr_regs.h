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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_FCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_FCR_REGS_H_

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
    __IO uint32_t urvbootaddr;          /**< <tt>\b 0x10:</tt> FCR URVBOOTADDR Register */
    __IO uint32_t urvctrl;              /**< <tt>\b 0x14:</tt> FCR URVCTRL Register */
    __IO uint32_t xo32mks;              /**< <tt>\b 0x18:</tt> FCR XO32MKS Register */
    __IO uint32_t sarbufcn;             /**< <tt>\b 0x1C:</tt> FCR SARBUFCN Register */
    __IO uint32_t ts0;                  /**< <tt>\b 0x20:</tt> FCR TS0 Register */
    __IO uint32_t ts1;                  /**< <tt>\b 0x24:</tt> FCR TS1 Register */
    __IO uint32_t adcreftrim0;          /**< <tt>\b 0x28:</tt> FCR ADCREFTRIM0 Register */
    __IO uint32_t adcreftrim1;          /**< <tt>\b 0x2C:</tt> FCR ADCREFTRIM1 Register */
    __IO uint32_t adcreftrim2;          /**< <tt>\b 0x30:</tt> FCR ADCREFTRIM2 Register */
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
#define MXC_R_FCR_URVBOOTADDR              ((uint32_t)0x00000010UL) /**< Offset from FCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_FCR_URVCTRL                  ((uint32_t)0x00000014UL) /**< Offset from FCR Base Address: <tt> 0x0014</tt> */
#define MXC_R_FCR_XO32MKS                  ((uint32_t)0x00000018UL) /**< Offset from FCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_FCR_SARBUFCN                 ((uint32_t)0x0000001CUL) /**< Offset from FCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_FCR_TS0                      ((uint32_t)0x00000020UL) /**< Offset from FCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_FCR_TS1                      ((uint32_t)0x00000024UL) /**< Offset from FCR Base Address: <tt> 0x0024</tt> */
#define MXC_R_FCR_ADCREFTRIM0              ((uint32_t)0x00000028UL) /**< Offset from FCR Base Address: <tt> 0x0028</tt> */
#define MXC_R_FCR_ADCREFTRIM1              ((uint32_t)0x0000002CUL) /**< Offset from FCR Base Address: <tt> 0x002C</tt> */
#define MXC_R_FCR_ADCREFTRIM2              ((uint32_t)0x00000030UL) /**< Offset from FCR Base Address: <tt> 0x0030</tt> */
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL0 FCR_FCTRL0
 * @brief    Function Control 0.
 * @{
 */
#define MXC_F_FCR_FCTRL0_RDSGCSEL_POS                  0 /**< FCTRL0_RDSGCSEL Position */
#define MXC_F_FCR_FCTRL0_RDSGCSEL                      ((uint32_t)(0x3FUL << MXC_F_FCR_FCTRL0_RDSGCSEL_POS)) /**< FCTRL0_RDSGCSEL Mask */

#define MXC_F_FCR_FCTRL0_RDSGCSET_POS                  6 /**< FCTRL0_RDSGCSET Position */
#define MXC_F_FCR_FCTRL0_RDSGCSET                      ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_RDSGCSET_POS)) /**< FCTRL0_RDSGCSET Mask */

#define MXC_F_FCR_FCTRL0_HYPERCGDLY_POS                8 /**< FCTRL0_HYPERCGDLY Position */
#define MXC_F_FCR_FCTRL0_HYPERCGDLY                    ((uint32_t)(0x3FUL << MXC_F_FCR_FCTRL0_HYPERCGDLY_POS)) /**< FCTRL0_HYPERCGDLY Mask */

#define MXC_F_FCR_FCTRL0_USBCLKSEL_POS                 16 /**< FCTRL0_USBCLKSEL Position */
#define MXC_F_FCR_FCTRL0_USBCLKSEL                     ((uint32_t)(0x3UL << MXC_F_FCR_FCTRL0_USBCLKSEL_POS)) /**< FCTRL0_USBCLKSEL Mask */

#define MXC_F_FCR_FCTRL0_I2C0DGEN0_POS                 20 /**< FCTRL0_I2C0DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C0DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN0_POS)) /**< FCTRL0_I2C0DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C0DGEN1_POS                 21 /**< FCTRL0_I2C0DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C0DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0DGEN1_POS)) /**< FCTRL0_I2C0DGEN1 Mask */

#define MXC_F_FCR_FCTRL0_I2C1DGEN0_POS                 22 /**< FCTRL0_I2C1DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C1DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN0_POS)) /**< FCTRL0_I2C1DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C1DGEN1_POS                 23 /**< FCTRL0_I2C1DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C1DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1DGEN1_POS)) /**< FCTRL0_I2C1DGEN1 Mask */

#define MXC_F_FCR_FCTRL0_I2C2DGEN0_POS                 24 /**< FCTRL0_I2C2DGEN0 Position */
#define MXC_F_FCR_FCTRL0_I2C2DGEN0                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2DGEN0_POS)) /**< FCTRL0_I2C2DGEN0 Mask */

#define MXC_F_FCR_FCTRL0_I2C2DGEN1_POS                 25 /**< FCTRL0_I2C2DGEN1 Position */
#define MXC_F_FCR_FCTRL0_I2C2DGEN1                     ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2DGEN1_POS)) /**< FCTRL0_I2C2DGEN1 Mask */

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

#define MXC_F_FCR_AUTOCAL0_LDTRM_POS                   2 /**< AUTOCAL0_LDTRM Position */
#define MXC_F_FCR_AUTOCAL0_LDTRM                       ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_LDTRM_POS)) /**< AUTOCAL0_LDTRM Mask */

#define MXC_F_FCR_AUTOCAL0_GAININV_POS                 3 /**< AUTOCAL0_GAININV Position */
#define MXC_F_FCR_AUTOCAL0_GAININV                     ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_GAININV_POS)) /**< AUTOCAL0_GAININV Mask */

#define MXC_F_FCR_AUTOCAL0_ATOMIC_POS                  4 /**< AUTOCAL0_ATOMIC Position */
#define MXC_F_FCR_AUTOCAL0_ATOMIC                      ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_ATOMIC_POS)) /**< AUTOCAL0_ATOMIC Mask */

#define MXC_F_FCR_AUTOCAL0_MU_POS                      8 /**< AUTOCAL0_MU Position */
#define MXC_F_FCR_AUTOCAL0_MU                          ((uint32_t)(0xFFFUL << MXC_F_FCR_AUTOCAL0_MU_POS)) /**< AUTOCAL0_MU Mask */

#define MXC_F_FCR_AUTOCAL0_HIRC96MACTMROUT_POS         23 /**< AUTOCAL0_HIRC96MACTMROUT Position */
#define MXC_F_FCR_AUTOCAL0_HIRC96MACTMROUT             ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL0_HIRC96MACTMROUT_POS)) /**< AUTOCAL0_HIRC96MACTMROUT Mask */

/**@} end of group FCR_AUTOCAL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL1 FCR_AUTOCAL1
 * @brief    Automatic Calibration 1.
 * @{
 */
#define MXC_F_FCR_AUTOCAL1_INITTRM_POS                 0 /**< AUTOCAL1_INITTRM Position */
#define MXC_F_FCR_AUTOCAL1_INITTRM                     ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL1_INITTRM_POS)) /**< AUTOCAL1_INITTRM Mask */

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
 * @defgroup FCR_URVCTRL FCR_URVCTRL
 * @brief    RISC-V Control Register.
 * @{
 */
#define MXC_F_FCR_URVCTRL_MEMSEL_POS                   0 /**< URVCTRL_MEMSEL Position */
#define MXC_F_FCR_URVCTRL_MEMSEL                       ((uint32_t)(0x1UL << MXC_F_FCR_URVCTRL_MEMSEL_POS)) /**< URVCTRL_MEMSEL Mask */

#define MXC_F_FCR_URVCTRL_IFLUSHEN_POS                 1 /**< URVCTRL_IFLUSHEN Position */
#define MXC_F_FCR_URVCTRL_IFLUSHEN                     ((uint32_t)(0x1UL << MXC_F_FCR_URVCTRL_IFLUSHEN_POS)) /**< URVCTRL_IFLUSHEN Mask */

/**@} end of group FCR_URVCTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_XO32MKS FCR_XO32MKS
 * @brief    RISC-V Control Register.
 * @{
 */
#define MXC_F_FCR_XO32MKS_CLK_POS                      0 /**< XO32MKS_CLK Position */
#define MXC_F_FCR_XO32MKS_CLK                          ((uint32_t)(0x7FUL << MXC_F_FCR_XO32MKS_CLK_POS)) /**< XO32MKS_CLK Mask */

#define MXC_F_FCR_XO32MKS_EN_POS                       7 /**< XO32MKS_EN Position */
#define MXC_F_FCR_XO32MKS_EN                           ((uint32_t)(0x1UL << MXC_F_FCR_XO32MKS_EN_POS)) /**< XO32MKS_EN Mask */

#define MXC_F_FCR_XO32MKS_DRIVER_POS                   8 /**< XO32MKS_DRIVER Position */
#define MXC_F_FCR_XO32MKS_DRIVER                       ((uint32_t)(0x7UL << MXC_F_FCR_XO32MKS_DRIVER_POS)) /**< XO32MKS_DRIVER Mask */

#define MXC_F_FCR_XO32MKS_PULSE_POS                    11 /**< XO32MKS_PULSE Position */
#define MXC_F_FCR_XO32MKS_PULSE                        ((uint32_t)(0x1UL << MXC_F_FCR_XO32MKS_PULSE_POS)) /**< XO32MKS_PULSE Mask */

#define MXC_F_FCR_XO32MKS_CLKSEL_POS                   12 /**< XO32MKS_CLKSEL Position */
#define MXC_F_FCR_XO32MKS_CLKSEL                       ((uint32_t)(0x3UL << MXC_F_FCR_XO32MKS_CLKSEL_POS)) /**< XO32MKS_CLKSEL Mask */
#define MXC_V_FCR_XO32MKS_CLKSEL_NONE                  ((uint32_t)0x0UL) /**< XO32MKS_CLKSEL_NONE Value */
#define MXC_S_FCR_XO32MKS_CLKSEL_NONE                  (MXC_V_FCR_XO32MKS_CLKSEL_NONE << MXC_F_FCR_XO32MKS_CLKSEL_POS) /**< XO32MKS_CLKSEL_NONE Setting */
#define MXC_V_FCR_XO32MKS_CLKSEL_TEST                  ((uint32_t)0x1UL) /**< XO32MKS_CLKSEL_TEST Value */
#define MXC_S_FCR_XO32MKS_CLKSEL_TEST                  (MXC_V_FCR_XO32MKS_CLKSEL_TEST << MXC_F_FCR_XO32MKS_CLKSEL_POS) /**< XO32MKS_CLKSEL_TEST Setting */
#define MXC_V_FCR_XO32MKS_CLKSEL_ISO                   ((uint32_t)0x2UL) /**< XO32MKS_CLKSEL_ISO Value */
#define MXC_S_FCR_XO32MKS_CLKSEL_ISO                   (MXC_V_FCR_XO32MKS_CLKSEL_ISO << MXC_F_FCR_XO32MKS_CLKSEL_POS) /**< XO32MKS_CLKSEL_ISO Setting */
#define MXC_V_FCR_XO32MKS_CLKSEL_IPO                   ((uint32_t)0x3UL) /**< XO32MKS_CLKSEL_IPO Value */
#define MXC_S_FCR_XO32MKS_CLKSEL_IPO                   (MXC_V_FCR_XO32MKS_CLKSEL_IPO << MXC_F_FCR_XO32MKS_CLKSEL_POS) /**< XO32MKS_CLKSEL_IPO Setting */

/**@} end of group FCR_XO32MKS_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_SARBUFCN FCR_SARBUFCN
 * @brief    TBD
 * @{
 */
#define MXC_F_FCR_SARBUFCN_THRU_PAD_SW_EN_POS          0 /**< SARBUFCN_THRU_PAD_SW_EN Position */
#define MXC_F_FCR_SARBUFCN_THRU_PAD_SW_EN              ((uint32_t)(0xFFUL << MXC_F_FCR_SARBUFCN_THRU_PAD_SW_EN_POS)) /**< SARBUFCN_THRU_PAD_SW_EN Mask */

#define MXC_F_FCR_SARBUFCN_THRU_EN_POS                 8 /**< SARBUFCN_THRU_EN Position */
#define MXC_F_FCR_SARBUFCN_THRU_EN                     ((uint32_t)(0x1UL << MXC_F_FCR_SARBUFCN_THRU_EN_POS)) /**< SARBUFCN_THRU_EN Mask */

#define MXC_F_FCR_SARBUFCN_RAMP_EN_POS                 9 /**< SARBUFCN_RAMP_EN Position */
#define MXC_F_FCR_SARBUFCN_RAMP_EN                     ((uint32_t)(0x1UL << MXC_F_FCR_SARBUFCN_RAMP_EN_POS)) /**< SARBUFCN_RAMP_EN Mask */

#define MXC_F_FCR_SARBUFCN_THRU_RRI_EN_POS             10 /**< SARBUFCN_THRU_RRI_EN Position */
#define MXC_F_FCR_SARBUFCN_THRU_RRI_EN                 ((uint32_t)(0x1UL << MXC_F_FCR_SARBUFCN_THRU_RRI_EN_POS)) /**< SARBUFCN_THRU_RRI_EN Mask */

#define MXC_F_FCR_SARBUFCN_DIVSEL_POS                  11 /**< SARBUFCN_DIVSEL Position */
#define MXC_F_FCR_SARBUFCN_DIVSEL                      ((uint32_t)(0x1UL << MXC_F_FCR_SARBUFCN_DIVSEL_POS)) /**< SARBUFCN_DIVSEL Mask */

/**@} end of group FCR_SARBUFCN_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_TS0 FCR_TS0
 * @brief    Temp Sensor trim0
 * @{
 */
#define MXC_F_FCR_TS0_GAIN_POS                         0 /**< TS0_GAIN Position */
#define MXC_F_FCR_TS0_GAIN                             ((uint32_t)(0xFFFUL << MXC_F_FCR_TS0_GAIN_POS)) /**< TS0_GAIN Mask */

/**@} end of group FCR_TS0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_TS1 FCR_TS1
 * @brief    Temp Sensor trim1
 * @{
 */
#define MXC_F_FCR_TS1_OFFSET_POS                       0 /**< TS1_OFFSET Position */
#define MXC_F_FCR_TS1_OFFSET                           ((uint32_t)(0x3FFFUL << MXC_F_FCR_TS1_OFFSET_POS)) /**< TS1_OFFSET Mask */

#define MXC_F_FCR_TS1_TS_OFFSET_SIGN_POS               14 /**< TS1_TS_OFFSET_SIGN Position */
#define MXC_F_FCR_TS1_TS_OFFSET_SIGN                   ((uint32_t)(0x3FFFFUL << MXC_F_FCR_TS1_TS_OFFSET_SIGN_POS)) /**< TS1_TS_OFFSET_SIGN Mask */

/**@} end of group FCR_TS1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ADCREFTRIM0 FCR_ADCREFTRIM0
 * @brief    Temp Sensor trim1
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
 * @brief    Temp Sensor trim1
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
 * @brief    Temp Sensor trim1
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
#define MXC_F_FCR_ADCREFTRIM2_VX2_TUNE                 ((uint32_t)(0x3FUL << MXC_F_FCR_ADCREFTRIM2_VX2_TUNE_POS)) /**< ADCREFTRIM2_VX2_TUNE Mask */

/**@} end of group FCR_ADCREFTRIM2_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_FCR_REGS_H_
