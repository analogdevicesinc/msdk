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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> FCR CTRL Register */
    __IO uint32_t autocal0;             /**< <tt>\b 0x04:</tt> FCR AUTOCAL0 Register */
    __IO uint32_t autocal1;             /**< <tt>\b 0x08:</tt> FCR AUTOCAL1 Register */
    __IO uint32_t autocal2;             /**< <tt>\b 0x0C:</tt> FCR AUTOCAL2 Register */
    __R  uint32_t rsv_0x10_0x17[2];
    __IO uint32_t erfoks;               /**< <tt>\b 0x18:</tt> FCR ERFOKS Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x1C:</tt> FCR INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x20:</tt> FCR INTEN Register */
    __IO uint32_t erfoctrl;             /**< <tt>\b 0x24:</tt> FCR ERFOCTRL Register */
    __IO uint32_t frqcntctrl;           /**< <tt>\b 0x28:</tt> FCR FRQCNTCTRL Register */
    __IO uint32_t frqcntcmp;            /**< <tt>\b 0x2C:</tt> FCR FRQCNTCMP Register */
    __I  uint32_t refclk;               /**< <tt>\b 0x30:</tt> FCR REFCLK Register */
    __I  uint32_t cmpclk;               /**< <tt>\b 0x34:</tt> FCR CMPCLK Register */
} mxc_fcr_regs_t;

/* Register offsets for module FCR */
/**
 * @ingroup    fcr_registers
 * @defgroup   FCR_Register_Offsets Register Offsets
 * @brief      FCR Peripheral Register Offsets from the FCR Base Peripheral Address.
 * @{
 */
#define MXC_R_FCR_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from FCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_FCR_AUTOCAL0                 ((uint32_t)0x00000004UL) /**< Offset from FCR Base Address: <tt> 0x0004</tt> */
#define MXC_R_FCR_AUTOCAL1                 ((uint32_t)0x00000008UL) /**< Offset from FCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_FCR_AUTOCAL2                 ((uint32_t)0x0000000CUL) /**< Offset from FCR Base Address: <tt> 0x000C</tt> */
#define MXC_R_FCR_ERFOKS                   ((uint32_t)0x00000018UL) /**< Offset from FCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_FCR_INTFL                    ((uint32_t)0x0000001CUL) /**< Offset from FCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_FCR_INTEN                    ((uint32_t)0x00000020UL) /**< Offset from FCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_FCR_ERFOCTRL                 ((uint32_t)0x00000024UL) /**< Offset from FCR Base Address: <tt> 0x0024</tt> */
#define MXC_R_FCR_FRQCNTCTRL               ((uint32_t)0x00000028UL) /**< Offset from FCR Base Address: <tt> 0x0028</tt> */
#define MXC_R_FCR_FRQCNTCMP                ((uint32_t)0x0000002CUL) /**< Offset from FCR Base Address: <tt> 0x002C</tt> */
#define MXC_R_FCR_REFCLK                   ((uint32_t)0x00000030UL) /**< Offset from FCR Base Address: <tt> 0x0030</tt> */
#define MXC_R_FCR_CMPCLK                   ((uint32_t)0x00000034UL) /**< Offset from FCR Base Address: <tt> 0x0034</tt> */
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_CTRL FCR_CTRL
 * @brief    Function Control 0 Register.
 * @{
 */
#define MXC_F_FCR_CTRL_I3CDGEN0_POS                    20 /**< CTRL_I3CDGEN0 Position */
#define MXC_F_FCR_CTRL_I3CDGEN0                        ((uint32_t)(0x1UL << MXC_F_FCR_CTRL_I3CDGEN0_POS)) /**< CTRL_I3CDGEN0 Mask */

#define MXC_F_FCR_CTRL_I3CDGEN1_POS                    21 /**< CTRL_I3CDGEN1 Position */
#define MXC_F_FCR_CTRL_I3CDGEN1                        ((uint32_t)(0x1UL << MXC_F_FCR_CTRL_I3CDGEN1_POS)) /**< CTRL_I3CDGEN1 Mask */

/**@} end of group FCR_CTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL0 FCR_AUTOCAL0
 * @brief    Automatic Calibration 0 Register.
 * @{
 */
#define MXC_F_FCR_AUTOCAL0_EN_POS                      0 /**< AUTOCAL0_EN Position */
#define MXC_F_FCR_AUTOCAL0_EN                          ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_EN_POS)) /**< AUTOCAL0_EN Mask */

#define MXC_F_FCR_AUTOCAL0_RUN_POS                     1 /**< AUTOCAL0_RUN Position */
#define MXC_F_FCR_AUTOCAL0_RUN                         ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_RUN_POS)) /**< AUTOCAL0_RUN Mask */

#define MXC_F_FCR_AUTOCAL0_LOAD_TRIM_POS               2 /**< AUTOCAL0_LOAD_TRIM Position */
#define MXC_F_FCR_AUTOCAL0_LOAD_TRIM                   ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_LOAD_TRIM_POS)) /**< AUTOCAL0_LOAD_TRIM Mask */

#define MXC_F_FCR_AUTOCAL0_GAIN_INV_POS                3 /**< AUTOCAL0_GAIN_INV Position */
#define MXC_F_FCR_AUTOCAL0_GAIN_INV                    ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_GAIN_INV_POS)) /**< AUTOCAL0_GAIN_INV Mask */

#define MXC_F_FCR_AUTOCAL0_ATOMIC_POS                  4 /**< AUTOCAL0_ATOMIC Position */
#define MXC_F_FCR_AUTOCAL0_ATOMIC                      ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_ATOMIC_POS)) /**< AUTOCAL0_ATOMIC Mask */

#define MXC_F_FCR_AUTOCAL0_MU_POS                      8 /**< AUTOCAL0_MU Position */
#define MXC_F_FCR_AUTOCAL0_MU                          ((uint32_t)(0xFFFUL << MXC_F_FCR_AUTOCAL0_MU_POS)) /**< AUTOCAL0_MU Mask */

#define MXC_F_FCR_AUTOCAL0_TRIM_OUT_POS                23 /**< AUTOCAL0_TRIM_OUT Position */
#define MXC_F_FCR_AUTOCAL0_TRIM_OUT                    ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL0_TRIM_OUT_POS)) /**< AUTOCAL0_TRIM_OUT Mask */

/**@} end of group FCR_AUTOCAL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL1 FCR_AUTOCAL1
 * @brief    Automatic Calibration 1 Register.
 * @{
 */
#define MXC_F_FCR_AUTOCAL1_INIT_TRIM_POS               0 /**< AUTOCAL1_INIT_TRIM Position */
#define MXC_F_FCR_AUTOCAL1_INIT_TRIM                   ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL1_INIT_TRIM_POS)) /**< AUTOCAL1_INIT_TRIM Mask */

/**@} end of group FCR_AUTOCAL1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL2 FCR_AUTOCAL2
 * @brief    Automatic Calibration 2 Register.
 * @{
 */
#define MXC_F_FCR_AUTOCAL2_RUNTIME_POS                 0 /**< AUTOCAL2_RUNTIME Position */
#define MXC_F_FCR_AUTOCAL2_RUNTIME                     ((uint32_t)(0xFFUL << MXC_F_FCR_AUTOCAL2_RUNTIME_POS)) /**< AUTOCAL2_RUNTIME Mask */

#define MXC_F_FCR_AUTOCAL2_DIV_POS                     8 /**< AUTOCAL2_DIV Position */
#define MXC_F_FCR_AUTOCAL2_DIV                         ((uint32_t)(0x1FFFUL << MXC_F_FCR_AUTOCAL2_DIV_POS)) /**< AUTOCAL2_DIV Mask */

/**@} end of group FCR_AUTOCAL2_Register */

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

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_INTFL FCR_INTFL
 * @brief    Interrupt Flag Register.
 * @{
 */
#define MXC_F_FCR_INTFL_ERFO_RDY_POS                   0 /**< INTFL_ERFO_RDY Position */
#define MXC_F_FCR_INTFL_ERFO_RDY                       ((uint32_t)(0x1UL << MXC_F_FCR_INTFL_ERFO_RDY_POS)) /**< INTFL_ERFO_RDY Mask */

#define MXC_F_FCR_INTFL_FRQCNT_POS                     1 /**< INTFL_FRQCNT Position */
#define MXC_F_FCR_INTFL_FRQCNT                         ((uint32_t)(0x1UL << MXC_F_FCR_INTFL_FRQCNT_POS)) /**< INTFL_FRQCNT Mask */

/**@} end of group FCR_INTFL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_INTEN FCR_INTEN
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_FCR_INTEN_ERFO_RDY_POS                   0 /**< INTEN_ERFO_RDY Position */
#define MXC_F_FCR_INTEN_ERFO_RDY                       ((uint32_t)(0x1UL << MXC_F_FCR_INTEN_ERFO_RDY_POS)) /**< INTEN_ERFO_RDY Mask */

#define MXC_F_FCR_INTEN_FRQCNT_POS                     1 /**< INTEN_FRQCNT Position */
#define MXC_F_FCR_INTEN_FRQCNT                         ((uint32_t)(0x1UL << MXC_F_FCR_INTEN_FRQCNT_POS)) /**< INTEN_FRQCNT Mask */

/**@} end of group FCR_INTEN_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_ERFOCTRL FCR_ERFOCTRL
 * @brief    ERFO Control Register.
 * @{
 */
#define MXC_F_FCR_ERFOCTRL_CAP_X1_POS                  0 /**< ERFOCTRL_CAP_X1 Position */
#define MXC_F_FCR_ERFOCTRL_CAP_X1                      ((uint32_t)(0x7FUL << MXC_F_FCR_ERFOCTRL_CAP_X1_POS)) /**< ERFOCTRL_CAP_X1 Mask */

#define MXC_F_FCR_ERFOCTRL_CAP_X2_POS                  7 /**< ERFOCTRL_CAP_X2 Position */
#define MXC_F_FCR_ERFOCTRL_CAP_X2                      ((uint32_t)(0x7FUL << MXC_F_FCR_ERFOCTRL_CAP_X2_POS)) /**< ERFOCTRL_CAP_X2 Mask */

#define MXC_F_FCR_ERFOCTRL_CAP_BYPASS_POS              14 /**< ERFOCTRL_CAP_BYPASS Position */
#define MXC_F_FCR_ERFOCTRL_CAP_BYPASS                  ((uint32_t)(0x1UL << MXC_F_FCR_ERFOCTRL_CAP_BYPASS_POS)) /**< ERFOCTRL_CAP_BYPASS Mask */

/**@} end of group FCR_ERFOCTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FRQCNTCTRL FCR_FRQCNTCTRL
 * @brief    Frequency Counter Control Register.
 * @{
 */
#define MXC_F_FCR_FRQCNTCTRL_START_POS                 0 /**< FRQCNTCTRL_START Position */
#define MXC_F_FCR_FRQCNTCTRL_START                     ((uint32_t)(0x1UL << MXC_F_FCR_FRQCNTCTRL_START_POS)) /**< FRQCNTCTRL_START Mask */

#define MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL_POS            1 /**< FRQCNTCTRL_CMP_CLKSEL Position */
#define MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL                ((uint32_t)(0x3UL << MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL_POS)) /**< FRQCNTCTRL_CMP_CLKSEL Mask */
#define MXC_V_FCR_FRQCNTCTRL_CMP_CLKSEL_RTC            ((uint32_t)0x0UL) /**< FRQCNTCTRL_CMP_CLKSEL_RTC Value */
#define MXC_S_FCR_FRQCNTCTRL_CMP_CLKSEL_RTC            (MXC_V_FCR_FRQCNTCTRL_CMP_CLKSEL_RTC << MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL_POS) /**< FRQCNTCTRL_CMP_CLKSEL_RTC Setting */
#define MXC_V_FCR_FRQCNTCTRL_CMP_CLKSEL_EXT_GPIO       ((uint32_t)0x2UL) /**< FRQCNTCTRL_CMP_CLKSEL_EXT_GPIO Value */
#define MXC_S_FCR_FRQCNTCTRL_CMP_CLKSEL_EXT_GPIO       (MXC_V_FCR_FRQCNTCTRL_CMP_CLKSEL_EXT_GPIO << MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL_POS) /**< FRQCNTCTRL_CMP_CLKSEL_EXT_GPIO Setting */
#define MXC_V_FCR_FRQCNTCTRL_CMP_CLKSEL_INRO           ((uint32_t)0x3UL) /**< FRQCNTCTRL_CMP_CLKSEL_INRO Value */
#define MXC_S_FCR_FRQCNTCTRL_CMP_CLKSEL_INRO           (MXC_V_FCR_FRQCNTCTRL_CMP_CLKSEL_INRO << MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL_POS) /**< FRQCNTCTRL_CMP_CLKSEL_INRO Setting */

/**@} end of group FCR_FRQCNTCTRL_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FRQCNTCMP FCR_FRQCNTCMP
 * @brief    Frequency Counter Compared Target Register.
 * @{
 */
#define MXC_F_FCR_FRQCNTCMP_TARGET_POS                 0 /**< FRQCNTCMP_TARGET Position */
#define MXC_F_FCR_FRQCNTCMP_TARGET                     ((uint32_t)(0x3FFFUL << MXC_F_FCR_FRQCNTCMP_TARGET_POS)) /**< FRQCNTCMP_TARGET Mask */

/**@} end of group FCR_FRQCNTCMP_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_REFCLK FCR_REFCLK
 * @brief    Reference Clock Result Register.
 * @{
 */
#define MXC_F_FCR_REFCLK_RESULT_POS                    0 /**< REFCLK_RESULT Position */
#define MXC_F_FCR_REFCLK_RESULT                        ((uint32_t)(0xFFFFFUL << MXC_F_FCR_REFCLK_RESULT_POS)) /**< REFCLK_RESULT Mask */

/**@} end of group FCR_REFCLK_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_CMPCLK FCR_CMPCLK
 * @brief    Compared Clock Result Register.
 * @{
 */
#define MXC_F_FCR_CMPCLK_RESULT_POS                    0 /**< CMPCLK_RESULT Position */
#define MXC_F_FCR_CMPCLK_RESULT                        ((uint32_t)(0x3FFFUL << MXC_F_FCR_CMPCLK_RESULT_POS)) /**< CMPCLK_RESULT Mask */

/**@} end of group FCR_CMPCLK_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FCR_REGS_H_
