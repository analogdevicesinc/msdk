/**
 * @file    fcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FCR Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_FCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_FCR_REGS_H_

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
/**@} end of group fcr_registers */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_FCTRL0 FCR_FCTRL0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_FCR_FCTRL0_USB_EXTCLK_SEL_POS            16 /**< FCTRL0_USB_EXTCLK_SEL Position */
#define MXC_F_FCR_FCTRL0_USB_EXTCLK_SEL                ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_USB_EXTCLK_SEL_POS)) /**< FCTRL0_USB_EXTCLK_SEL Mask */

#define MXC_F_FCR_FCTRL0_I2C0_SDA_FILTER_EN_POS        20 /**< FCTRL0_I2C0_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C0_SDA_FILTER_EN            ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0_SDA_FILTER_EN_POS)) /**< FCTRL0_I2C0_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C0_SCL_FILTER_EN_POS        21 /**< FCTRL0_I2C0_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C0_SCL_FILTER_EN            ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C0_SCL_FILTER_EN_POS)) /**< FCTRL0_I2C0_SCL_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C1_SDA_FILTER_EN_POS        22 /**< FCTRL0_I2C1_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C1_SDA_FILTER_EN            ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1_SDA_FILTER_EN_POS)) /**< FCTRL0_I2C1_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C1_SCL_FILTER_EN_POS        23 /**< FCTRL0_I2C1_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C1_SCL_FILTER_EN            ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C1_SCL_FILTER_EN_POS)) /**< FCTRL0_I2C1_SCL_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C2AF2_SDA_FILTER_EN_POS     24 /**< FCTRL0_I2C2AF2_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C2AF2_SDA_FILTER_EN         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2AF2_SDA_FILTER_EN_POS)) /**< FCTRL0_I2C2AF2_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C2AF2_SCL_FILTER_EN_POS     25 /**< FCTRL0_I2C2AF2_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C2AF2_SCL_FILTER_EN         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2AF2_SCL_FILTER_EN_POS)) /**< FCTRL0_I2C2AF2_SCL_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C2AF3_SDA_FILTER_EN_POS     26 /**< FCTRL0_I2C2AF3_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C2AF3_SDA_FILTER_EN         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2AF3_SDA_FILTER_EN_POS)) /**< FCTRL0_I2C2AF3_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C2AF3_SCL_FILTER_EN_POS     27 /**< FCTRL0_I2C2AF3_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C2AF3_SCL_FILTER_EN         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2AF3_SCL_FILTER_EN_POS)) /**< FCTRL0_I2C2AF3_SCL_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C2AF4_SDA_FILTER_EN_POS     28 /**< FCTRL0_I2C2AF4_SDA_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C2AF4_SDA_FILTER_EN         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2AF4_SDA_FILTER_EN_POS)) /**< FCTRL0_I2C2AF4_SDA_FILTER_EN Mask */

#define MXC_F_FCR_FCTRL0_I2C2AF4_SCL_FILTER_EN_POS     29 /**< FCTRL0_I2C2AF4_SCL_FILTER_EN Position */
#define MXC_F_FCR_FCTRL0_I2C2AF4_SCL_FILTER_EN         ((uint32_t)(0x1UL << MXC_F_FCR_FCTRL0_I2C2AF4_SCL_FILTER_EN_POS)) /**< FCTRL0_I2C2AF4_SCL_FILTER_EN Mask */

/**@} end of group FCR_FCTRL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL0 FCR_AUTOCAL0
 * @brief    Register 1.
 * @{
 */
#define MXC_F_FCR_AUTOCAL0_SEL_POS                     0 /**< AUTOCAL0_SEL Position */
#define MXC_F_FCR_AUTOCAL0_SEL                         ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_SEL_POS)) /**< AUTOCAL0_SEL Mask */

#define MXC_F_FCR_AUTOCAL0_EN_POS                      1 /**< AUTOCAL0_EN Position */
#define MXC_F_FCR_AUTOCAL0_EN                          ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_EN_POS)) /**< AUTOCAL0_EN Mask */

#define MXC_F_FCR_AUTOCAL0_LOAD_POS                    2 /**< AUTOCAL0_LOAD Position */
#define MXC_F_FCR_AUTOCAL0_LOAD                        ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_LOAD_POS)) /**< AUTOCAL0_LOAD Mask */

#define MXC_F_FCR_AUTOCAL0_INVERT_POS                  3 /**< AUTOCAL0_INVERT Position */
#define MXC_F_FCR_AUTOCAL0_INVERT                      ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_INVERT_POS)) /**< AUTOCAL0_INVERT Mask */

#define MXC_F_FCR_AUTOCAL0_ATOMIC_POS                  4 /**< AUTOCAL0_ATOMIC Position */
#define MXC_F_FCR_AUTOCAL0_ATOMIC                      ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL0_ATOMIC_POS)) /**< AUTOCAL0_ATOMIC Mask */

#define MXC_F_FCR_AUTOCAL0_GAIN_POS                    8 /**< AUTOCAL0_GAIN Position */
#define MXC_F_FCR_AUTOCAL0_GAIN                        ((uint32_t)(0xFFFUL << MXC_F_FCR_AUTOCAL0_GAIN_POS)) /**< AUTOCAL0_GAIN Mask */

#define MXC_F_FCR_AUTOCAL0_TRIM_POS                    23 /**< AUTOCAL0_TRIM Position */
#define MXC_F_FCR_AUTOCAL0_TRIM                        ((uint32_t)(0x1FFUL << MXC_F_FCR_AUTOCAL0_TRIM_POS)) /**< AUTOCAL0_TRIM Mask */

/**@} end of group FCR_AUTOCAL0_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL1 FCR_AUTOCAL1
 * @brief    Register 2.
 * @{
 */
#define MXC_F_FCR_AUTOCAL1_NFC_FWD_EN_POS              0 /**< AUTOCAL1_NFC_FWD_EN Position */
#define MXC_F_FCR_AUTOCAL1_NFC_FWD_EN                  ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_NFC_FWD_EN_POS)) /**< AUTOCAL1_NFC_FWD_EN Mask */

#define MXC_F_FCR_AUTOCAL1_NFC_CLK_EN_POS              1 /**< AUTOCAL1_NFC_CLK_EN Position */
#define MXC_F_FCR_AUTOCAL1_NFC_CLK_EN                  ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_NFC_CLK_EN_POS)) /**< AUTOCAL1_NFC_CLK_EN Mask */

#define MXC_F_FCR_AUTOCAL1_NFC_FWD_TX_DATA_OVR_POS     2 /**< AUTOCAL1_NFC_FWD_TX_DATA_OVR Position */
#define MXC_F_FCR_AUTOCAL1_NFC_FWD_TX_DATA_OVR         ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_NFC_FWD_TX_DATA_OVR_POS)) /**< AUTOCAL1_NFC_FWD_TX_DATA_OVR Mask */

#define MXC_F_FCR_AUTOCAL1_XO_EN_DGL_POS               3 /**< AUTOCAL1_XO_EN_DGL Position */
#define MXC_F_FCR_AUTOCAL1_XO_EN_DGL                   ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_XO_EN_DGL_POS)) /**< AUTOCAL1_XO_EN_DGL Mask */

#define MXC_F_FCR_AUTOCAL1_RX_BIAS_PD_POS              4 /**< AUTOCAL1_RX_BIAS_PD Position */
#define MXC_F_FCR_AUTOCAL1_RX_BIAS_PD                  ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_RX_BIAS_PD_POS)) /**< AUTOCAL1_RX_BIAS_PD Mask */

#define MXC_F_FCR_AUTOCAL1_RX_BIAS_EN_POS              5 /**< AUTOCAL1_RX_BIAS_EN Position */
#define MXC_F_FCR_AUTOCAL1_RX_BIAS_EN                  ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_RX_BIAS_EN_POS)) /**< AUTOCAL1_RX_BIAS_EN Mask */

#define MXC_F_FCR_AUTOCAL1_RX_TM_VBG_VABUS_POS         6 /**< AUTOCAL1_RX_TM_VBG_VABUS Position */
#define MXC_F_FCR_AUTOCAL1_RX_TM_VBG_VABUS             ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_RX_TM_VBG_VABUS_POS)) /**< AUTOCAL1_RX_TM_VBG_VABUS Mask */

#define MXC_F_FCR_AUTOCAL1_RX_TM_BIAS_POS              7 /**< AUTOCAL1_RX_TM_BIAS Position */
#define MXC_F_FCR_AUTOCAL1_RX_TM_BIAS                  ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_RX_TM_BIAS_POS)) /**< AUTOCAL1_RX_TM_BIAS Mask */

#define MXC_F_FCR_AUTOCAL1_NFC_FWD_DOUT_POS            8 /**< AUTOCAL1_NFC_FWD_DOUT Position */
#define MXC_F_FCR_AUTOCAL1_NFC_FWD_DOUT                ((uint32_t)(0x1UL << MXC_F_FCR_AUTOCAL1_NFC_FWD_DOUT_POS)) /**< AUTOCAL1_NFC_FWD_DOUT Mask */

/**@} end of group FCR_AUTOCAL1_Register */

/**
 * @ingroup  fcr_registers
 * @defgroup FCR_AUTOCAL2 FCR_AUTOCAL2
 * @brief    Register 3.
 * @{
 */
#define MXC_F_FCR_AUTOCAL2_RUNTIME_POS                 0 /**< AUTOCAL2_RUNTIME Position */
#define MXC_F_FCR_AUTOCAL2_RUNTIME                     ((uint32_t)(0xFFUL << MXC_F_FCR_AUTOCAL2_RUNTIME_POS)) /**< AUTOCAL2_RUNTIME Mask */

/**@} end of group FCR_AUTOCAL2_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_FCR_REGS_H_
