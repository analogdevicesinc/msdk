/**
 * @file    mcr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MCR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup mcr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_

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
    __IO uint32_t outen;                /**< <tt>\b 0x08:</tt> MCR OUTEN Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __R  uint32_t rsv_0x14_0x1f[3];
    __IO uint32_t gpio1_ctrl;           /**< <tt>\b 0x20:</tt> MCR GPIO1_CTRL Register */
    __IO uint32_t rtctrim;              /**< <tt>\b 0x24:</tt> MCR RTCTRIM Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_ECCEN                    ((uint32_t)0x00000000UL) /**< Offset from MCR Base Address: <tt> 0x0000</tt> */
#define MXC_R_MCR_OUTEN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_GPIO1_CTRL               ((uint32_t)0x00000020UL) /**< Offset from MCR Base Address: <tt> 0x0020</tt> */
#define MXC_R_MCR_RTCTRIM                  ((uint32_t)0x00000024UL) /**< Offset from MCR Base Address: <tt> 0x0024</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_ECCEN MCR_ECCEN
 * @brief    ECC Enable Register
 * @{
 */
#define MXC_F_MCR_ECCEN_FLASH_POS                      0 /**< ECCEN_FLASH Position */
#define MXC_F_MCR_ECCEN_FLASH                          ((uint32_t)(0x1UL << MXC_F_MCR_ECCEN_FLASH_POS)) /**< ECCEN_FLASH Mask */

/**@} end of group MCR_ECCEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_OUTEN MCR_OUTEN
 * @brief    Output Enable Register.
 * @{
 */
#define MXC_F_MCR_OUTEN_SQWOUT_EN_POS                  0 /**< OUTEN_SQWOUT_EN Position */
#define MXC_F_MCR_OUTEN_SQWOUT_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_SQWOUT_EN_POS)) /**< OUTEN_SQWOUT_EN Mask */

#define MXC_F_MCR_OUTEN_PDOWN_EN_POS                   1 /**< OUTEN_PDOWN_EN Position */
#define MXC_F_MCR_OUTEN_PDOWN_EN                       ((uint32_t)(0x1UL << MXC_F_MCR_OUTEN_PDOWN_EN_POS)) /**< OUTEN_PDOWN_EN Mask */

/**@} end of group MCR_OUTEN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_ERTCO_EN_POS                    3 /**< CTRL_ERTCO_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_EN                        ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_EN_POS)) /**< CTRL_ERTCO_EN Mask */

#define MXC_F_MCR_CTRL_ERTCO_32KHZ_EN_POS              5 /**< CTRL_ERTCO_32KHZ_EN Position */
#define MXC_F_MCR_CTRL_ERTCO_32KHZ_EN                  ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_ERTCO_32KHZ_EN_POS)) /**< CTRL_ERTCO_32KHZ_EN Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_GPIO1_CTRL MCR_GPIO1_CTRL
 * @brief    GPIO1 Pin Control Register.
 * @{
 */
#define MXC_F_MCR_GPIO1_CTRL_P1_0_OUT_POS              0 /**< GPIO1_CTRL_P1_0_OUT Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_0_OUT                  ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_0_OUT_POS)) /**< GPIO1_CTRL_P1_0_OUT Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_0_OUTEN_POS            1 /**< GPIO1_CTRL_P1_0_OUTEN Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_0_OUTEN                ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_0_OUTEN_POS)) /**< GPIO1_CTRL_P1_0_OUTEN Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_0_PUPEN_POS            2 /**< GPIO1_CTRL_P1_0_PUPEN Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_0_PUPEN                ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_0_PUPEN_POS)) /**< GPIO1_CTRL_P1_0_PUPEN Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_0_IN_POS               3 /**< GPIO1_CTRL_P1_0_IN Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_0_IN                   ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_0_IN_POS)) /**< GPIO1_CTRL_P1_0_IN Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_1_OUT_POS              4 /**< GPIO1_CTRL_P1_1_OUT Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_1_OUT                  ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_1_OUT_POS)) /**< GPIO1_CTRL_P1_1_OUT Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_1_OUTEN_POS            5 /**< GPIO1_CTRL_P1_1_OUTEN Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_1_OUTEN                ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_1_OUTEN_POS)) /**< GPIO1_CTRL_P1_1_OUTEN Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_1_PUPEN_POS            6 /**< GPIO1_CTRL_P1_1_PUPEN Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_1_PUPEN                ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_1_PUPEN_POS)) /**< GPIO1_CTRL_P1_1_PUPEN Mask */

#define MXC_F_MCR_GPIO1_CTRL_P1_1_IN_POS               7 /**< GPIO1_CTRL_P1_1_IN Position */
#define MXC_F_MCR_GPIO1_CTRL_P1_1_IN                   ((uint32_t)(0x1UL << MXC_F_MCR_GPIO1_CTRL_P1_1_IN_POS)) /**< GPIO1_CTRL_P1_1_IN Mask */

/**@} end of group MCR_GPIO1_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RTCTRIM MCR_RTCTRIM
 * @brief    User RTC Trim Register.
 * @{
 */
#define MXC_F_MCR_RTCTRIM_X2_POS                       0 /**< RTCTRIM_X2 Position */
#define MXC_F_MCR_RTCTRIM_X2                           ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_X2_POS)) /**< RTCTRIM_X2 Mask */

#define MXC_F_MCR_RTCTRIM_X1_POS                       5 /**< RTCTRIM_X1 Position */
#define MXC_F_MCR_RTCTRIM_X1                           ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_X1_POS)) /**< RTCTRIM_X1 Mask */

/**@} end of group MCR_RTCTRIM_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MCR_REGS_H_
