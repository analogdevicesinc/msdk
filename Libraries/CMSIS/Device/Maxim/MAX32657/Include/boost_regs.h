/**
 * @file    boost_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the BOOST Peripheral Module.
 * @note    This file is @generated.
 * @ingroup boost_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_BOOST_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_BOOST_REGS_H_

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
 * @ingroup     boost
 * @defgroup    boost_registers BOOST_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the BOOST Peripheral Module.
 * @details     Boost Controller
 */

/**
 * @ingroup boost_registers
 * Structure type to access the BOOST Registers.
 */
typedef struct {
    __IO uint32_t disable;              /**< <tt>\b 0x000:</tt> BOOST DISABLE Register */
    __IO uint32_t vregctrl;             /**< <tt>\b 0x004:</tt> BOOST VREGCTRL Register */
    __I  uint32_t ipeak;                /**< <tt>\b 0x008:</tt> BOOST IPEAK Register */
    __I  uint32_t maxton;               /**< <tt>\b 0x00C:</tt> BOOST MAXTON Register */
    __I  uint32_t iload;                /**< <tt>\b 0x010:</tt> BOOST ILOAD Register */
    __IO uint32_t alert;                /**< <tt>\b 0x014:</tt> BOOST ALERT Register */
    __I  uint32_t rdy;                  /**< <tt>\b 0x018:</tt> BOOST RDY Register */
    __I  uint32_t zxcal;                /**< <tt>\b 0x01C:</tt> BOOST ZXCAL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x020:</tt> BOOST INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x024:</tt> BOOST INTFL Register */
} mxc_boost_regs_t;

/* Register offsets for module BOOST */
/**
 * @ingroup    boost_registers
 * @defgroup   BOOST_Register_Offsets Register Offsets
 * @brief      BOOST Peripheral Register Offsets from the BOOST Base Peripheral Address.
 * @{
 */
#define MXC_R_BOOST_DISABLE                ((uint32_t)0x00000000UL) /**< Offset from BOOST Base Address: <tt> 0x0000</tt> */
#define MXC_R_BOOST_VREGCTRL               ((uint32_t)0x00000004UL) /**< Offset from BOOST Base Address: <tt> 0x0004</tt> */
#define MXC_R_BOOST_IPEAK                  ((uint32_t)0x00000008UL) /**< Offset from BOOST Base Address: <tt> 0x0008</tt> */
#define MXC_R_BOOST_MAXTON                 ((uint32_t)0x0000000CUL) /**< Offset from BOOST Base Address: <tt> 0x000C</tt> */
#define MXC_R_BOOST_ILOAD                  ((uint32_t)0x00000010UL) /**< Offset from BOOST Base Address: <tt> 0x0010</tt> */
#define MXC_R_BOOST_ALERT                  ((uint32_t)0x00000014UL) /**< Offset from BOOST Base Address: <tt> 0x0014</tt> */
#define MXC_R_BOOST_RDY                    ((uint32_t)0x00000018UL) /**< Offset from BOOST Base Address: <tt> 0x0018</tt> */
#define MXC_R_BOOST_ZXCAL                  ((uint32_t)0x0000001CUL) /**< Offset from BOOST Base Address: <tt> 0x001C</tt> */
#define MXC_R_BOOST_INTEN                  ((uint32_t)0x00000020UL) /**< Offset from BOOST Base Address: <tt> 0x0020</tt> */
#define MXC_R_BOOST_INTFL                  ((uint32_t)0x00000024UL) /**< Offset from BOOST Base Address: <tt> 0x0024</tt> */
/**@} end of group boost_registers */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_DISABLE BOOST_DISABLE
 * @brief    Boost Disable Register.
 * @{
 */
#define MXC_F_BOOST_DISABLE_DIS_POS                    0 /**< DISABLE_DIS Position */
#define MXC_F_BOOST_DISABLE_DIS                        ((uint32_t)(0x1UL << MXC_F_BOOST_DISABLE_DIS_POS)) /**< DISABLE_DIS Mask */

/**@} end of group BOOST_DISABLE_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_VREGCTRL BOOST_VREGCTRL
 * @brief    Boost Voltage Regulator Control Register.
 * @{
 */
#define MXC_F_BOOST_VREGCTRL_SET_POS                   0 /**< VREGCTRL_SET Position */
#define MXC_F_BOOST_VREGCTRL_SET                       ((uint32_t)(0x1FUL << MXC_F_BOOST_VREGCTRL_SET_POS)) /**< VREGCTRL_SET Mask */

/**@} end of group BOOST_VREGCTRL_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_IPEAK BOOST_IPEAK
 * @brief    Low Side FET Peak Current Register.
 * @{
 */
#define MXC_F_BOOST_IPEAK_SET_POS                      0 /**< IPEAK_SET Position */
#define MXC_F_BOOST_IPEAK_SET                          ((uint32_t)(0x7UL << MXC_F_BOOST_IPEAK_SET_POS)) /**< IPEAK_SET Mask */

/**@} end of group BOOST_IPEAK_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_MAXTON BOOST_MAXTON
 * @brief    Maximum Low Side FET Time-On Register.
 * @{
 */
#define MXC_F_BOOST_MAXTON_THD_POS                     0 /**< MAXTON_THD Position */
#define MXC_F_BOOST_MAXTON_THD                         ((uint32_t)(0xFUL << MXC_F_BOOST_MAXTON_THD_POS)) /**< MAXTON_THD Mask */

/**@} end of group BOOST_MAXTON_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_ILOAD BOOST_ILOAD
 * @brief    Boost Cycle Count Register.
 * @{
 */
#define MXC_F_BOOST_ILOAD_CNT_POS                      0 /**< ILOAD_CNT Position */
#define MXC_F_BOOST_ILOAD_CNT                          ((uint32_t)(0xFFUL << MXC_F_BOOST_ILOAD_CNT_POS)) /**< ILOAD_CNT Mask */

/**@} end of group BOOST_ILOAD_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_ALERT BOOST_ALERT
 * @brief    Boost Cycle Count Alert Register.
 * @{
 */
#define MXC_F_BOOST_ALERT_THD_POS                      0 /**< ALERT_THD Position */
#define MXC_F_BOOST_ALERT_THD                          ((uint32_t)(0xFFUL << MXC_F_BOOST_ALERT_THD_POS)) /**< ALERT_THD Mask */

/**@} end of group BOOST_ALERT_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_RDY BOOST_RDY
 * @brief    Boost Output Ready Register.
 * @{
 */
#define MXC_F_BOOST_RDY_OUT_POS                        0 /**< RDY_OUT Position */
#define MXC_F_BOOST_RDY_OUT                            ((uint32_t)(0x1UL << MXC_F_BOOST_RDY_OUT_POS)) /**< RDY_OUT Mask */

/**@} end of group BOOST_RDY_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_ZXCAL BOOST_ZXCAL
 * @brief    Zero Cross Calibration Register.
 * @{
 */
#define MXC_F_BOOST_ZXCAL_VAL_POS                      0 /**< ZXCAL_VAL Position */
#define MXC_F_BOOST_ZXCAL_VAL                          ((uint32_t)(0x1FUL << MXC_F_BOOST_ZXCAL_VAL_POS)) /**< ZXCAL_VAL Mask */

/**@} end of group BOOST_ZXCAL_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_INTEN BOOST_INTEN
 * @brief    Boost Alert Interrupt Enable Register.
 * @{
 */
#define MXC_F_BOOST_INTEN_ALERT_POS                    0 /**< INTEN_ALERT Position */
#define MXC_F_BOOST_INTEN_ALERT                        ((uint32_t)(0x1UL << MXC_F_BOOST_INTEN_ALERT_POS)) /**< INTEN_ALERT Mask */

/**@} end of group BOOST_INTEN_Register */

/**
 * @ingroup  boost_registers
 * @defgroup BOOST_INTFL BOOST_INTFL
 * @brief    Boost Alert Interrupt Status Register.
 * @{
 */
#define MXC_F_BOOST_INTFL_ALERT_POS                    0 /**< INTFL_ALERT Position */
#define MXC_F_BOOST_INTFL_ALERT                        ((uint32_t)(0x1UL << MXC_F_BOOST_INTFL_ALERT_POS)) /**< INTFL_ALERT Mask */

/**@} end of group BOOST_INTFL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_BOOST_REGS_H_
