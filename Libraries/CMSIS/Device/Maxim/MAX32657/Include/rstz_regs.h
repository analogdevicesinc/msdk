/**
 * @file    rstz_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the RSTZ Peripheral Module.
 * @note    This file is @generated.
 * @ingroup rstz_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_RSTZ_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_RSTZ_REGS_H_

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
 * @ingroup     rstz
 * @defgroup    rstz_registers RSTZ_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the RSTZ Peripheral Module.
 * @details     RSTZ Controller
 */

/**
 * @ingroup rstz_registers
 * Structure type to access the RSTZ Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x000:</tt> RSTZ CTRL Register */
    __IO uint32_t boost_clkctrl;        /**< <tt>\b 0x004:</tt> RSTZ BOOST_CLKCTRL Register */
    __R  uint32_t rsv_0x8_0x27[8];
    __IO uint32_t status_ch[8];         /**< <tt>\b 0x028:</tt> RSTZ STATUS_CH Register */
} mxc_rstz_regs_t;

/* Register offsets for module RSTZ */
/**
 * @ingroup    rstz_registers
 * @defgroup   RSTZ_Register_Offsets Register Offsets
 * @brief      RSTZ Peripheral Register Offsets from the RSTZ Base Peripheral Address.
 * @{
 */
#define MXC_R_RSTZ_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from RSTZ Base Address: <tt> 0x0000</tt> */
#define MXC_R_RSTZ_BOOST_CLKCTRL           ((uint32_t)0x00000004UL) /**< Offset from RSTZ Base Address: <tt> 0x0004</tt> */
#define MXC_R_RSTZ_STATUS_CH               ((uint32_t)0x00000028UL) /**< Offset from RSTZ Base Address: <tt> 0x0028</tt> */
/**@} end of group rstz_registers */

/**
 * @ingroup  rstz_registers
 * @defgroup RSTZ_CTRL RSTZ_CTRL
 * @brief    RSTZ Control Register.
 * @{
 */
#define MXC_F_RSTZ_CTRL_EN_POS                         0 /**< CTRL_EN Position */
#define MXC_F_RSTZ_CTRL_EN                             ((uint32_t)(0x1UL << MXC_F_RSTZ_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_RSTZ_CTRL_SVC_EN_POS                     1 /**< CTRL_SVC_EN Position */
#define MXC_F_RSTZ_CTRL_SVC_EN                         ((uint32_t)(0x1UL << MXC_F_RSTZ_CTRL_SVC_EN_POS)) /**< CTRL_SVC_EN Mask */

#define MXC_F_RSTZ_CTRL_CH_SEL_POS                     2 /**< CTRL_CH_SEL Position */
#define MXC_F_RSTZ_CTRL_CH_SEL                         ((uint32_t)(0x7UL << MXC_F_RSTZ_CTRL_CH_SEL_POS)) /**< CTRL_CH_SEL Mask */

#define MXC_F_RSTZ_CTRL_CAL_EN_POS                     5 /**< CTRL_CAL_EN Position */
#define MXC_F_RSTZ_CTRL_CAL_EN                         ((uint32_t)(0x1UL << MXC_F_RSTZ_CTRL_CAL_EN_POS)) /**< CTRL_CAL_EN Mask */

#define MXC_F_RSTZ_CTRL_DMEASURE_EN_POS                6 /**< CTRL_DMEASURE_EN Position */
#define MXC_F_RSTZ_CTRL_DMEASURE_EN                    ((uint32_t)(0x1UL << MXC_F_RSTZ_CTRL_DMEASURE_EN_POS)) /**< CTRL_DMEASURE_EN Mask */

#define MXC_F_RSTZ_CTRL_OFFTR_P_POS                    7 /**< CTRL_OFFTR_P Position */
#define MXC_F_RSTZ_CTRL_OFFTR_P                        ((uint32_t)(0x1FUL << MXC_F_RSTZ_CTRL_OFFTR_P_POS)) /**< CTRL_OFFTR_P Mask */

#define MXC_F_RSTZ_CTRL_OFFTR_N_POS                    12 /**< CTRL_OFFTR_N Position */
#define MXC_F_RSTZ_CTRL_OFFTR_N                        ((uint32_t)(0x1FUL << MXC_F_RSTZ_CTRL_OFFTR_N_POS)) /**< CTRL_OFFTR_N Mask */

#define MXC_F_RSTZ_CTRL_DOUT_POS                       17 /**< CTRL_DOUT Position */
#define MXC_F_RSTZ_CTRL_DOUT                           ((uint32_t)(0x1UL << MXC_F_RSTZ_CTRL_DOUT_POS)) /**< CTRL_DOUT Mask */

#define MXC_F_RSTZ_CTRL_CAL_DOUT_POL_POS               18 /**< CTRL_CAL_DOUT_POL Position */
#define MXC_F_RSTZ_CTRL_CAL_DOUT_POL                   ((uint32_t)(0x1UL << MXC_F_RSTZ_CTRL_CAL_DOUT_POL_POS)) /**< CTRL_CAL_DOUT_POL Mask */

#define MXC_F_RSTZ_CTRL_NUM_SAMP_POS                   24 /**< CTRL_NUM_SAMP Position */
#define MXC_F_RSTZ_CTRL_NUM_SAMP                       ((uint32_t)(0xFUL << MXC_F_RSTZ_CTRL_NUM_SAMP_POS)) /**< CTRL_NUM_SAMP Mask */

#define MXC_F_RSTZ_CTRL_TRIP_TOL_POS                   28 /**< CTRL_TRIP_TOL Position */
#define MXC_F_RSTZ_CTRL_TRIP_TOL                       ((uint32_t)(0xFUL << MXC_F_RSTZ_CTRL_TRIP_TOL_POS)) /**< CTRL_TRIP_TOL Mask */

/**@} end of group RSTZ_CTRL_Register */

/**
 * @ingroup  rstz_registers
 * @defgroup RSTZ_BOOST_CLKCTRL RSTZ_BOOST_CLKCTRL
 * @brief    Boost Clock Control Register.
 * @{
 */
#define MXC_F_RSTZ_BOOST_CLKCTRL_EXIT_NUM_SAMP_POS     0 /**< BOOST_CLKCTRL_EXIT_NUM_SAMP Position */
#define MXC_F_RSTZ_BOOST_CLKCTRL_EXIT_NUM_SAMP         ((uint32_t)(0x3UL << MXC_F_RSTZ_BOOST_CLKCTRL_EXIT_NUM_SAMP_POS)) /**< BOOST_CLKCTRL_EXIT_NUM_SAMP Mask */

#define MXC_F_RSTZ_BOOST_CLKCTRL_CH_SEL_POS            2 /**< BOOST_CLKCTRL_CH_SEL Position */
#define MXC_F_RSTZ_BOOST_CLKCTRL_CH_SEL                ((uint32_t)(0x7UL << MXC_F_RSTZ_BOOST_CLKCTRL_CH_SEL_POS)) /**< BOOST_CLKCTRL_CH_SEL Mask */

/**@} end of group RSTZ_BOOST_CLKCTRL_Register */

/**
 * @ingroup  rstz_registers
 * @defgroup RSTZ_STATUS_CH RSTZ_STATUS_CH
 * @brief    Channel X Status Register.
 * @{
 */
#define MXC_F_RSTZ_STATUS_CH_RSTZ_POS                  0 /**< STATUS_CH_RSTZ Position */
#define MXC_F_RSTZ_STATUS_CH_RSTZ                      ((uint32_t)(0x1UL << MXC_F_RSTZ_STATUS_CH_RSTZ_POS)) /**< STATUS_CH_RSTZ Mask */

#define MXC_F_RSTZ_STATUS_CH_DOUT_POS                  1 /**< STATUS_CH_DOUT Position */
#define MXC_F_RSTZ_STATUS_CH_DOUT                      ((uint32_t)(0x1UL << MXC_F_RSTZ_STATUS_CH_DOUT_POS)) /**< STATUS_CH_DOUT Mask */

#define MXC_F_RSTZ_STATUS_CH_RSTZ_FL_POS               31 /**< STATUS_CH_RSTZ_FL Position */
#define MXC_F_RSTZ_STATUS_CH_RSTZ_FL                   ((uint32_t)(0x1UL << MXC_F_RSTZ_STATUS_CH_RSTZ_FL_POS)) /**< STATUS_CH_RSTZ_FL Mask */

/**@} end of group RSTZ_STATUS_CH_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_RSTZ_REGS_H_
