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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_MCR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_MCR_REGS_H_

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
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t pdown;                /**< <tt>\b 0x08:</tt> MCR PDOWN Register */
    __R  uint32_t rsv_0xc;
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> MCR CTRL Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x14:</tt> MCR CLKCTRL Register */
    __IO uint32_t rst;                  /**< <tt>\b 0x18:</tt> MCR RST Register */
    __IO uint32_t rtctrim;              /**< <tt>\b 0x1C:</tt> MCR RTCTRIM Register */
    __R  uint32_t rsv_0x20_0x5f[16];
    __IO uint32_t ldoctrl;              /**< <tt>\b 0x60:</tt> MCR LDOCTRL Register */
    __IO uint32_t pwrmonst;             /**< <tt>\b 0x64:</tt> MCR PWRMONST Register */
} mxc_mcr_regs_t;

/* Register offsets for module MCR */
/**
 * @ingroup    mcr_registers
 * @defgroup   MCR_Register_Offsets Register Offsets
 * @brief      MCR Peripheral Register Offsets from the MCR Base Peripheral Address.
 * @{
 */
#define MXC_R_MCR_PDOWN                    ((uint32_t)0x00000008UL) /**< Offset from MCR Base Address: <tt> 0x0008</tt> */
#define MXC_R_MCR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from MCR Base Address: <tt> 0x0010</tt> */
#define MXC_R_MCR_CLKCTRL                  ((uint32_t)0x00000014UL) /**< Offset from MCR Base Address: <tt> 0x0014</tt> */
#define MXC_R_MCR_RST                      ((uint32_t)0x00000018UL) /**< Offset from MCR Base Address: <tt> 0x0018</tt> */
#define MXC_R_MCR_RTCTRIM                  ((uint32_t)0x0000001CUL) /**< Offset from MCR Base Address: <tt> 0x001C</tt> */
#define MXC_R_MCR_LDOCTRL                  ((uint32_t)0x00000060UL) /**< Offset from MCR Base Address: <tt> 0x0060</tt> */
#define MXC_R_MCR_PWRMONST                 ((uint32_t)0x00000064UL) /**< Offset from MCR Base Address: <tt> 0x0064</tt> */
/**@} end of group mcr_registers */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PDOWN MCR_PDOWN
 * @brief    PDOWN Drive Strength
 * @{
 */
#define MXC_F_MCR_PDOWN_PDOWNDS_POS                    0 /**< PDOWN_PDOWNDS Position */
#define MXC_F_MCR_PDOWN_PDOWNDS                        ((uint32_t)(0x3UL << MXC_F_MCR_PDOWN_PDOWNDS_POS)) /**< PDOWN_PDOWNDS Mask */

#define MXC_F_MCR_PDOWN_PDOWNVS_POS                    2 /**< PDOWN_PDOWNVS Position */
#define MXC_F_MCR_PDOWN_PDOWNVS                        ((uint32_t)(0x1UL << MXC_F_MCR_PDOWN_PDOWNVS_POS)) /**< PDOWN_PDOWNVS Mask */

/**@} end of group MCR_PDOWN_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CTRL MCR_CTRL
 * @brief    Misc Power State Control Register
 * @{
 */
#define MXC_F_MCR_CTRL_VDDCSW_POS                      1 /**< CTRL_VDDCSW Position */
#define MXC_F_MCR_CTRL_VDDCSW                          ((uint32_t)(0x3UL << MXC_F_MCR_CTRL_VDDCSW_POS)) /**< CTRL_VDDCSW Mask */

#define MXC_F_MCR_CTRL_USBSWEN_N_POS                   3 /**< CTRL_USBSWEN_N Position */
#define MXC_F_MCR_CTRL_USBSWEN_N                       ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_USBSWEN_N_POS)) /**< CTRL_USBSWEN_N Mask */

#define MXC_F_MCR_CTRL_P1M_POS                         9 /**< CTRL_P1M Position */
#define MXC_F_MCR_CTRL_P1M                             ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_P1M_POS)) /**< CTRL_P1M Mask */

#define MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL_POS            10 /**< CTRL_RSTN_VOLTAGE_SEL Position */
#define MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL                ((uint32_t)(0x1UL << MXC_F_MCR_CTRL_RSTN_VOLTAGE_SEL_POS)) /**< CTRL_RSTN_VOLTAGE_SEL Mask */

/**@} end of group MCR_CTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_CLKCTRL MCR_CLKCTRL
 * @brief    Clock Control Register.
 * @{
 */
#define MXC_F_MCR_CLKCTRL_ERTCO_PD_POS                 16 /**< CLKCTRL_ERTCO_PD Position */
#define MXC_F_MCR_CLKCTRL_ERTCO_PD                     ((uint32_t)(0x1UL << MXC_F_MCR_CLKCTRL_ERTCO_PD_POS)) /**< CLKCTRL_ERTCO_PD Mask */

#define MXC_F_MCR_CLKCTRL_ERTCO_EN_POS                 17 /**< CLKCTRL_ERTCO_EN Position */
#define MXC_F_MCR_CLKCTRL_ERTCO_EN                     ((uint32_t)(0x1UL << MXC_F_MCR_CLKCTRL_ERTCO_EN_POS)) /**< CLKCTRL_ERTCO_EN Mask */

/**@} end of group MCR_CLKCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RST MCR_RST
 * @brief    Reset Register.
 * @{
 */
#define MXC_F_MCR_RST_RTC_POS                          0 /**< RST_RTC Position */
#define MXC_F_MCR_RST_RTC                              ((uint32_t)(0x1UL << MXC_F_MCR_RST_RTC_POS)) /**< RST_RTC Mask */

/**@} end of group MCR_RST_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_RTCTRIM MCR_RTCTRIM
 * @brief    RTC Trim Register.
 * @{
 */
#define MXC_F_MCR_RTCTRIM_TRIM_X1_POS                  0 /**< RTCTRIM_TRIM_X1 Position */
#define MXC_F_MCR_RTCTRIM_TRIM_X1                      ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_TRIM_X1_POS)) /**< RTCTRIM_TRIM_X1 Mask */

#define MXC_F_MCR_RTCTRIM_TRIM_X2_POS                  8 /**< RTCTRIM_TRIM_X2 Position */
#define MXC_F_MCR_RTCTRIM_TRIM_X2                      ((uint32_t)(0x1FUL << MXC_F_MCR_RTCTRIM_TRIM_X2_POS)) /**< RTCTRIM_TRIM_X2 Mask */

/**@} end of group MCR_RTCTRIM_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_LDOCTRL MCR_LDOCTRL
 * @brief    LDO Control Register.
 * @{
 */
#define MXC_F_MCR_LDOCTRL_0P9V_EN_POS                  0 /**< LDOCTRL_0P9V_EN Position */
#define MXC_F_MCR_LDOCTRL_0P9V_EN                      ((uint32_t)(0x1UL << MXC_F_MCR_LDOCTRL_0P9V_EN_POS)) /**< LDOCTRL_0P9V_EN Mask */

/**@} end of group MCR_LDOCTRL_Register */

/**
 * @ingroup  mcr_registers
 * @defgroup MCR_PWRMONST MCR_PWRMONST
 * @brief    Power Monitor Statuses Register.
 * @{
 */
#define MXC_F_MCR_PWRMONST_PORZ_VLOSS_POS              0 /**< PWRMONST_PORZ_VLOSS Position */
#define MXC_F_MCR_PWRMONST_PORZ_VLOSS                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VLOSS_POS)) /**< PWRMONST_PORZ_VLOSS Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VBAT_POS               1 /**< PWRMONST_PORZ_VBAT Position */
#define MXC_F_MCR_PWRMONST_PORZ_VBAT                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VBAT_POS)) /**< PWRMONST_PORZ_VBAT Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VRTC_POS               2 /**< PWRMONST_PORZ_VRTC Position */
#define MXC_F_MCR_PWRMONST_PORZ_VRTC                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VRTC_POS)) /**< PWRMONST_PORZ_VRTC Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDC_POS               5 /**< PWRMONST_PORZ_VDDC Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDC                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDC_POS)) /**< PWRMONST_PORZ_VDDC Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDA_POS               6 /**< PWRMONST_PORZ_VDDA Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDA                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDA_POS)) /**< PWRMONST_PORZ_VDDA Mask */

#define MXC_F_MCR_PWRMONST_PORZ_VDDB_POS               7 /**< PWRMONST_PORZ_VDDB Position */
#define MXC_F_MCR_PWRMONST_PORZ_VDDB                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_PORZ_VDDB_POS)) /**< PWRMONST_PORZ_VDDB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDC_POS               9 /**< PWRMONST_RSTZ_VDDC Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDC                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDC_POS)) /**< PWRMONST_RSTZ_VDDC Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDA_POS               10 /**< PWRMONST_RSTZ_VDDA Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDA                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDA_POS)) /**< PWRMONST_RSTZ_VDDA Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDB_POS               11 /**< PWRMONST_RSTZ_VDDB Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDB                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDB_POS)) /**< PWRMONST_RSTZ_VDDB Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIO_POS              12 /**< PWRMONST_RSTZ_VDDIO Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIO                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIO_POS)) /**< PWRMONST_RSTZ_VDDIO Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOH_POS             13 /**< PWRMONST_RSTZ_VDDIOH Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOH                 ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOH_POS)) /**< PWRMONST_RSTZ_VDDIOH Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VRTC_POS               14 /**< PWRMONST_RSTZ_VRTC Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VRTC                   ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VRTC_POS)) /**< PWRMONST_RSTZ_VRTC Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_LDO_0P9V_POS           16 /**< PWRMONST_RSTZ_LDO_0P9V Position */
#define MXC_F_MCR_PWRMONST_RSTZ_LDO_0P9V               ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_LDO_0P9V_POS)) /**< PWRMONST_RSTZ_LDO_0P9V Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDCA_POS              17 /**< PWRMONST_RSTZ_VDDCA Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDCA                  ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDCA_POS)) /**< PWRMONST_RSTZ_VDDCA Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VCOREHV_POS            18 /**< PWRMONST_RSTZ_VCOREHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VCOREHV                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VCOREHV_POS)) /**< PWRMONST_RSTZ_VCOREHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV_POS            19 /**< PWRMONST_RSTZ_VDDIOHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV                ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOHV_POS)) /**< PWRMONST_RSTZ_VDDIOHV Mask */

#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV_POS           20 /**< PWRMONST_RSTZ_VDDIOHHV Position */
#define MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV               ((uint32_t)(0x1UL << MXC_F_MCR_PWRMONST_RSTZ_VDDIOHHV_POS)) /**< PWRMONST_RSTZ_VDDIOHHV Mask */

/**@} end of group MCR_PWRMONST_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_MCR_REGS_H_
