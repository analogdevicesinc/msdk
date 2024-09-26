/**
 * @file    rtc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the RTC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup rtc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_RTC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_RTC_REGS_H_

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
 * @ingroup     rtc
 * @defgroup    rtc_registers RTC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the RTC Peripheral Module.
 * @details     Real Time Clock and Alarm.
 */

/**
 * @ingroup rtc_registers
 * Structure type to access the RTC Registers.
 */
typedef struct {
    __IO uint32_t sec;                  /**< <tt>\b 0x00:</tt> RTC SEC Register */
    __IO uint32_t ssec;                 /**< <tt>\b 0x04:</tt> RTC SSEC Register */
    __IO uint32_t toda;                 /**< <tt>\b 0x08:</tt> RTC TODA Register */
    __IO uint32_t sseca;                /**< <tt>\b 0x0C:</tt> RTC SSECA Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> RTC CTRL Register */
    __R  uint32_t rsv_0x14;
    __IO uint32_t oscctrl;              /**< <tt>\b 0x18:</tt> RTC OSCCTRL Register */
} mxc_rtc_regs_t;

/* Register offsets for module RTC */
/**
 * @ingroup    rtc_registers
 * @defgroup   RTC_Register_Offsets Register Offsets
 * @brief      RTC Peripheral Register Offsets from the RTC Base Peripheral Address.
 * @{
 */
#define MXC_R_RTC_SEC                      ((uint32_t)0x00000000UL) /**< Offset from RTC Base Address: <tt> 0x0000</tt> */
#define MXC_R_RTC_SSEC                     ((uint32_t)0x00000004UL) /**< Offset from RTC Base Address: <tt> 0x0004</tt> */
#define MXC_R_RTC_TODA                     ((uint32_t)0x00000008UL) /**< Offset from RTC Base Address: <tt> 0x0008</tt> */
#define MXC_R_RTC_SSECA                    ((uint32_t)0x0000000CUL) /**< Offset from RTC Base Address: <tt> 0x000C</tt> */
#define MXC_R_RTC_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from RTC Base Address: <tt> 0x0010</tt> */
#define MXC_R_RTC_OSCCTRL                  ((uint32_t)0x00000018UL) /**< Offset from RTC Base Address: <tt> 0x0018</tt> */
/**@} end of group rtc_registers */

/**
 * @ingroup  rtc_registers
 * @defgroup RTC_SEC RTC_SEC
 * @brief    RTC Second Counter. This register contains the 32-bit second counter.
 * @{
 */
#define MXC_F_RTC_SEC_SEC_POS                          0 /**< SEC_SEC Position */
#define MXC_F_RTC_SEC_SEC                              ((uint32_t)(0xFFFFFFFFUL << MXC_F_RTC_SEC_SEC_POS)) /**< SEC_SEC Mask */

/**@} end of group RTC_SEC_Register */

/**
 * @ingroup  rtc_registers
 * @defgroup RTC_SSEC RTC_SSEC
 * @brief    RTC Sub-second Counter. This counter increments at 256Hz. RTC_SEC is incremented
 *           when this register rolls over from 0xFF to 0x00.
 * @{
 */
#define MXC_F_RTC_SSEC_SSEC_POS                        0 /**< SSEC_SSEC Position */
#define MXC_F_RTC_SSEC_SSEC                            ((uint32_t)(0xFFUL << MXC_F_RTC_SSEC_SSEC_POS)) /**< SSEC_SSEC Mask */

/**@} end of group RTC_SSEC_Register */

/**
 * @ingroup  rtc_registers
 * @defgroup RTC_TODA RTC_TODA
 * @brief    Time-of-day Alarm.
 * @{
 */
#define MXC_F_RTC_TODA_TOD_ALARM_POS                   0 /**< TODA_TOD_ALARM Position */
#define MXC_F_RTC_TODA_TOD_ALARM                       ((uint32_t)(0xFFFFFUL << MXC_F_RTC_TODA_TOD_ALARM_POS)) /**< TODA_TOD_ALARM Mask */

/**@} end of group RTC_TODA_Register */

/**
 * @ingroup  rtc_registers
 * @defgroup RTC_SSECA RTC_SSECA
 * @brief    RTC sub-second alarm.  This register contains the reload value for the sub-
 *           second alarm.
 * @{
 */
#define MXC_F_RTC_SSECA_SSEC_ALARM_POS                 0 /**< SSECA_SSEC_ALARM Position */
#define MXC_F_RTC_SSECA_SSEC_ALARM                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_RTC_SSECA_SSEC_ALARM_POS)) /**< SSECA_SSEC_ALARM Mask */

/**@} end of group RTC_SSECA_Register */

/**
 * @ingroup  rtc_registers
 * @defgroup RTC_CTRL RTC_CTRL
 * @brief    RTC Control Register.
 * @{
 */
#define MXC_F_RTC_CTRL_ENABLE_POS                      0 /**< CTRL_ENABLE Position */
#define MXC_F_RTC_CTRL_ENABLE                          ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_ENABLE_POS)) /**< CTRL_ENABLE Mask */
#define MXC_V_RTC_CTRL_ENABLE_DIS                      ((uint32_t)0x0UL) /**< CTRL_ENABLE_DIS Value */
#define MXC_S_RTC_CTRL_ENABLE_DIS                      (MXC_V_RTC_CTRL_ENABLE_DIS << MXC_F_RTC_CTRL_ENABLE_POS) /**< CTRL_ENABLE_DIS Setting */
#define MXC_V_RTC_CTRL_ENABLE_EN                       ((uint32_t)0x1UL) /**< CTRL_ENABLE_EN Value */
#define MXC_S_RTC_CTRL_ENABLE_EN                       (MXC_V_RTC_CTRL_ENABLE_EN << MXC_F_RTC_CTRL_ENABLE_POS) /**< CTRL_ENABLE_EN Setting */

#define MXC_F_RTC_CTRL_TOD_ALARM_EN_POS                1 /**< CTRL_TOD_ALARM_EN Position */
#define MXC_F_RTC_CTRL_TOD_ALARM_EN                    ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_TOD_ALARM_EN_POS)) /**< CTRL_TOD_ALARM_EN Mask */
#define MXC_V_RTC_CTRL_TOD_ALARM_EN_DIS                ((uint32_t)0x0UL) /**< CTRL_TOD_ALARM_EN_DIS Value */
#define MXC_S_RTC_CTRL_TOD_ALARM_EN_DIS                (MXC_V_RTC_CTRL_TOD_ALARM_EN_DIS << MXC_F_RTC_CTRL_TOD_ALARM_EN_POS) /**< CTRL_TOD_ALARM_EN_DIS Setting */
#define MXC_V_RTC_CTRL_TOD_ALARM_EN_EN                 ((uint32_t)0x1UL) /**< CTRL_TOD_ALARM_EN_EN Value */
#define MXC_S_RTC_CTRL_TOD_ALARM_EN_EN                 (MXC_V_RTC_CTRL_TOD_ALARM_EN_EN << MXC_F_RTC_CTRL_TOD_ALARM_EN_POS) /**< CTRL_TOD_ALARM_EN_EN Setting */

#define MXC_F_RTC_CTRL_SSEC_ALARM_EN_POS               2 /**< CTRL_SSEC_ALARM_EN Position */
#define MXC_F_RTC_CTRL_SSEC_ALARM_EN                   ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_SSEC_ALARM_EN_POS)) /**< CTRL_SSEC_ALARM_EN Mask */
#define MXC_V_RTC_CTRL_SSEC_ALARM_EN_DIS               ((uint32_t)0x0UL) /**< CTRL_SSEC_ALARM_EN_DIS Value */
#define MXC_S_RTC_CTRL_SSEC_ALARM_EN_DIS               (MXC_V_RTC_CTRL_SSEC_ALARM_EN_DIS << MXC_F_RTC_CTRL_SSEC_ALARM_EN_POS) /**< CTRL_SSEC_ALARM_EN_DIS Setting */
#define MXC_V_RTC_CTRL_SSEC_ALARM_EN_EN                ((uint32_t)0x1UL) /**< CTRL_SSEC_ALARM_EN_EN Value */
#define MXC_S_RTC_CTRL_SSEC_ALARM_EN_EN                (MXC_V_RTC_CTRL_SSEC_ALARM_EN_EN << MXC_F_RTC_CTRL_SSEC_ALARM_EN_POS) /**< CTRL_SSEC_ALARM_EN_EN Setting */

#define MXC_F_RTC_CTRL_BUSY_POS                        3 /**< CTRL_BUSY Position */
#define MXC_F_RTC_CTRL_BUSY                            ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_BUSY_POS)) /**< CTRL_BUSY Mask */
#define MXC_V_RTC_CTRL_BUSY_IDLE                       ((uint32_t)0x0UL) /**< CTRL_BUSY_IDLE Value */
#define MXC_S_RTC_CTRL_BUSY_IDLE                       (MXC_V_RTC_CTRL_BUSY_IDLE << MXC_F_RTC_CTRL_BUSY_POS) /**< CTRL_BUSY_IDLE Setting */
#define MXC_V_RTC_CTRL_BUSY_BUSY                       ((uint32_t)0x1UL) /**< CTRL_BUSY_BUSY Value */
#define MXC_S_RTC_CTRL_BUSY_BUSY                       (MXC_V_RTC_CTRL_BUSY_BUSY << MXC_F_RTC_CTRL_BUSY_POS) /**< CTRL_BUSY_BUSY Setting */

#define MXC_F_RTC_CTRL_READY_POS                       4 /**< CTRL_READY Position */
#define MXC_F_RTC_CTRL_READY                           ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_READY_POS)) /**< CTRL_READY Mask */
#define MXC_V_RTC_CTRL_READY_NOT_READY                 ((uint32_t)0x0UL) /**< CTRL_READY_NOT_READY Value */
#define MXC_S_RTC_CTRL_READY_NOT_READY                 (MXC_V_RTC_CTRL_READY_NOT_READY << MXC_F_RTC_CTRL_READY_POS) /**< CTRL_READY_NOT_READY Setting */
#define MXC_V_RTC_CTRL_READY_READY                     ((uint32_t)0x1UL) /**< CTRL_READY_READY Value */
#define MXC_S_RTC_CTRL_READY_READY                     (MXC_V_RTC_CTRL_READY_READY << MXC_F_RTC_CTRL_READY_POS) /**< CTRL_READY_READY Setting */

#define MXC_F_RTC_CTRL_READY_INT_EN_POS                5 /**< CTRL_READY_INT_EN Position */
#define MXC_F_RTC_CTRL_READY_INT_EN                    ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_READY_INT_EN_POS)) /**< CTRL_READY_INT_EN Mask */
#define MXC_V_RTC_CTRL_READY_INT_EN_DIS                ((uint32_t)0x0UL) /**< CTRL_READY_INT_EN_DIS Value */
#define MXC_S_RTC_CTRL_READY_INT_EN_DIS                (MXC_V_RTC_CTRL_READY_INT_EN_DIS << MXC_F_RTC_CTRL_READY_INT_EN_POS) /**< CTRL_READY_INT_EN_DIS Setting */
#define MXC_V_RTC_CTRL_READY_INT_EN_EN                 ((uint32_t)0x1UL) /**< CTRL_READY_INT_EN_EN Value */
#define MXC_S_RTC_CTRL_READY_INT_EN_EN                 (MXC_V_RTC_CTRL_READY_INT_EN_EN << MXC_F_RTC_CTRL_READY_INT_EN_POS) /**< CTRL_READY_INT_EN_EN Setting */

#define MXC_F_RTC_CTRL_TOD_ALARM_FL_POS                6 /**< CTRL_TOD_ALARM_FL Position */
#define MXC_F_RTC_CTRL_TOD_ALARM_FL                    ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_TOD_ALARM_FL_POS)) /**< CTRL_TOD_ALARM_FL Mask */
#define MXC_V_RTC_CTRL_TOD_ALARM_FL_INACTIVE           ((uint32_t)0x0UL) /**< CTRL_TOD_ALARM_FL_INACTIVE Value */
#define MXC_S_RTC_CTRL_TOD_ALARM_FL_INACTIVE           (MXC_V_RTC_CTRL_TOD_ALARM_FL_INACTIVE << MXC_F_RTC_CTRL_TOD_ALARM_FL_POS) /**< CTRL_TOD_ALARM_FL_INACTIVE Setting */
#define MXC_V_RTC_CTRL_TOD_ALARM_FL_PENDING            ((uint32_t)0x1UL) /**< CTRL_TOD_ALARM_FL_PENDING Value */
#define MXC_S_RTC_CTRL_TOD_ALARM_FL_PENDING            (MXC_V_RTC_CTRL_TOD_ALARM_FL_PENDING << MXC_F_RTC_CTRL_TOD_ALARM_FL_POS) /**< CTRL_TOD_ALARM_FL_PENDING Setting */

#define MXC_F_RTC_CTRL_SSEC_ALARM_FL_POS               7 /**< CTRL_SSEC_ALARM_FL Position */
#define MXC_F_RTC_CTRL_SSEC_ALARM_FL                   ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_SSEC_ALARM_FL_POS)) /**< CTRL_SSEC_ALARM_FL Mask */
#define MXC_V_RTC_CTRL_SSEC_ALARM_FL_INACTIVE          ((uint32_t)0x0UL) /**< CTRL_SSEC_ALARM_FL_INACTIVE Value */
#define MXC_S_RTC_CTRL_SSEC_ALARM_FL_INACTIVE          (MXC_V_RTC_CTRL_SSEC_ALARM_FL_INACTIVE << MXC_F_RTC_CTRL_SSEC_ALARM_FL_POS) /**< CTRL_SSEC_ALARM_FL_INACTIVE Setting */
#define MXC_V_RTC_CTRL_SSEC_ALARM_FL_PENDING           ((uint32_t)0x1UL) /**< CTRL_SSEC_ALARM_FL_PENDING Value */
#define MXC_S_RTC_CTRL_SSEC_ALARM_FL_PENDING           (MXC_V_RTC_CTRL_SSEC_ALARM_FL_PENDING << MXC_F_RTC_CTRL_SSEC_ALARM_FL_POS) /**< CTRL_SSEC_ALARM_FL_PENDING Setting */

#define MXC_F_RTC_CTRL_SQWOUT_EN_POS                   8 /**< CTRL_SQWOUT_EN Position */
#define MXC_F_RTC_CTRL_SQWOUT_EN                       ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_SQWOUT_EN_POS)) /**< CTRL_SQWOUT_EN Mask */
#define MXC_V_RTC_CTRL_SQWOUT_EN_DIS                   ((uint32_t)0x0UL) /**< CTRL_SQWOUT_EN_DIS Value */
#define MXC_S_RTC_CTRL_SQWOUT_EN_DIS                   (MXC_V_RTC_CTRL_SQWOUT_EN_DIS << MXC_F_RTC_CTRL_SQWOUT_EN_POS) /**< CTRL_SQWOUT_EN_DIS Setting */
#define MXC_V_RTC_CTRL_SQWOUT_EN_EN                    ((uint32_t)0x1UL) /**< CTRL_SQWOUT_EN_EN Value */
#define MXC_S_RTC_CTRL_SQWOUT_EN_EN                    (MXC_V_RTC_CTRL_SQWOUT_EN_EN << MXC_F_RTC_CTRL_SQWOUT_EN_POS) /**< CTRL_SQWOUT_EN_EN Setting */

#define MXC_F_RTC_CTRL_FREQ_SEL_POS                    9 /**< CTRL_FREQ_SEL Position */
#define MXC_F_RTC_CTRL_FREQ_SEL                        ((uint32_t)(0x3UL << MXC_F_RTC_CTRL_FREQ_SEL_POS)) /**< CTRL_FREQ_SEL Mask */
#define MXC_V_RTC_CTRL_FREQ_SEL_FREQ1HZ                ((uint32_t)0x0UL) /**< CTRL_FREQ_SEL_FREQ1HZ Value */
#define MXC_S_RTC_CTRL_FREQ_SEL_FREQ1HZ                (MXC_V_RTC_CTRL_FREQ_SEL_FREQ1HZ << MXC_F_RTC_CTRL_FREQ_SEL_POS) /**< CTRL_FREQ_SEL_FREQ1HZ Setting */
#define MXC_V_RTC_CTRL_FREQ_SEL_FREQ512HZ              ((uint32_t)0x1UL) /**< CTRL_FREQ_SEL_FREQ512HZ Value */
#define MXC_S_RTC_CTRL_FREQ_SEL_FREQ512HZ              (MXC_V_RTC_CTRL_FREQ_SEL_FREQ512HZ << MXC_F_RTC_CTRL_FREQ_SEL_POS) /**< CTRL_FREQ_SEL_FREQ512HZ Setting */
#define MXC_V_RTC_CTRL_FREQ_SEL_FREQ4KHZ               ((uint32_t)0x2UL) /**< CTRL_FREQ_SEL_FREQ4KHZ Value */
#define MXC_S_RTC_CTRL_FREQ_SEL_FREQ4KHZ               (MXC_V_RTC_CTRL_FREQ_SEL_FREQ4KHZ << MXC_F_RTC_CTRL_FREQ_SEL_POS) /**< CTRL_FREQ_SEL_FREQ4KHZ Setting */

#define MXC_F_RTC_CTRL_ACRE_POS                        14 /**< CTRL_ACRE Position */
#define MXC_F_RTC_CTRL_ACRE                            ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_ACRE_POS)) /**< CTRL_ACRE Mask */
#define MXC_V_RTC_CTRL_ACRE_SYNC                       ((uint32_t)0x0UL) /**< CTRL_ACRE_SYNC Value */
#define MXC_S_RTC_CTRL_ACRE_SYNC                       (MXC_V_RTC_CTRL_ACRE_SYNC << MXC_F_RTC_CTRL_ACRE_POS) /**< CTRL_ACRE_SYNC Setting */
#define MXC_V_RTC_CTRL_ACRE_ASYNC                      ((uint32_t)0x1UL) /**< CTRL_ACRE_ASYNC Value */
#define MXC_S_RTC_CTRL_ACRE_ASYNC                      (MXC_V_RTC_CTRL_ACRE_ASYNC << MXC_F_RTC_CTRL_ACRE_POS) /**< CTRL_ACRE_ASYNC Setting */

#define MXC_F_RTC_CTRL_WRITE_EN_POS                    15 /**< CTRL_WRITE_EN Position */
#define MXC_F_RTC_CTRL_WRITE_EN                        ((uint32_t)(0x1UL << MXC_F_RTC_CTRL_WRITE_EN_POS)) /**< CTRL_WRITE_EN Mask */
#define MXC_V_RTC_CTRL_WRITE_EN_DIS                    ((uint32_t)0x0UL) /**< CTRL_WRITE_EN_DIS Value */
#define MXC_S_RTC_CTRL_WRITE_EN_DIS                    (MXC_V_RTC_CTRL_WRITE_EN_DIS << MXC_F_RTC_CTRL_WRITE_EN_POS) /**< CTRL_WRITE_EN_DIS Setting */
#define MXC_V_RTC_CTRL_WRITE_EN_EN                     ((uint32_t)0x1UL) /**< CTRL_WRITE_EN_EN Value */
#define MXC_S_RTC_CTRL_WRITE_EN_EN                     (MXC_V_RTC_CTRL_WRITE_EN_EN << MXC_F_RTC_CTRL_WRITE_EN_POS) /**< CTRL_WRITE_EN_EN Setting */

/**@} end of group RTC_CTRL_Register */

/**
 * @ingroup  rtc_registers
 * @defgroup RTC_OSCCTRL RTC_OSCCTRL
 * @brief    RTC Oscillator Control Register.
 * @{
 */
#define MXC_F_RTC_OSCCTRL_FILTER_EN_POS                0 /**< OSCCTRL_FILTER_EN Position */
#define MXC_F_RTC_OSCCTRL_FILTER_EN                    ((uint32_t)(0x1UL << MXC_F_RTC_OSCCTRL_FILTER_EN_POS)) /**< OSCCTRL_FILTER_EN Mask */

#define MXC_F_RTC_OSCCTRL_IBIAS_SEL_POS                1 /**< OSCCTRL_IBIAS_SEL Position */
#define MXC_F_RTC_OSCCTRL_IBIAS_SEL                    ((uint32_t)(0x1UL << MXC_F_RTC_OSCCTRL_IBIAS_SEL_POS)) /**< OSCCTRL_IBIAS_SEL Mask */

#define MXC_F_RTC_OSCCTRL_HYST_EN_POS                  2 /**< OSCCTRL_HYST_EN Position */
#define MXC_F_RTC_OSCCTRL_HYST_EN                      ((uint32_t)(0x1UL << MXC_F_RTC_OSCCTRL_HYST_EN_POS)) /**< OSCCTRL_HYST_EN Mask */

#define MXC_F_RTC_OSCCTRL_IBIAS_EN_POS                 3 /**< OSCCTRL_IBIAS_EN Position */
#define MXC_F_RTC_OSCCTRL_IBIAS_EN                     ((uint32_t)(0x1UL << MXC_F_RTC_OSCCTRL_IBIAS_EN_POS)) /**< OSCCTRL_IBIAS_EN Mask */

#define MXC_F_RTC_OSCCTRL_BYPASS_POS                   4 /**< OSCCTRL_BYPASS Position */
#define MXC_F_RTC_OSCCTRL_BYPASS                       ((uint32_t)(0x1UL << MXC_F_RTC_OSCCTRL_BYPASS_POS)) /**< OSCCTRL_BYPASS Mask */

#define MXC_F_RTC_OSCCTRL_32KOUT_POS                   5 /**< OSCCTRL_32KOUT Position */
#define MXC_F_RTC_OSCCTRL_32KOUT                       ((uint32_t)(0x1UL << MXC_F_RTC_OSCCTRL_32KOUT_POS)) /**< OSCCTRL_32KOUT Mask */

/**@} end of group RTC_OSCCTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_RTC_REGS_H_
