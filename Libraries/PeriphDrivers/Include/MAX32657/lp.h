/**
 * @file    lp.h
 * @brief   Low Power(LP) function prototypes and data types.
 */

/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_LP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_LP_H_

/* **** Includes **** */
#include <stdint.h>
#include "pwrseq_regs.h"
#include "mcr_regs.h"
#include "gcr_regs.h"
#include "gpio.h"
#include "tmr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup pwrseq Low Power (LP)
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief   Enumeration type for voltage selection
 *
 */
typedef enum { MXC_LP_V0_9 = 0, MXC_LP_V1_0, MXC_LP_V1_1 } mxc_lp_ovr_t;

/**
 * @brief      Places the device into SLEEP mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterSleepMode(void);

/**
 * @brief      Places the device into Low Power mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterLowPowerMode(void);

/**
 * @brief      Places the device into Micro Power mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterMicroPowerMode(void);

/**
 * @brief      Places the device into Standby mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterStandbyMode(void);

/**
 * @brief      Places the device into BACKUP mode.  CPU state is not maintained in this mode, so this function never returns.
 *             Instead, the device will restart once an RTC or external interrupt occur.
 */
void MXC_LP_EnterBackupMode(void);

/**
 * @brief      Routine for when exiting Backup mode via warmboot.
 */
void MXC_LP_ExitBackupMode(void);

/**
 * @brief      Places the device into Shutdown mode.  CPU state is not maintained in this mode, so this function never returns.
 *             Instead, the device will restart once an RTC, USB wakeup, or external interrupt occur.
 */
void MXC_LP_EnterPowerDownMode(void);

/**
 * @brief      Set ovr bits to set the voltage the micro will run at.
 *
 * @param[in]  ovr   The ovr options are only 0.9V, 1.0V, and 1.1V use enum mxc_lp_ovr_t
 */
void MXC_LP_SetOVR(mxc_lp_ovr_t ovr);

/**
 * @brief      Enable retention regulator
 */
void MXC_LP_EnableRetentionReg(void);

/**
 * @brief      Disable retention regulator
 */
void MXC_LP_DisableRetentionReg(void);

/**
 * @brief      Is the retention regulator enabled
 *
 * @return     1 = enabled 0 =  disabled
 */
int MXC_LP_RetentionRegIsEnabled(void);

/**
 * @brief      Enables data retention to RAM blocks 0-4 in BACKUP mode.
 * 
 * @param[in]  mask  The mask of the RAM blocks to enable data retention. Bit 0 enables 
 *                   block 0, bit 1 enables block 1, etc.
 */
void MXC_LP_EnableSramRetention(uint32_t mask);

/**
 * @brief      Disables data retention to RAM blocks 0-4 in BACKUP mode.
 * 
 * @param[in]  mask  The mask of the RAM blocks to disable data retention. Bit 0 disables 
 *                   block 0, bit 1 disables block 1, etc.
 */
void MXC_LP_DisableSramRetention(uint32_t mask);

/**
 * @brief      Turn bandgap on
 */
void MXC_LP_BandgapOn(void);

/**
 * @brief      Turn bandgap off
 */
void MXC_LP_BandgapOff(void);

/**
 * @brief      Is the bandgap on or off
 *
 * @return     1 = bandgap on , 0 = bandgap off
 */
int MXC_LP_BandgapIsOn(void);

/**
 * @brief      clear all wake up status
 */
void MXC_LP_ClearWakeStatus(void);

/**
 * @brief      Enables the selected GPIO port and its selected pins to wake up the device from any low power mode.
 *             Call this function multiple times to enable pins on multiple ports.  This function does not configure
 *             the GPIO pins nor does it setup their interrupt functionality.
 * @param      wu_pins      The port and pins to configure as wakeup sources.  Only the gpio and mask fields of the
 *                          structure are used.  The func and pad fields are ignored.
 */

void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t *wu_pins);

/**
 * @brief      Disables the selected GPIO port and its selected pins as a wake up source.
 *             Call this function multiple times to disable pins on multiple ports.
 * @param      wu_pins      The port and pins to disable as wakeup sources.  Only the gpio and mask fields of the
 *                          structure are used.  The func and pad fields are ignored.
 */
void MXC_LP_DisableGPIOWakeup(mxc_gpio_cfg_t *wu_pins);

/**
 * @brief      Enables the RTC alarm to wake up the device from any low power mode.
 */
void MXC_LP_EnableRTCAlarmWakeup(void);

/**
 * @brief      Disables the RTC alarm from waking up the device.
 */
void MXC_LP_DisableRTCAlarmWakeup(void);

/**
 * @brief      Enables Timer to wakeup from any low power mode. 
 * 
 * @param      tmr  Pointer to timer module.
 */
void MXC_LP_EnableTimerWakeup(mxc_tmr_regs_t *tmr);

/**
 * @brief      Disables Timer from waking up device.  
 * 
 * @param      tmr  Pointer to timer module.
 */
void MXC_LP_DisableTimerWakeup(mxc_tmr_regs_t *tmr);

/**
 * @brief      Enables the WUT alarm to wake up the device from any low power mode.
 */
void MXC_LP_EnableWUTAlarmWakeup(void);

/**
 * @brief      Disables the WUT alarm from waking up the device.
 */
void MXC_LP_DisableWUTAlarmWakeup(void);

/**
 * @brief      Configure which clocks are powered down at deep sleep and which are not affected.
 *
 * @note       Need to configure all clocks at once any clock not passed in the mask will be 
 *             unaffected by Deepsleep.
 *
 * @param[in]  mask         The mask of the clocks to power down when part goes into deepsleep.
 *                          Only ERTCO can be switched off.
 *
 * @return     #E_NO_ERROR or error based on /ref MXC_Error_Codes
 */
int MXC_LP_ConfigDeepSleepClocks(uint32_t mask);

/**@} end of group pwrseq */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_LP_H_
