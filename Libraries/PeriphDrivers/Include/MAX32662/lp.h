/**
 * @file    lp.h
 * @brief   Low Power(LP) function prototypes and data types.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_LP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_LP_H_

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
 * @brief      Places the device into SLEEP mode.  This function returns once any interrupt occurs.
 * @note 	   MXC_LP_ClearWakeStatus should be called before this function, to avoid immediately waking up again
 */
void MXC_LP_EnterSleepMode(void);

/**
 * @brief      Places the device into DEEPSLEEP mode.  This function returns once an RTC or external interrupt occur.
 * @note      MXC_LP_ClearWakeStatus should be called before this function, to avoid immediately waking up again
*/
void MXC_LP_EnterDeepSleepMode(void);

/**
 * @brief      Places the device into BACKUP mode.  CPU state is not maintained in this mode, so this function never returns.
 *             Instead, the device will restart once an RTC or external interrupt occur.
 * @note       MXC_LP_ClearWakeStatus should be called before this function, to avoid immediately waking up again
 */
void MXC_LP_EnterBackupMode(void);

/**
 * @brief      Places the device into Shutdown mode.  CPU state is not maintained in this mode, so this function never returns.
 *             Instead, the device will restart once an RTC, USB wakeup, or external interrupt occur.
 */
void MXC_LP_EnterShutDownMode(void);

/**
 * @brief      Enables power to System RAM block 3.
 */
void MXC_LP_EnableSRAM3(void);

/**
 * @brief      Disables power to System RAM block 3. The contents of the RAM are destroyed.
 */
void MXC_LP_DisableSRAM3(void);

/**
 * @brief      Enables power to System RAM block 2.
 */
void MXC_LP_EnableSRAM2(void);

/**
 * @brief      Disables power to System RAM block 2. The contents of the RAM are destroyed.
 */
void MXC_LP_DisableSRAM2(void);

/**
 * @brief      Enables power to System RAM block 1.
 */
void MXC_LP_EnableSRAM1(void);

/**
 * @brief      Disables power to System RAM block 1. The contents of the RAM are destroyed.
 */
void MXC_LP_DisableSRAM1(void);

/**
 * @brief      Enables power to System RAM block 0.
 */
void MXC_LP_EnableSRAM0(void);

/**
 * @brief      Disables power to System RAM block 0. The contents of the RAM are destroyed.
 */
void MXC_LP_DisableSRAM0(void);

/**
 * @brief      Enables power to the specified System RAM block.
 * 
 * @param[in] block The System RAM block to enable (0, 1, 2, or 3)
 * 
 * @return E_BAD_PARAM if the block specified is invalid, otherwise E_SUCCESS
 */
int MXC_LP_EnableSRAM(int block);

/**
 * @brief      Disables power to the specified System RAM block.  The contents of the RAM are destroyed.
 * 
 * @param[in] block The System RAM block to enable (0, 1, 2, or 3)
 * 
 * @return E_BAD_PARAM if the block specified is invalid, otherwise E_SUCCESS
 */
int MXC_LP_DisableSRAM(int block);

/**
 * @brief      NOT SUPPORTED. Set OVR bits to set the voltage the micro will run.
 *
 * @param[in]  ovr   The ovr options are only 0.9V, 1.0V, and 1.1V use enum mxc_lp_ovr_t
 *
 * @return     E_NOT_SUPPORTED.
 */
int MXC_LP_SetOVR(mxc_lp_ovr_t ovr);

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

void MXC_LP_EnableGPIOWakeup(const mxc_gpio_cfg_t *wu_pins);

/**
 * @brief      Disables the selected GPIO port and its selected pins as a wake up source.
 *             Call this function multiple times to disable pins on multiple ports.
 * @param      wu_pins      The port and pins to disable as wakeup sources.  Only the gpio and mask fields of the
 *                          structure are used.  The func and pad fields are ignored.
 */
void MXC_LP_DisableGPIOWakeup(const mxc_gpio_cfg_t *wu_pins);

/**
 * @brief      Enables the RTC alarm to wake up the device from any low power mode.
 */
void MXC_LP_EnableRTCAlarmWakeup(void);

/**
 * @brief      Disables the RTC alarm from waking up the device.
 */
void MXC_LP_DisableRTCAlarmWakeup(void);

/**
 * @brief      Enables Timer to wakeup from any low power mode.  Only TMR3 is supported.
 * 
 * @param      tmr  Pointer to timer module.
 */
void MXC_LP_EnableTimerWakeup(mxc_tmr_regs_t *tmr);

/**
 * @brief      Disables Timer from waking up device.  Only TMR3 is supported.
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
 * @note       Need to configure all clocks at once any clock not passed in the mask will be unaffected by Deepsleep.  This will
 *             always overwrite the previous settings of ALL clocks.
 *
 * @param[in]  mask  The mask of the clocks to power down when part goes into deepsleep
 *
 * @return     #E_NO_ERROR or error based on /ref MXC_Error_Codes
 */
int MXC_LP_ConfigDeepSleepClocks(uint32_t mask);

/**@} end of group pwrseq */

/**
 * @brief      Enable Internal Cache Controller RAM Light Sleep mode.
 */

void MXC_LP_EnableICacheLightSleep(void);

/**
 * @brief      Disable Internal Cache Controller RAM Light Sleep mode.
 */
void MXC_LP_DisableICacheLightSleep(void);

/**
 * @brief      Enable ROM Light Sleep mode.
 */
void MXC_LP_ROMLightSleepEnable(void);

/**
 * @brief      Disable ROM Light Sleep mode.
 */
void MXC_LP_RomLightSleepDisable(void);

/**
 * @brief      Enable Light Sleep mode for the specified System RAM instance
 * 
 * @param[in] instance The System RAM instance to enable Light Sleep mode for (0, 1, 2, or 3)
 * 
 * @return E_BAD_PARAM if the system RAM instance specified is invalid, otherwise E_SUCCESS
 */
int MXC_LP_EnableSysRAMLightSleep(int instance);

/**
 * @brief      Disable Light Sleep mode for the specified System RAM instance
 * 
 * @param[in] instance The System RAM instance to disable Light Sleep mode for (0, 1, 2, or 3)
 * 
 * @return E_BAD_PARAM if the system RAM instance specified is invalid, otherwise E_SUCCESS
 */
int MXC_LP_DisableSysRAMLightSleep(int instance);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_LP_H_
