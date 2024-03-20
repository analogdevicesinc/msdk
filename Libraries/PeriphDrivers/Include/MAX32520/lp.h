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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_LP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_LP_H_

/* **** Includes **** */
#include <stdint.h>
#include "pwrseq_regs.h"
#include "mcr_regs.h"
#include "gcr_regs.h"
#include "gpio.h"

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
 * @brief   Enumeration type for PM Mode
 *
 */
typedef enum {
    MXC_LP_IPO = MXC_F_GCR_PM_IPO_PD,
    MXC_LP_IBRO = MXC_F_GCR_PM_IBRO_PD,
} mxc_lp_cfg_ds_pd_t;

/**
 * @brief      Places the device into SLEEP mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterSleepMode(void);

/**
 * @brief      Places the device into DEEPSLEEP mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterDeepSleepMode(void);

/**
 * @brief      Places the device into BACKUP mode.  CPU state is not maintained in this mode, so this function never returns.
 *             Instead, the device will restart once an RTC or external interrupt occur.
 */
void MXC_LP_EnterBackupMode(void);

/**
 * @brief      Places the device into Shutdown mode.  CPU state is not maintained in this mode, so this function never returns.
 *             Instead, the device will restart once an RTC, USB wakeup, or external interrupt occur.
 */
void MXC_LP_EnterShutDownMode(void);

/**
 * @brief      Set ovr bits to set the voltage the micro will run at.
 *
 * @param[in]  ovr   The ovr options are only 0.9V, 1.0V, and 1.1V use enum mxc_lp_ovr_t
 */
void MXC_LP_SetOVR(mxc_lp_ovr_t ovr);

/**
 * @brief      Enable VDD Core Monitor
 */
void MXC_LP_VCOREoreMonitorEnable(void);

/**
 * @brief      Disable VDD Core Monitor
 */
void MXC_LP_VCOREoreMonitorDisable(void);

/**
 * @brief      Is VDD Core Monitor enabled
 *
 * @return     1 = enabled , 0 = disabled
 */
int MXC_LP_VCOREoreMonitorIsEnabled(void);

/**
 * @brief      Enable LDO
 */
void MXC_LP_LDOEnable(void);

/**
 * @brief      Disable LDO
 */
void MXC_LP_LDODisable(void);

/**
 * @brief      Is LDO enabled
 *
 * @return     1 = enabled , 0 = disabled
 */
int MXC_LP_LDOIsEnabled(void);

/**
 * @brief      clear all wake up status
 */
void MXC_LP_ClearWakeStatus(void);

/**
 * @brief      Enables the selected GPIO port and its selected pins to wake up the device from any low power mode.
 *             Call this function multiple times to enable pins on multiple ports.  This function does not configure
 *             the GPIO pins nor does it setup their interrupt functionality.
 * @param      wu_pins      The port and pins to configure as wakeup sources.  Only the gpio and mask fields of the
 *                          structure are used.  The func and pad fields are ignored. \ref mxc_gpio_cfg_t
 */
void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t *wu_pins);

/**
 * @brief      Disables the selected GPIO port and its selected pins as a wake up source.
 *             Call this function multiple times to disable pins on multiple ports.
 * @param      wu_pins      The port and pins to disable as wakeup sources.  Only the gpio and mask fields of the
 *                          structure are used.  The func and pad fields are ignored. \ref mxc_gpio_cfg_t
 */
void MXC_LP_DisableGPIOWakeup(mxc_gpio_cfg_t *wu_pins);

/**
 * @brief      Configure which clocks are powered down at deep sleep and which are not affected.
 *
 * @note       Need to configure all clocks at once any clock not passed in the mask will be unaffected by Deepsleep.  This will
 *             always overwrite the previous settings of ALL clocks.
 *
 * @param[in]  mask  The mask of the clocks to power down when part goes into deepsleep
 *
 * @return     #E_NO_ERROR or error based on \ref MXC_Error_Codes
 */
int MXC_LP_ConfigDeepSleepClocks(uint32_t mask);

/**
 * @brief Enable System Ram 0 in light sleep
 */
void MXC_LP_SysRam0LightSleepEnable(void);

/**
 * @brief Enable System Ram 1 in light sleep
 */
void MXC_LP_SysRam1LightSleepEnable(void);

/**
 * @brief Enable System Ram 2 in light sleep
 */
void MXC_LP_SysRam2LightSleepEnable(void);

/**
 * @brief Enable System Ram 3 in light sleep
 */
void MXC_LP_SysRam3LightSleepEnable(void);

/**
 * @brief Enable System Ram 4 in light sleep
 */
void MXC_LP_SysRam4LightSleepEnable(void);

/**
 * @brief Enable Icache 0 in light sleep
 */
void MXC_LP_ICache0LightSleepEnable(void);

/**
 * @brief Enable ROM 0 in light sleep
 */
void MXC_LP_ROMLightSleepEnable(void);

/**
 * @brief Disable System Ram 0 in light sleep
 */
void MXC_LP_SysRam0LightSleepDisable(void);

/**
 * @brief Disable System Ram 1 in light sleep
 */
void MXC_LP_SysRam1LightSleepDisable(void);

/**
 * @brief Disable System Ram 2 in light sleep
 */
void MXC_LP_SysRam2LightSleepDisable(void);

/**
 * @brief Disable System Ram 3 in light sleep
 */
void MXC_LP_SysRam3LightSleepDisable(void);

/**
 * @brief Disable System Ram 4 in light sleep
 */
void MXC_LP_SysRam4LightSleepDisable(void);

/**
 * @brief Disable Icache 0 in light sleep
 */
void MXC_LP_ICache0LightSleepDisable(void);

/**
 * @brief Disable ROM 0 in light sleep
 */
void MXC_LP_ROMLightSleepDisable(void);

/**
 * @brief Shutdown Internal Cache
 */
void MXC_LP_ICache0Shutdown(void);

/**
 * @brief PowerUp Internal Cache
 */
void MXC_LP_ICache0PowerUp(void);

/**
 * @brief Shutdown ROM
 */
void MXC_LP_ROMShutdown(void);

/**
 * @brief PowerUp ROM
 */
void MXC_LP_ROMPowerUp(void);

/**@} end of group pwrseq */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_LP_H_
