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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_LP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_LP_H_

/* **** Includes **** */
#include <stdint.h>
#include "pwrseq_regs.h"
#include "mcr_regs.h"
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
    MXC_LP_ISO = MXC_F_GCR_PM_ISO_PD,
    MXC_LP_IPO = MXC_F_GCR_PM_IPO_PD,
    MXC_LP_IBRO = MXC_F_GCR_PM_IBRO_PD,
    MXC_LP_XRFO = MXC_F_GCR_PM_ERFO_PD,
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
 * @brief      Enable retention regulator
 */
void MXC_LP_RetentionRegEnable(void);

/**
 * @brief      Disable retention regulator
 */
void MXC_LP_RetentionRegDisable(void);

/**
 * @brief      Is the retention regulator enabled
 *
 * @return     1 = enabled 0 =  disabled
 */
int MXC_LP_RetentionRegIsEnabled(void);

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
 * @brief      Enable Power on Reset VDD Core Monitor
 */
void MXC_LP_PORVCOREoreMonitorEnable(void);

/**
 * @brief      Disable Power on Reset VDD Core Monitor
 */
void MXC_LP_PORVCOREoreMonitorDisable(void);

/**
 * @brief      Is Power on Reset VDD Core Monitor enabled
 *
 * @return     1 = enabled , 0 = disabled
 */
int MXC_LP_PORVCOREoreMonitorIsEnabled(void);

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
 * @brief      Enable Fast wakeup
 */
void MXC_LP_FastWakeupEnable(void);

/**
 * @brief      Disable Fast wakeup
 */
void MXC_LP_FastWakeupDisable(void);

/**
 * @brief      Is Fast wake up is Enabled
 *
 * @return     1 = enabled , 0 = disabled
 */
int MXC_LP_FastWakeupIsEnabled(void);

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
 * @brief      Enables the RTC alarm to wake up the device from any low power mode.
 */
void MXC_LP_EnableRTCAlarmWakeup(void);

/**
 * @brief      Disables the RTC alarm from waking up the device.
 */
void MXC_LP_DisableRTCAlarmWakeup(void);
/**
 * @brief      Enables the USB to wake up the device from any low power mode.
 */
void MXC_LP_EnableUSBWakeup(void);

/**
 * @brief      Disables the USB from waking up the device.
 */
void MXC_LP_DisableUSBWakeup(void);

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
 * @brief      Enable NFC Oscilator Bypass
 */
void MXC_LP_NFCOscBypassEnable(void);

/**
 * @brief      Disable NFC Oscilator Bypass
 */
void MXC_LP_NFCOscBypassDisable(void);

/**
 * @brief      Is NFC Oscilator Bypass Enabled
 *
 * @return     1 = enabled, 0 = disabled
 */
int MXC_LP_NFCOscBypassIsEnabled(void);

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
 * @brief Enable System Ram 5 in light sleep
 */
void MXC_LP_SysRam5LightSleepEnable(void);

/**
 * @brief Enable System Ram 6 in light sleep
 */
void MXC_LP_SysRam6LightSleepEnable(void);

/**
 * @brief Enable Icache XIP in light sleep
 */
void MXC_LP_ICacheXIPLightSleepEnable(void);

/**
 * @brief Enable Crypto in light sleep
 */
void MXC_LP_CryptoLightSleepEnable(void);

/**
 * @brief Enable USB in light sleep
 */
void MXC_LP_USBFIFOLightSleepEnable(void);

/**
 * @brief Enable ROM 0 in light sleep
 */
void MXC_LP_ROM0LightSleepEnable(void);

/**
 * @brief Enable ROM 1 in light sleep
 */
void MXC_LP_ROM1LightSleepEnable(void);

/**
 * @brief Enable MAA in light sleep
 */
void MXC_LP_MAALightSleepEnable(void);

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
 * @brief Enable System Ram 4 in light sleep
 */
void MXC_LP_SysRam4LightSleepDisable(void);

/**
 * @brief Enable System Ram 5 in light sleep
 */
void MXC_LP_SysRam5LightSleepDisable(void);

/**
 * @brief Enable System Ram 6 in light sleep
 */
void MXC_LP_SysRam6LightSleepDisable(void);

/**
 * @brief Disable System Ram 3 in light sleep
 */
void MXC_LP_SysRam3LightSleepDisable(void);

/**
 * @brief Disable Icache XIP in light sleep
 */
void MXC_LP_ICacheXIPLightSleepDisable(void);

/**
 * @brief Disable Crypto in light sleep
 */
void MXC_LP_CryptoLightSleepDisable(void);

/**
 * @brief Disable USB in light sleep
 */
void MXC_LP_USBFIFOLightSleepDisable(void);

/**
 * @brief Disable ROM 0 in light sleep
 */
void MXC_LP_ROM0LightSleepDisable(void);

/**
 * @brief Disable ROM 1 in light sleep
 */
void MXC_LP_ROM1LightSleepDisable(void);

/**
 * @brief Disable MAA in light sleep
 */
void MXC_LP_MAALightSleepDisable(void);

/**
 * @brief Shutdown System Ram 0
 */
void MXC_LP_SysRam0Shutdown(void);

/**
 * @brief Wakeup System Ram 0
 */
void MXC_LP_SysRam0PowerUp(void);

/**
 * @brief Shutdown System Ram 1
 */
void MXC_LP_SysRam1Shutdown(void);

/**
 * @brief PowerUp System Ram 1
 */
void MXC_LP_SysRam1PowerUp(void);

/**
 * @brief Shutdown System Ram 2
 */
void MXC_LP_SysRam2Shutdown(void);

/**
 * @brief PowerUp System Ram 2
 */
void MXC_LP_SysRam2PowerUp(void);

/**
 * @brief Shutdown System Ram 3
 */
void MXC_LP_SysRam3Shutdown(void);

/**
 * @brief PowerUp System Ram 3
 */
void MXC_LP_SysRam3PowerUp(void);

/**
 * @brief Shutdown System Ram 4
 */
void MXC_LP_SysRam4Shutdown(void);

/**
 * @brief PowerUp System Ram 4
 */
void MXC_LP_SysRam4PowerUp(void);

/**
 * @brief Shutdown System Ram 5
 */
void MXC_LP_SysRam5Shutdown(void);

/**
 * @brief PowerUp System Ram 5
 */
void MXC_LP_SysRam5PowerUp(void);

/**
 * @brief Shutdown System Ram 6
 */
void MXC_LP_SysRam6Shutdown(void);

/**
 * @brief PowerUp System Ram 6
 */
void MXC_LP_SysRam6PowerUp(void);

/**
 * @brief Shutdown Internal Cache XIP
 */
void MXC_LP_ICacheXIPShutdown(void);

/**
 * @brief PowerUp Internal Cache XIP
 */
void MXC_LP_ICacheXIPPowerUp(void);

/**
 * @brief Shutdown Crypto
 */
void MXC_LP_CryptoShutdown(void);

/**
 * @brief PowerUp Crypto
 */
void MXC_LP_CryptoPowerUp(void);

/**
 * @brief Shutdown USB FIFO
 */
void MXC_LP_USBFIFOShutdown(void);

/**
 * @brief PowerUp USB FIFO
 */
void MXC_LP_USBFIFOPowerUp(void);

/**
 * @brief Shutdown ROM0
 */
void MXC_LP_ROM0Shutdown(void);

/**
 * @brief PowerUp ROM0
 */
void MXC_LP_ROM0PowerUp(void);

/**
 * @brief Shutdown ROM1
 */
void MXC_LP_ROM1Shutdown(void);

/**
 * @brief PowerUp ROM1
 */
void MXC_LP_ROM1PowerUp(void);

/**@} end of group pwrseq */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_LP_H_
