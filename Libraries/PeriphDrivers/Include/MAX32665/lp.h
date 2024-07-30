/**
 * @file    lp.h
 * @brief   Low power function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_LP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_LP_H_

/* **** Includes **** */
#include <stdint.h>
#include "gpio.h"
#include "pwrseq_regs.h"
#include "mcr_regs.h"
#include "gcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup pwrseq Low Power (LP)
 * @ingroup periphlibs
 * @{
 */

typedef enum {
    MXC_RETAIN_NONE = MXC_S_PWRSEQ_LPCN_RAMRET_DIS,
    MXC_RETAIN_32k = MXC_S_PWRSEQ_LPCN_RAMRET_EN1,
    MXC_RETAIN_64k = MXC_S_PWRSEQ_LPCN_RAMRET_EN2,
    MXC_RETAIN_ALL = MXC_S_PWRSEQ_LPCN_RAMRET_EN3
} mxc_ram_retained_t;

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
    MXC_LP_HIRC = MXC_F_GCR_PM_HIRCPD,
    MXC_LP_HIRC96M = MXC_F_GCR_PM_HIRC96MPD,
    MXC_LP_HIRC8M = MXC_F_GCR_PM_HIRC8MPD,
    MXC_LP_XTAL = MXC_F_GCR_PM_XTALPB,
} mxc_lp_cfg_ds_pd_t;

/**
 * @brief      Clears the wakup status bits.  
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
 * @brief      Enables the WUT alarm to wake up the device from any low power mode.
 */
void MXC_LP_EnableWUTAlarmWakeup(void);

/**
 * @brief      Disables the WUT alarm from waking up the device.
 */
void MXC_LP_DisableWUTAlarmWakeup(void);

/**
 * @brief Puts System Ram 0 in light sleep
 */
void MXC_LP_SysRam0LightSleepEnable(void);

/**
 * @brief Puts System Ram 1 in light sleep
 */
void MXC_LP_SysRam1LightSleepEnable(void);

/**
 * @brief Puts System Ram 2 in light sleep
 */
void MXC_LP_SysRam2LightSleepEnable(void);

/**
 * @brief Puts System Ram 3 in light sleep
 */
void MXC_LP_SysRam3LightSleepEnable(void);

/**
 * @brief Puts System Ram 4 in light sleep
 */
void MXC_LP_SysRam4LightSleepEnable(void);

/**
 * @brief Puts System Ram 5 in light sleep
 */
void MXC_LP_SysRam5LightSleepEnable(void);

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
 * @brief Wakeup System Ram 1
 */
void MXC_LP_SysRam1PowerUp(void);

/**
 * @brief Shutdown System Ram 2
 */
void MXC_LP_SysRam2Shutdown(void);

/**
 * @brief Wakeup System Ram 2
 */
void MXC_LP_SysRam2PowerUp(void);

/**
 * @brief Shutdown System Ram 3
 */
void MXC_LP_SysRam3Shutdown(void);

/**
 * @brief Wakeup System Ram 3
 */
void MXC_LP_SysRam3PowerUp(void);

/**
 * @brief Shutdown System Ram 4
 */
void MXC_LP_SysRam4Shutdown(void);

/**
 * @brief Wakeup System Ram 4
 */
void MXC_LP_SysRam4PowerUp(void);

/**
 * @brief Shutdown System Ram 5
 */
void MXC_LP_SysRam5Shutdown(void);

/**
 * @brief Wakeup System Ram 5
 */
void MXC_LP_SysRam5PowerUp(void);

/**
 * @brief Shutdown Internal Cache
 */
void MXC_LP_ICache0Shutdown(void);

/**
 * @brief Wakeup Internal Cache
 */
void MXC_LP_ICache0PowerUp(void);

/**
 * @brief Shutdown Internal Cache XIP
 */
void MXC_LP_ICacheXIPShutdown(void);

/**
 * @brief Wakeup Internal Cache XIP
 */
void MXC_LP_ICacheXIPPowerUp(void);

/**
 * @brief Shutdown Crypto
 */
void MXC_LP_CryptoShutdown(void);

/**
 * @brief Wakeup Crypto
 */
void MXC_LP_CryptoPowerUp(void);

/**
 * @brief Shutdown USB FIFO
 */
void MXC_LP_USBFIFOShutdown(void);

/**
 * @brief Wakeup USB FIFO
 */
void MXC_LP_USBFIFOPowerUp(void);

/**
 * @brief Shutdown ROM
 */
void MXC_LP_ROM0Shutdown(void);

/**
 * @brief Wakeup ROM
 */
void MXC_LP_ROM0PowerUp(void);

/**
 * @brief Shutdown ROM 1
 */
void MXC_LP_ROM1Shutdown(void);

/**
 * @brief Wakeup ROM 1
 */
void MXC_LP_ROM1PowerUp(void);

/**
 * @brief Shutdown Internal Cache 1
 */
void MXC_LP_ICache1Shutdown(void);

/**
 * @brief Wakeup Internal Cache 1
 */
void MXC_LP_ICache1PowerUp(void);

/**
 * @brief Disable USB Software Low Power
 */
void MXC_LP_USBSWLPDisable(void);

/**
 * @brief Enable USB Software Low Power 
 */
void MXC_LP_USBSWLPEnable(void);

/**
 * @brief Power Down VDD2
 */
void MXC_LP_VDD2PowerDown(void);

/**
 * @brief Power up VDD2
 */
void MXC_LP_VDD2PowerUp(void);

/**
 * @brief Power Down VDD3
 */
void MXC_LP_VDD3PowerDown(void);

/**
 * @brief Power Up VDD3
 */
void MXC_LP_VDD3PowerUp(void);

/**
 * @brief Power Down VDD4
 */
void MXC_LP_VDD4PowerDown(void);

/**
 * @brief Power Up VDD4
 */
void MXC_LP_VDD4PowerUp(void);

/**
 * @brief Power Down VDD5
 */
void MXC_LP_VDD5PowerDown(void);

/**
 * @brief Power Up VDD5
 */
void MXC_LP_VDD5PowerUp(void);

/**
 * @brief Power Down SIMOV regB
 */
void MXC_LP_SIMOVregBPowerDown(void);

/**
 * @brief Power Up SIMOV regB 
 */
void MXC_LP_SIMOVregBPowerUp(void);

/**
 * @brief Power Down SIMOV regD 
 */
void MXC_LP_SIMOVregDPowerDown(void);

/**
 * @brief Power Up SIMOV regD
 */
void MXC_LP_SIMOVregDPowerUp(void);

/**
 * @brief Enable Fast Wakeup
 * @details Deprecated due to issues with SIMO in wakeup.
 */
void __attribute__((deprecated("Causes SIMO soft start in wakeup"))) MXC_LP_FastWakeupEnable(void);

/**
 * @brief Disable Fast Wakeup
 */
void MXC_LP_FastWakeupDisable(void);

/**
 * @brief Enables the selected amount of RAM retention in backup mode
 *        Using any RAM retention removes the ability to shut down VcoreB
 */
void MXC_LP_SetRAMRetention(mxc_ram_retained_t ramRetained);

/**
 * @brief      Places the device into SLEEP mode.  This function returns once any interrupt occurs. 
 */
void MXC_LP_EnterSleepMode(void);

/**
 * @brief      Places the device into DEEPSLEEP mode.  This function returns once an RTC or external interrupt occur. 
 */
void MXC_LP_EnterDeepSleepMode(void);

/**
 * @brief      Places the device into BACKGROUND mode.  This function returns once an RTC or external interrupt occur.
 */
void MXC_LP_EnterBackgroundMode(void);

/**
 * @brief      Places the device into BACKUP mode.  CPU state is not maintained in this mode, so this function never returns.  
 *             Instead, the device will restart once an RTC or external interrupt occur. 
 * @param  	   func 	Function that backup mode returns to, if null, the part will return to Reset_Handler
 * @note 	   When returning from backup mode, depending on the RAM retention settings the processor
 * 			   could have no state information. It will not have a valid stack pointer. 
 * 			   This function also uses MXC_PWRSEQ->gp0 and gp1.
 */
void MXC_LP_EnterBackupMode(void *func(void));

/**
 * @brief      Places the device into Shutdown mode.  CPU state is not maintained in this mode, so this function never returns.  
 *             Instead, the device will restart once an RTC, USB wakeup, or external interrupt occur. 
 */
void MXC_LP_EnterShutDownMode(void);

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

/**
 * @brief      Set ovr bits to set the voltage the micro will run at.
 *
 * @param[in]  ovr   The ovr options are only 0.9V, 1.0V, and 1.1V use enum mxc_lp_ovr_t
 */
void MXC_LP_SetOVR(mxc_lp_ovr_t ovr);

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
 * @brief      Is Fast wake up is Enabled
 *
 * @return     1 = enabled , 0 = disabled
 */
int MXC_LP_FastWakeupIsEnabled(void);

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
 * @brief Disable Icache 0 in light sleep
 */
void MXC_LP_ICache0LightSleepDisable(void);

/**
 * @brief Disable Icache 1 in light sleep
 */
void MXC_LP_ICache1LightSleepDisable(void);

/**
 * @brief Disable Icache XIP in light sleep
 */
void MXC_LP_ICacheXIPLightSleepDisable(void);

/**
 * @brief Enable System Cache in light sleep
 */
void MXC_LP_SRCCLightSleepEnable(void);

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
 * @brief Enable ROM 0 in light sleep
 */
void MXC_LP_ROM1LightSleepEnable(void);

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
 * @brief Disable System Ram 5 in light sleep
 */
void MXC_LP_SysRam5LightSleepDisable(void);

/**
 * @brief Disable System Cache in light sleep
 */
void MXC_LP_SRCCLightSleepDisable(void);

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
 * @brief Shutdown SRCC
 */
void MXC_LP_SRCCShutdown(void);

/**
 * @brief PowerUp SRCC
 */
void MXC_LP_SRCCPowerUp(void);

/**
 * @brief Enable Icache XIP in light sleep
 */
void MXC_LP_ICacheXIPLightSleepEnable(void);

/**
 * @brief Enable Icache 0 in light sleep
 */
void MXC_LP_ICache0LightSleepEnable(void);

/**
 * @brief Enable Icache 0 in light sleep
 */
void MXC_LP_ICache1LightSleepEnable(void);

#ifdef __cplusplus
}
#endif
/**@} end of group pwrseq */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_LP_H_
