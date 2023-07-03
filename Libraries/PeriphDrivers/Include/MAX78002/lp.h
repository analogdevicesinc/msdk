/**
 * @file    lp.h
 * @brief   Low Power(LP) function prototypes and data types.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_LP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_LP_H_

/* **** Includes **** */
#include <stdint.h>
#include "pwrseq_regs.h"
#include "mcr_regs.h"
#include "gcr_regs.h"
#include "lpcmp.h"
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
 * @brief      Enables the LPCMP to wake up the device from any low power mode.
 */
void MXC_LP_EnableLPCMPWakeup(mxc_lpcmp_cmpsel_t cmp);

/**
 * @brief      Disables the LPCMP from waking up the device.
 */
void MXC_LP_DisableLPCMPWakeup(mxc_lpcmp_cmpsel_t cmp);

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

/**@} end of group pwrseq */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_LP_H_
