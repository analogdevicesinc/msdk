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

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "mcr_regs.h"
#include "lp.h"

void MXC_LP_EnterSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Clear SLEEPDEEP bit */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Go into Sleep mode and wait for an interrupt to wake the processor */
    __WFI();
}

void MXC_LP_EnterDeepSleepMode(void)
{
    // Set SLEEPDEEP bit
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // // Auto-powerdown 96 MHz oscillator when in deep sleep
    // MXC_GCR->pm |= MXC_F_GCR_PM_HFIOPD; // Not supported on ME12
    // Go into Deepsleep mode and wait for an interrupt to wake the processor
    __WFI();
}

void MXC_LP_EnterBackupMode(void)
{
    MXC_LP_ClearWakeStatus();

    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_BACKUP;

    while (1) {}
    // Should never reach this line - device will jump to backup vector on exit from background mode.
}

void MXC_LP_EnterShutDownMode(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_SHUTDOWN;

    while (1) {}
    // Should never reach this line - device will reset on exit from shutdown mode.
}

void MXC_LP_EnableSRAM3(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM3;
}

void MXC_LP_DisableSRAM3(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM3;
}

void MXC_LP_EnableSRAM2(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM2;
}

void MXC_LP_DisableSRAM2(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM2;
}

void MXC_LP_EnableSRAM1(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM1;
}

void MXC_LP_DisableSRAM1(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM1;
}

void MXC_LP_EnableSRAM0(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM0;
}

void MXC_LP_DisableSRAM0(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM0;
}

int MXC_LP_EnableSRAM(int instance)
{
    if (instance == 0) {
        MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM0;
    } else if (instance == 1) {
        MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM1;
    } else if (instance == 2) {
        MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM2;
    } else if (instance == 3) {
        MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM3;
    } else {
        return E_BAD_PARAM;
    }

    return E_SUCCESS;
}

int MXC_LP_DisableSRAM(int instance)
{
    if (instance == 0) {
        MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM0;
    } else if (instance == 1) {
        MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM1;
    } else if (instance == 2) {
        MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM2;
    } else if (instance == 3) {
        MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM3;
    } else {
        return E_BAD_PARAM;
    }

    return E_SUCCESS;
}

int MXC_LP_SetOVR(mxc_lp_ovr_t ovr)
{
    return E_NOT_SUPPORTED;
}

void MXC_LP_BandgapOn(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_BG_DIS; // 0 = Bandgap is always ON
}

void MXC_LP_BandgapOff(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_BG_DIS; // 1 = Bandgap is always OFF.
}

int MXC_LP_BandgapIsOn(void)
{
    return ~(MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_BG_DIS); // Logic on Bandgap bit is inverted
}

void MXC_LP_ClearWakeStatus(void)
{
    /* Write 1 to clear */
    MXC_PWRSEQ->lpwkfl0 = 0xFFFFFFFF;
    MXC_PWRSEQ->lppwkfl = 0xFFFFFFFF;
}

void MXC_LP_EnableGPIOWakeup(const mxc_gpio_cfg_t *wu_pins)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_GPIO_WE;

    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 |= wu_pins->mask;
        break;
    }
}

void MXC_LP_DisableGPIOWakeup(const mxc_gpio_cfg_t *wu_pins)
{
    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 &= ~wu_pins->mask;
        break;
    }

    if (MXC_PWRSEQ->lpwken0 == 0) {
        MXC_GCR->pm &= ~MXC_F_GCR_PM_GPIO_WE;
    }
}

void MXC_LP_EnableRTCAlarmWakeup(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_RTC_WE;
}

void MXC_LP_DisableRTCAlarmWakeup(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_RTC_WE;
}

void MXC_LP_EnableTimerWakeup(mxc_tmr_regs_t *tmr)
{
    if (tmr == MXC_TMR3) {
        // MXC_TMR3 (LPTIMER0) is the only timer that supports WE
        MXC_GCR->pm |= MXC_F_GCR_PM_TMR3_WE;
    }
}

void MXC_LP_DisableTimerWakeup(mxc_tmr_regs_t *tmr)
{
    if (tmr == MXC_TMR3) {
        // MXC_TMR3 (LPTIMER0) is the only timer that supports WE
        MXC_GCR->pm &= ~(MXC_F_GCR_PM_TMR3_WE);
    }
}

void MXC_LP_EnableICacheLightSleep(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ICC0LS_EN;
}

void MXC_LP_DisableICacheLightSleep(void)
{
    MXC_GCR->memctrl &= ~(MXC_F_GCR_MEMCTRL_ICC0LS_EN);
}

void MXC_LP_ROMLightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ROMLS_EN;
}

void MXC_LP_RomLightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~(MXC_F_GCR_MEMCTRL_ROMLS_EN);
}

int MXC_LP_EnableSysRAMLightSleep(int instance)
{
    if (instance == 0) {
        MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM0LS_EN;
    } else if (instance == 1) {
        MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM1LS_EN;
    } else if (instance == 2) {
        MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM2LS_EN;
    } else if (instance == 3) {
        MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM3LS_EN;
    } else {
        return E_BAD_PARAM;
    }

    return E_SUCCESS;
}

int MXC_LP_DisableSysRAMLightSleep(int instance)
{
    if (instance == 0) {
        MXC_GCR->memctrl &= ~(MXC_F_GCR_MEMCTRL_RAM0LS_EN);
    } else if (instance == 1) {
        MXC_GCR->memctrl &= ~(MXC_F_GCR_MEMCTRL_RAM1LS_EN);
    } else if (instance == 2) {
        MXC_GCR->memctrl &= ~(MXC_F_GCR_MEMCTRL_RAM2LS_EN);
    } else if (instance == 3) {
        MXC_GCR->memctrl &= ~(MXC_F_GCR_MEMCTRL_RAM1LS_EN);
    } else {
        return E_BAD_PARAM;
    }

    return E_SUCCESS;
}
