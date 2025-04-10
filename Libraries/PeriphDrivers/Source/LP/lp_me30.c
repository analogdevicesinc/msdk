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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "mcr_regs.h"
#include "lp.h"

/* ARM */
#define SET_SLEEPDEEP(X) (SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk)
#define CLR_SLEEPDEEP(X) (SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk)

void MXC_LP_EnterSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Clear SLEEPDEEP bit */
    CLR_SLEEPDEEP();

    /* Go into Sleep mode and wait for an interrupt to wake the processor */
    __WFI();
}

void MXC_LP_EnterStandbyMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Clear SLEEPDEEP bit */
    SET_SLEEPDEEP();

    /* Go into Standby mode and wait for an interrupt to wake the processor */
    __WFI();

    /* Clear SLEEPDEEP bit to prevent WFI from entering deep sleep */
    CLR_SLEEPDEEP();
}

void MXC_LP_EnterBackupMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Enable backup mode */
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_BACKUP;

    while (1) {}
    // Should never reach this line - device will jump to backup vector on exit from backup mode.
}

void MXC_LP_ExitBackupMode(void)
{
    MXC_LP_ClearWakeStatus();
    CLR_SLEEPDEEP();
}

void MXC_LP_EnterPowerDownMode(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_V_GCR_PM_MODE_PDM;

    while (1) {}
    // Should never reach this line - device will reset on exit from shutdown mode.
}

void MXC_LP_SetOVR(mxc_lp_ovr_t ovr)
{
    //not supported yet
}

void MXC_LP_EnableRetentionReg(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_RETLDO_EN;
}

void MXC_LP_DisableRetentionReg(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_RETLDO_EN;
}

int MXC_LP_RetentionRegIsEnabled(void)
{
    return (MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_RETLDO_EN);
}

void MXC_LP_EnableSramRetention(uint32_t mask)
{
    MXC_PWRSEQ->lpctrl |= (mask & 0x1F) << MXC_F_PWRSEQ_LPCTRL_SRAMRET_EN_POS;
}

void MXC_LP_DisableSramRetention(uint32_t mask)
{
    MXC_PWRSEQ->lpctrl &= ~((mask & 0x1F) << MXC_F_PWRSEQ_LPCTRL_SRAMRET_EN_POS);
}

void MXC_LP_BandgapOn(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_BG_DIS;
}

void MXC_LP_BandgapOff(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_BG_DIS;
}

int MXC_LP_BandgapIsOn(void)
{
    return (MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_BG_DIS);
}

void MXC_LP_ClearWakeStatus(void)
{
    /* Write 1 to clear */
    MXC_PWRSEQ->lpwkfl0 = 0xFFFFFFFF;
    MXC_PWRSEQ->lppwst = 0xFFFFFFFF;
}

void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_GPIO_WE;

    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 |= wu_pins->mask;
        break;
    }
}

void MXC_LP_DisableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
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

void MXC_LP_EnableWUTAlarmWakeup(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_WUT_WE;
}

void MXC_LP_DisableWUTAlarmWakeup(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_WUT_WE;
}

int MXC_LP_ConfigDeepSleepClocks(uint32_t mask)
{
    if (!(mask & MXC_F_MCR_CTRL_ERTCO_EN)) {
        return E_BAD_PARAM;
    }

    MXC_MCR->ctrl &= ~mask;

    return E_NO_ERROR;
}
