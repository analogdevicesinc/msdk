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
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "flc_regs.h"
#include "lp.h"

void MXC_LP_EnterSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    // set block detect bit
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_VCORE_DET_BYPASS;

    // Clear SLEEPDEEP bit
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    // Go into Sleep mode and wait for an interrupt to wake the processor
    __WFI();
}

void MXC_LP_EnterDeepSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    // set block detect bit
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_VCORE_DET_BYPASS;

    // Set SLEEPDEEP bit
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Go into Deepsleep mode and wait for an interrupt to wake the processor
    __WFI();
}

void MXC_LP_EnterBackupMode(void)
{
    MXC_LP_ClearWakeStatus();

    // set block detect bit
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_VCORE_DET_BYPASS;

    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_BACKUP;

    while (1) {}
    // Should never reach this line - device will jump to backup vector on exit from background mode.
}

void MXC_LP_EnterStorageMode(void)
{
    MXC_LP_ClearWakeStatus();
    /*set block detect bit */
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_VCORE_DET_BYPASS;

    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_STORAGE_EN;
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

int MXC_LP_SetOVR(mxc_lp_ovr_t ovr)
{
    uint32_t current_clock, div;
    int error;

    // Ensure part is operating from internal LDO for core power
    if (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_LDO_DIS) {
        return E_BAD_STATE;
    }

    // Select the 8KHz nanoring (no guarantee 32KHz is attached) as system clock source
    current_clock = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL;
    if (current_clock == MXC_SYS_CLOCK_IPO) {
        error = MXC_SYS_Clock_Select(MXC_SYS_CLOCK_INRO);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    // Set flash wait state for any clock so its not to low after clock changes.
    MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                       (0x5UL << MXC_F_GCR_MEMCTRL_FWS_POS);

    // Set the OVR bits
    //  The OVR enums in mxc_lp_ovr_t equals to their appropriate register setting.
    MXC_SETFIELD(MXC_PWRSEQ->lpcn, MXC_F_PWRSEQ_LPCN_OVR, ovr);

    // Set LVE bit
    if ((ovr == MXC_LP_OVR_0_9) || (ovr == MXC_LP_OVR_1_0)) {
        MXC_FLC0->ctrl |= MXC_F_FLC_CTRL_LVE;

    } else {
        MXC_FLC0->ctrl &= ~(MXC_F_FLC_CTRL_LVE);
    }

    // Revert the clock to original state if it was IPO
    if (current_clock == MXC_SYS_CLOCK_IPO) {
        error = MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    // Update SystemCoreClock variable
    SystemCoreClockUpdate();

    // Get the clock divider
    div = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_DIV) >> MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS;

    // Set Flash Wait States
    if (ovr == MXC_LP_OVR_0_9) {
        if (div == 0) {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x2UL << MXC_F_GCR_MEMCTRL_FWS_POS);

        } else {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x1UL << MXC_F_GCR_MEMCTRL_FWS_POS);
        }

    } else if (ovr == MXC_LP_OVR_1_0) {
        if (div == 0) {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x2UL << MXC_F_GCR_MEMCTRL_FWS_POS);

        } else {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x1UL << MXC_F_GCR_MEMCTRL_FWS_POS);
        }

    } else {
        if (div == 0) {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x4UL << MXC_F_GCR_MEMCTRL_FWS_POS);

        } else if (div == 1) {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x2UL << MXC_F_GCR_MEMCTRL_FWS_POS);

        } else {
            MXC_GCR->memctrl = (MXC_GCR->memctrl & ~(MXC_F_GCR_MEMCTRL_FWS)) |
                               (0x1UL << MXC_F_GCR_MEMCTRL_FWS_POS);
        }
    }

    // Caller must perform peripheral reset

    return E_NO_ERROR;
}

void MXC_LP_RetentionRegEnable(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_RETREG_EN;
}

void MXC_LP_RetentionRegDisable(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_RETREG_EN;
}

int MXC_LP_RetentionRegIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_RETREG_EN);
}

void MXC_LP_BandgapOn(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_BG_DIS;
}

void MXC_LP_BandgapOff(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BG_DIS;
}

int MXC_LP_BandgapIsOn(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_BG_DIS);
}

void MXC_LP_PORVCOREoreMonitorEnable(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_VCOREPOR_DIS;
}

void MXC_LP_PORVCOREoreMonitorDisable(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_VCOREPOR_DIS;
}

int MXC_LP_PORVCOREoreMonitorIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_VCOREPOR_DIS);
}

void MXC_LP_LDOEnable(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_LDO_DIS;
}

void MXC_LP_LDODisable(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_LDO_DIS;
}

int MXC_LP_LDOIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_LDO_DIS);
}

void MXC_LP_FastWakeupEnable(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_FASTWK_EN;
}

void MXC_LP_FastWakeupDisable(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_FASTWK_EN;
}

int MXC_LP_FastWakeupIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_FASTWK_EN);
}

void MXC_LP_ClearWakeStatus(void)
{
    // Write 1 to clear
    MXC_PWRSEQ->lpwkst0 = 0xFFFFFFFF;
    MXC_PWRSEQ->lpwkst1 = 0xFFFFFFFF;
    MXC_PWRSEQ->lppwkst = 0xFFFFFFFF;
}

void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_GPIO_WE;

    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 |= wu_pins->mask;
        break;

    case MXC_GPIO_PORT_1:
        MXC_PWRSEQ->lpwken1 |= wu_pins->mask;
    }
}

void MXC_LP_DisableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
{
    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 &= ~wu_pins->mask;
        break;

    case MXC_GPIO_PORT_1:
        MXC_PWRSEQ->lpwken1 &= ~wu_pins->mask;
    }

    if (MXC_PWRSEQ->lpwken1 == 0 && MXC_PWRSEQ->lpwken0 == 0) {
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
    MXC_ASSERT(MXC_TMR_GET_IDX(tmr) > 3);

    if (tmr == MXC_TMR4) {
        MXC_GCR->pm |= MXC_F_GCR_PM_LPTMR0_WE;
        MXC_PWRSEQ->lppwken |= MXC_F_PWRSEQ_LPPWKEN_LPTMR0;
    } else {
        MXC_GCR->pm |= MXC_F_GCR_PM_LPTMR1_WE;
        MXC_PWRSEQ->lppwken |= MXC_F_PWRSEQ_LPPWKEN_LPTMR1;
    }
}

void MXC_LP_DisableTimerWakeup(mxc_tmr_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX(tmr) > 3);

    if (tmr == MXC_TMR4) {
        MXC_GCR->pm &= ~MXC_F_GCR_PM_LPTMR0_WE;
        MXC_PWRSEQ->lppwken &= ~MXC_F_PWRSEQ_LPPWKEN_LPTMR0;
    } else {
        MXC_GCR->pm &= ~MXC_F_GCR_PM_LPTMR1_WE;
        MXC_PWRSEQ->lppwken &= ~MXC_F_PWRSEQ_LPPWKEN_LPTMR1;
    }
}

void MXC_LP_EnableUARTWakeup(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_LPUART0_WE;
    MXC_PWRSEQ->lppwken |= MXC_F_PWRSEQ_LPPWKEN_LPUART0;
}

void MXC_LP_DisableUARTWakeup(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_LPUART0_WE;
    MXC_PWRSEQ->lppwken &= ~MXC_F_PWRSEQ_LPPWKEN_LPUART0;
}

int MXC_LP_ConfigDeepSleepClocks(uint32_t mask)
{
    if (!(mask & (MXC_F_GCR_PM_IBRO_PD | MXC_F_GCR_PM_IPO_PD | MXC_F_GCR_PM_ERFO_PD))) {
        return E_BAD_PARAM;
    }

    MXC_GCR->pm |= mask;
    return E_NO_ERROR;
}

void MXC_LP_SysRam0LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM0LS_EN;
}

void MXC_LP_SysRam1LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM1LS_EN;
}

void MXC_LP_SysRam2LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM2LS_EN;
}

void MXC_LP_SysRam3LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM3LS_EN;
}

void MXC_LP_ICache0LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ICC0LS_EN;
}

void MXC_LP_ROMLightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ROMLS_EN;
}

void MXC_LP_SysRam0LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM0LS_EN;
}

void MXC_LP_SysRam1LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM1LS_EN;
}

void MXC_LP_SysRam2LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM2LS_EN;
}

void MXC_LP_SysRam3LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM3LS_EN;
}

void MXC_LP_ICache0LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ICC0LS_EN;
}

void MXC_LP_ROMLightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ROMLS_EN;
}

void MXC_LP_SysRam0Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM0;
}

void MXC_LP_SysRam0PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM0;
}

void MXC_LP_SysRam1Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM1;
}

void MXC_LP_SysRam1PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM1;
}

void MXC_LP_SysRam2Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM2;
}

void MXC_LP_SysRam2PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM2;
}

void MXC_LP_SysRam3Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM3;
}

void MXC_LP_SysRam3PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM3;
}
