/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "lp.h"

void MXC_LP_EnterSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    // Clear SLEEPDEEP bit
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    // Go into Sleep mode and wait for an interrupt to wake the processor
    __WFI();
}

void MXC_LP_EnterDeepSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    // Set SLEEPDEEP bit
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_DEEPSLEEP;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

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

void MXC_LP_SetOVR(mxc_lp_ovr_t ovr)
{
    //not supported yet
}

void MXC_LP_VCOREoreMonitorEnable(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_VCOREMON_DIS;
}

void MXC_LP_VCOREoreMonitorDisable(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_VCOREMON_DIS;
}

int MXC_LP_VCOREoreMonitorIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_VCOREMON_DIS);
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

int MXC_LP_ConfigDeepSleepClocks(uint32_t mask)
{
    if (!(mask & (MXC_F_GCR_PM_IBRO_PD | MXC_F_GCR_PM_IPO_PD))) {
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

void MXC_LP_SysRam4LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM4LS_EN;
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

void MXC_LP_SysRam4LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM4LS_EN;
}

void MXC_LP_ICache0LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ICC0LS_EN;
}

void MXC_LP_ROMLightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ROMLS_EN;
}

void MXC_LP_ICache0Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ICACHE;
}

void MXC_LP_ICache0PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ICACHE;
}

void MXC_LP_ROMShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ROM;
}

void MXC_LP_ROMPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ROM;
}
