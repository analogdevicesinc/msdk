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
#include "lp.h"

void MXC_LP_EnterSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

#ifndef __riscv
    // Clear SLEEPDEEP bit
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
#endif

    // Go into Sleep mode and wait for an interrupt to wake the processor
    __WFI();
}

void MXC_LP_EnterDeepSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    // Set SLEEPDEEP bit
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_DEEPSLEEP;

#ifndef __riscv
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#endif

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
    // TODO(SDK Team): Check this function once SBT is added for MAX32572.

    // MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    // MXC_GCR->pm |= MXC_S_GCR_PM_MODE_SHUTDOWN;

    // while (1) {}
    // // Should never reach this line - device will reset on exit from shutdown mode.

    return;
}

void MXC_LP_SetOVR(mxc_lp_ovr_t ovr)
{
    //not supported yet
}

void MXC_LP_RetentionRegEnable(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_RETREG_EN;
}

void MXC_LP_RetentionRegDisable(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_RETREG_EN;
}

int MXC_LP_RetentionRegIsEnabled(void)
{
    return (MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_RETREG_EN);
}

void MXC_LP_BandgapOn(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_BGOFF;
}

void MXC_LP_BandgapOff(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_BGOFF;
}

int MXC_LP_BandgapIsOn(void)
{
    return (MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_BGOFF);
}

void MXC_LP_PORVCOREoreMonitorEnable(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS;
}

void MXC_LP_PORVCOREoreMonitorDisable(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS;
}

int MXC_LP_PORVCOREoreMonitorIsEnabled(void)
{
    return (MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_VCOREPOR_DIS);
}

void MXC_LP_FastWakeupEnable(void)
{
    MXC_PWRSEQ->lpctrl |= MXC_F_PWRSEQ_LPCTRL_FASTWK_EN;
}

void MXC_LP_FastWakeupDisable(void)
{
    MXC_PWRSEQ->lpctrl &= ~MXC_F_PWRSEQ_LPCTRL_FASTWK_EN;
}

int MXC_LP_FastWakeupIsEnabled(void)
{
    return (MXC_PWRSEQ->lpctrl & MXC_F_PWRSEQ_LPCTRL_FASTWK_EN);
}

void MXC_LP_ClearWakeStatus(void)
{
    // Write 1 to clear
    MXC_PWRSEQ->lppwkfl = 0xFFFFFFFF;
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
        break;
    case MXC_GPIO_PORT_2:
        MXC_PWRSEQ->lpwken2 |= wu_pins->mask;
        break;
    case MXC_GPIO_PORT_3:
        MXC_PWRSEQ->lpwken3 |= wu_pins->mask;
        break;
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
        break;
    case MXC_GPIO_PORT_2:
        MXC_PWRSEQ->lpwken2 &= ~wu_pins->mask;
        break;
    case MXC_GPIO_PORT_3:
        MXC_PWRSEQ->lpwken3 &= ~wu_pins->mask;
        break;
    }

    if (MXC_PWRSEQ->lpwken3 == 0 && MXC_PWRSEQ->lpwken2 == 0 && MXC_PWRSEQ->lpwken1 == 0 &&
        MXC_PWRSEQ->lpwken0 == 0) {
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

void MXC_LP_EnableUSBWakeup(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_USB_WE;
}

void MXC_LP_DisableUSBWakeup(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_USB_WE;
}

int MXC_LP_ConfigDeepSleepClocks(uint32_t mask)
{
    if (!(mask & (MXC_F_GCR_PM_IBRO_PD | MXC_F_GCR_PM_IPO_PD | MXC_F_GCR_PM_ISO_PD |
                  MXC_F_GCR_PM_ERFO_PD))) {
        return E_BAD_PARAM;
    }

    MXC_GCR->pm |= mask;
    return E_NO_ERROR;
}

void MXC_LP_NFCOscBypassEnable(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_ERFO_BP;
}

void MXC_LP_NFCOscBypassDisable(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_ERFO_BP;
}

int MXC_LP_NFCOscBypassIsEnabled(void)
{
    return (MXC_GCR->pm & MXC_F_GCR_PM_ERFO_BP);
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

void MXC_LP_SysRam5LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM5LS_EN;
}

void MXC_LP_SysRam6LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_RAM6LS_EN;
}

void MXC_LP_ICacheXIPLightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ICCXIPLS_EN;
}

void MXC_LP_CryptoLightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_CRYPTOLS_EN;
}

void MXC_LP_USBFIFOLightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_USBLS_EN;
}

void MXC_LP_ROM0LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ROM0LS_EN;
}

void MXC_LP_ROM1LightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_ROM1LS_EN;
}

void MXC_LP_MAALightSleepEnable(void)
{
    MXC_GCR->memctrl |= MXC_F_GCR_MEMCTRL_MAALS_EN;
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

void MXC_LP_SysRam5LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM5LS_EN;
}

void MXC_LP_SysRam6LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_RAM6LS_EN;
}

void MXC_LP_ICacheXIPLightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ICCXIPLS_EN;
}

void MXC_LP_CryptoLightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_CRYPTOLS_EN;
}

void MXC_LP_USBFIFOLightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_USBLS_EN;
}

void MXC_LP_ROM0LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ROM0LS_EN;
}

void MXC_LP_ROM1LightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_ROM1LS_EN;
}

void MXC_LP_MAALightSleepDisable(void)
{
    MXC_GCR->memctrl &= ~MXC_F_GCR_MEMCTRL_MAALS_EN;
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

void MXC_LP_SysRam4Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM4;
}

void MXC_LP_SysRam4PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM4;
}

void MXC_LP_SysRam5Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM5;
}

void MXC_LP_SysRam5PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM5;
}

void MXC_LP_SysRam6Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_RAM6;
}

void MXC_LP_SysRam6PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_RAM6;
}

void MXC_LP_ICacheXIPShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ICCXIP;
}

void MXC_LP_ICacheXIPPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ICCXIP;
}

void MXC_LP_CryptoShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_CRYPTO;
}

void MXC_LP_CryptoPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_CRYPTO;
}

void MXC_LP_USBFIFOShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_USBFIFO;
}

void MXC_LP_USBFIFOPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_USBFIFO;
}

void MXC_LP_ROM0Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ROM0;
}

void MXC_LP_ROM0PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ROM0;
}

void MXC_LP_ROM1Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ROM1;
}

void MXC_LP_ROM1PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ROM1;
}
