/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

/***** Includes *****/
#include "lp.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "tmr.h"
#include <stdio.h>

extern void Reset_Handler(void);
extern void Backup_Handler(void);

/***** Functions *****/
void MXC_LP_ClearWakeStatus(void)
{
    /* Write 1 to clear */
    MXC_PWRSEQ->lpwkst0 = 0xFFFFFFFF;
    MXC_PWRSEQ->lpwkst1 = 0xFFFFFFFF;
    MXC_PWRSEQ->lppwst = 0xFFFFFFFF;
}

void MXC_LP_EnableRTCAlarmWakeup(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_RTCWKEN;
}

void MXC_LP_DisableRTCAlarmWakeup(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_RTCWKEN;
}

void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t* wu_pins)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_GPIOWKEN;
    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 |= wu_pins->mask;
        break;
    case MXC_GPIO_PORT_1:
        MXC_PWRSEQ->lpwken1 |= wu_pins->mask;
    }
}

void MXC_LP_DisableGPIOWakeup(mxc_gpio_cfg_t* wu_pins)
{
    switch (1 << MXC_GPIO_GET_IDX(wu_pins->port)) {
    case MXC_GPIO_PORT_0:
        MXC_PWRSEQ->lpwken0 &= ~wu_pins->mask;
        break;
    case MXC_GPIO_PORT_1:
        MXC_PWRSEQ->lpwken1 &= ~wu_pins->mask;
    }

    if (MXC_PWRSEQ->lpwken1 == 0 && MXC_PWRSEQ->lpwken0 == 0) {
        MXC_GCR->pm &= ~MXC_F_GCR_PM_GPIOWKEN;
    }
}

void MXC_LP_EnableWUTAlarmWakeup(void)
{
    MXC_GCR->pm |= MXC_F_GCR_PM_WUTWKEN;
}

void MXC_LP_DisableWUTAlarmWakeup(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_WUTWKEN;
}

void MXC_LP_SysRam0LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SYSRAM0LS;
}

void MXC_LP_SysRam1LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SYSRAM1LS;
}

void MXC_LP_SysRam2LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SYSRAM2LS;
}

void MXC_LP_SysRam3LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SYSRAM3LS;
}

void MXC_LP_SysRam4LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SYSRAM4LS;
}

void MXC_LP_SysRam5LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SYSRAM5LS;
}

void MXC_LP_SysRam0Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRAM0SD;
}

void MXC_LP_SysRam0PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRAM0SD;
}

void MXC_LP_SysRam1Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRAM1SD;
}

void MXC_LP_SysRam1PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRAM1SD;
}

void MXC_LP_SysRam2Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRAM2SD;
}

void MXC_LP_SysRam2PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRAM2SD;
}

void MXC_LP_SysRam3Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRAM3SD;
}

void MXC_LP_SysRam3PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRAM3SD;
}

void MXC_LP_SysRam4Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRAM4SD;
}

void MXC_LP_SysRam4PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRAM4SD;
}

void MXC_LP_SysRam5Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRAM5SD;
}

void MXC_LP_SysRam5PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRAM5SD;
}

void MXC_LP_ICache0Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ICACHESD;
}

void MXC_LP_ICache0PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ICACHESD;
}

void MXC_LP_ICacheXIPShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ICACHEXIPSD;
}

void MXC_LP_ICacheXIPPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ICACHEXIPSD;
}

void MXC_LP_CryptoShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_CRYPTOSD;
}

void MXC_LP_CryptoPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_CRYPTOSD;
}

void MXC_LP_USBFIFOShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_USBFIFOSD;
}

void MXC_LP_USBFIFOPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_USBFIFOSD;
}

void MXC_LP_ROM0Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ROMSD;
}

void MXC_LP_ROM0PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ROMSD;
}

void MXC_LP_ROM1Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ROM1SD;
}

void MXC_LP_ROM1PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ROM1SD;
}

void MXC_LP_ICache1Shutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_IC1SD;
}

void MXC_LP_ICache1PowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_IC1SD;
}

void MXC_LP_USBSWLPDisable(void)
{
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_USBSWEN_N;
}

void MXC_LP_USBSWLPEnable(void)
{
    MXC_MCR->ctrl &= ~MXC_F_MCR_CTRL_USBSWEN_N;
}

void MXC_LP_VDD2PowerDown(void)
{
    MXC_PWRSEQ->lpvddpd |= MXC_F_PWRSEQ_LPVDDPD_VDD2PD;
}

void MXC_LP_VDD2PowerUp(void)
{
    MXC_PWRSEQ->lpvddpd &= ~MXC_F_PWRSEQ_LPVDDPD_VDD2PD;
}

void MXC_LP_VDD3PowerDown(void)
{
    MXC_PWRSEQ->lpvddpd |= MXC_F_PWRSEQ_LPVDDPD_VDD3PD;
}

void MXC_LP_VDD3PowerUp(void)
{
    MXC_PWRSEQ->lpvddpd &= ~MXC_F_PWRSEQ_LPVDDPD_VDD3PD;
}

void MXC_LP_VDD4PowerDown(void)
{
    MXC_PWRSEQ->lpvddpd |= MXC_F_PWRSEQ_LPVDDPD_VDD4PD;
}

void MXC_LP_VDD4PowerUp(void)
{
    MXC_PWRSEQ->lpvddpd &= ~MXC_F_PWRSEQ_LPVDDPD_VDD4PD;
}

void MXC_LP_VDD5PowerDown(void)
{
    MXC_PWRSEQ->lpvddpd |= MXC_F_PWRSEQ_LPVDDPD_VDD5PD;
}

void MXC_LP_VDD5PowerUp(void)
{
    MXC_PWRSEQ->lpvddpd &= ~MXC_F_PWRSEQ_LPVDDPD_VDD5PD;
}

void MXC_LP_SIMOVregBPowerDown(void)
{
    MXC_PWRSEQ->lpvddpd |= MXC_F_PWRSEQ_LPVDDPD_VREGOBPD;
}

void MXC_LP_SIMOVregBPowerUp(void)
{
    MXC_PWRSEQ->lpvddpd &= ~MXC_F_PWRSEQ_LPVDDPD_VREGOBPD;
}

void MXC_LP_SIMOVregDPowerDown(void)
{
    MXC_PWRSEQ->lpvddpd |= MXC_F_PWRSEQ_LPVDDPD_VREGODPD;
}

void MXC_LP_SIMOVregDPowerUp(void)
{
    MXC_PWRSEQ->lpvddpd &= ~MXC_F_PWRSEQ_LPVDDPD_VREGODPD;
}

void MXC_LP_FastWakeupEnable(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_FWKM;
}

void MXC_LP_FastWakeupDisable(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_FWKM;
}

void MXC_LP_SetRAMRetention(mxc_ram_retained_t ramRetained)
{
    MXC_SETFIELD(MXC_PWRSEQ->lpcn, MXC_F_PWRSEQ_LPCN_RAMRET, ramRetained);
}

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
    MXC_LP_ClearWakeStatus();

    /* Set SLEEPDEEP bit */
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_BCKGRND;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Go into Deepsleep mode and wait for an interrupt to wake the processor */
    __WFI();
}

void MXC_LP_EnterBackgroundMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Set BACKGROUND bit and SLEEPDEEP bit */
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BCKGRND;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Go into Background mode and wait for an interrupt to wake the processor */
    __WFI();
}

void MXC_LP_EnterBackupMode(void* func(void))
{
    MXC_LP_ClearWakeStatus();

    MXC_PWRSEQ->buretvec = (uint32_t)(&Backup_Handler) | 1;
    if (func == NULL) {
        MXC_PWRSEQ->buaod = (uint32_t)(&Reset_Handler) | 1;
    } else {
        MXC_PWRSEQ->buaod = (uint32_t)(&func) | 1;
    }

    // Enable the VDDCSW to ensure we have enough power to start
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_VDDCSWEN;

    // Enable backup mode
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_BACKUP;
    while (1) { }
    // Should never reach this line - device will jump to backup vector on exit from background
    // mode.
}

void MXC_LP_EnterShutdownMode(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_SHUTDOWN;
    while (1) { }
    // Should never reach this line - device will reset on exit from shutdown mode.
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void MXC_LP_BandgapOn(void)
{
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_BGOFF;
}

void MXC_LP_BandgapOff(void)
{
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BGOFF;
}

int MXC_LP_BandgapIsOn(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_BGOFF);
}

int MXC_LP_FastWakeupIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_FWKM);
}

int MXC_LP_ConfigDeepSleepClocks(uint32_t mask)
{
    if (!(mask
            & (MXC_F_GCR_PM_HIRCPD | MXC_F_GCR_PM_HIRC96MPD | MXC_F_GCR_PM_HIRC8MPD
                | MXC_F_GCR_PM_XTALPB))) {
        return E_BAD_PARAM;
    }

    MXC_GCR->pm |= mask;
    return E_NO_ERROR;
}

void MXC_LP_ICache0LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_ICACHELS;
}

void MXC_LP_ICache1LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_ICACHE1LS;
}

void MXC_LP_ICacheXIPLightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_ICACHEXIPLS;
}

void MXC_LP_SRCCLightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_SCACHELS;
}

void MXC_LP_CryptoLightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_CRYPTOLS;
}

void MXC_LP_USBFIFOLightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_USBLS;
}

void MXC_LP_ROM0LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_ROM0LS;
}

void MXC_LP_ROM1LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_ROM1LS;
}

void MXC_LP_SysRam0LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SYSRAM0LS;
}

void MXC_LP_SysRam1LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SYSRAM1LS;
}

void MXC_LP_SysRam2LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SYSRAM2LS;
}

void MXC_LP_SysRam3LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SYSRAM3LS;
}

void MXC_LP_SysRam4LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SYSRAM4LS;
}

void MXC_LP_SysRam5LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SYSRAM5LS;
}

void MXC_LP_SRCCLightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_SCACHELS;
}

void MXC_LP_CryptoLightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_CRYPTOLS;
}

void MXC_LP_USBFIFOLightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_USBLS;
}

void MXC_LP_ROM0LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_ROM0LS;
}

void MXC_LP_ROM1LightSleepDisable(void)
{
    MXC_GCR->memckcn &= ~MXC_F_GCR_MEMCKCN_ROM1LS;
}

void MXC_LP_SRCCShutdown(void)
{
    MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_SRCCSD;
}

void MXC_LP_SRCCPowerUp(void)
{
    MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_SRCCSD;
}

void MXC_LP_ICache0LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_ICACHELS;
}

void MXC_LP_ICache1LightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_ICACHE1LS;
}

void MXC_LP_ICacheXIPLightSleepEnable(void)
{
    MXC_GCR->memckcn |= MXC_F_GCR_MEMCKCN_ICACHEXIPLS;
}
