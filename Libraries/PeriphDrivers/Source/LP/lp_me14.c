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

/***** Includes *****/
#include <stdio.h>
#include "lp.h"
#include "mxc_errors.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "tmr.h"
#include "mxc_delay.h"
#include "mxc_assert.h"
#include "simo.h"
#include "icc_reva.h"
#include "icc_reva_regs.h"

extern void Reset_Handler(void);
extern void Backup_Handler(void);

/***** Variables *****/
#define CTRL_POS (0)
#define ICSD_POS (1)
uint32_t icc0_state, icc1_state, clkcn_state;

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

void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
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

void __attribute__((deprecated("Causes SIMO soft start in wakeup"))) MXC_LP_FastWakeupEnable(void)
{
    // Deprecated due to issues with SIMO in wakeup.
    // MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_FWKM;
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

void MXC_LP_EnterBackgroundMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Set BACKGROUND bit and SLEEPDEEP bit */
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BCKGRND;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Go into Background mode and wait for an interrupt to wake the processor */
    __WFI();
}

void MXC_LP_EnterShutDownMode(void)
{
    MXC_GCR->pm &= ~MXC_F_GCR_PM_MODE;
    MXC_GCR->pm |= MXC_S_GCR_PM_MODE_SHUTDOWN;
    while (1) {}
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

void MXC_LP_EnableUSBWakeup(void)
{
    MXC_MCR->ctrl &= ~MXC_F_MCR_CTRL_USBSWEN_N;
    MXC_PWRSEQ->lppwen |= (MXC_F_PWRSEQ_LPPWEN_USBVBUSWKEN | MXC_F_PWRSEQ_LPPWEN_USBLSWKEN);
}

void MXC_LP_DisableUSBWakeup(void)
{
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_USBSWEN_N;
    MXC_PWRSEQ->lppwen &= ~(MXC_F_PWRSEQ_LPPWEN_USBVBUSWKEN | MXC_F_PWRSEQ_LPPWEN_USBLSWKEN);
}

int MXC_LP_FastWakeupIsEnabled(void)
{
    return (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_FWKM);
}

int MXC_LP_ConfigDeepSleepClocks(uint32_t mask)
{
    if (!(mask & (MXC_F_GCR_PM_HIRCPD | MXC_F_GCR_PM_HIRC96MPD | MXC_F_GCR_PM_HIRC8MPD |
                  MXC_F_GCR_PM_XTALPB))) {
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

/*
 *  Switch the system clock to the HIRC / 4
 *
 *  Enable the HIRC, set the divide ration to /4, and disable the 96 MHz oscillator.
 */
static void switchToHIRCD4(void)
{
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_PSC, MXC_S_GCR_CLKCN_PSC_DIV4);
    MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_HIRC);
    /* Disable unused clocks */
    while (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CKRDY)) {}
    /* Wait for the switch to occur */
    MXC_GCR->clkcn &= ~(MXC_F_GCR_CLKCN_HIRC96M_EN);
    SystemCoreClockUpdate();
}

static void save_preDeepSleep_state(void)
{
    /* Save ICC state */
    icc0_state = 0;
    if (MXC_ICC0->cache_ctrl & MXC_F_ICC_CACHE_CTRL_EN)
        icc0_state |= (1 << CTRL_POS);
    if (MXC_PWRSEQ->lpmemsd & MXC_F_PWRSEQ_LPMEMSD_ICACHESD)
        icc0_state |= (1 << ICSD_POS);
    icc1_state = 0;
    if (MXC_ICC1->cache_ctrl & MXC_F_ICC_CACHE_CTRL_EN)
        icc1_state |= (1 << CTRL_POS);
    if (MXC_PWRSEQ->lpmemsd & MXC_F_PWRSEQ_LPMEMSD_IC1SD)
        icc1_state |= (1 << ICSD_POS);

    /* Save CLKCN state */
    clkcn_state = MXC_GCR->clkcn;
}

static void restore_preDeepSleep_state(void)
{
    /* Restore CLKCN state */
    MXC_GCR->clkcn = clkcn_state;
    /* Wait for the switch to occur */
    while (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CKRDY)) {}
    SystemCoreClockUpdate();

    /* Restore ICC1 state */
    if (icc1_state & (1 << ICSD_POS)) {
        /* ICC power down. Do not restore Enable state. */
        MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_IC1SD;
    } else {
        /* ICC power up */
        MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_IC1SD;
        /* Enable */
        if (icc1_state & (1 << CTRL_POS))
            MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)MXC_ICC1);
    }

    /* Restore ICC0 state */
    if (icc0_state & (1 << ICSD_POS)) {
        /* ICC power down. Do not restore Enable state. */
        MXC_PWRSEQ->lpmemsd |= MXC_F_PWRSEQ_LPMEMSD_ICACHESD;
    } else {
        /* ICC power up */
        MXC_PWRSEQ->lpmemsd &= ~MXC_F_PWRSEQ_LPMEMSD_ICACHESD;
        /* Enable */
        if (icc0_state & (1 << CTRL_POS))
            MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)MXC_ICC0);
    }
}

void MXC_LP_EnterDeepSleepMode(void)
{
    save_preDeepSleep_state();

    MXC_ICC_Disable();
    MXC_LP_ICache0Shutdown();

    /* Shutdown unused power domains */
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BGOFF;

    switchToHIRCD4();

    /* Prevent SIMO soft start on wakeup */
    /* Set SIMO clock, VDDCSW, VregO_B voltage for DeepSleep */
    MXC_LP_FastWakeupDisable();

    /* Enable VDDCSWEN=1 prior to enter DEEPSLEEP */
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_VDDCSWEN;

    /* SIMO clock setup for deep sleep */
    *(volatile int *)0x40005434 = 1; /* SIMOCLKDIV [1:0] : 0=div1; 1=div8; 2=div1 3=div16 */
    /* BUCK_CLKSEL [25:24] : 0=8K; 1=16K; 2=30K; 3=RFU */
    *(volatile int *)0x40005440 = (*(volatile int *)0x40005440 & (~(0x3 << 24))) | (0x2 << 24);
    /* BUCK_CLKSEL_LP [7:6] : 0=8K; 1=16K; 2=30K; 3=RFU */
    *(volatile int *)0x40005444 = (*(volatile int *)0x40005444 & (~(0x3 << 6))) | (0x2 << 6);

    /* Wait for VCOREB to be ready */
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}

    /* Lower VregB to reduce power consumption */
    MXC_SIMO_SetVregO_B(900);

    /* Move VCORE switch to VCOREB (< VCOREA) */
    MXC_MCR->ctrl = (MXC_MCR->ctrl & ~(MXC_F_MCR_CTRL_VDDCSW)) | (0x2 << MXC_F_MCR_CTRL_VDDCSW_POS);

    /* Wait for VCOREA ready.  Should be ready already */
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC)) {}

    MXC_LP_ClearWakeStatus();

    /* Set SLEEPDEEP bit */
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_BCKGRND;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Go into Deepsleep mode and wait for an interrupt to wake the processor */
    __WFI();

    /* SIMO soft start workaround on wakeup */
    /* Check to see if VCOREA is ready on  */
    if (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC)) {
        /* Wait for VCOREB to be ready */
        while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}

        /* Move VCORE switch back to VCOREB */
        MXC_MCR->ctrl = (MXC_MCR->ctrl & ~(MXC_F_MCR_CTRL_VDDCSW)) |
                        (0x1 << MXC_F_MCR_CTRL_VDDCSW_POS);

        /* Raise the VCORE_B voltage */
        while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
        MXC_SIMO_SetVregO_B(1000);
        while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
    } else {
        if ((MXC_MCR->ctrl & MXC_F_MCR_CTRL_VDDCSW) == (1 << MXC_F_MCR_CTRL_VDDCSW_POS)) {
            /* Raise the VCORE_B voltage */
            while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
            MXC_SIMO_SetVregO_B(1000);
            while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
        }
    }
    restore_preDeepSleep_state();
}

void MXC_LP_EnterBackupMode(void *func(void))
{
    MXC_ICC_Disable();
    MXC_LP_ICache0Shutdown();

    /* Shutdown unused power domains */
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BGOFF;

    switchToHIRCD4();

    /* No RAM retention in BACKUP */
    MXC_LP_SetRAMRetention(MXC_S_PWRSEQ_LPCN_RAMRET_DIS);

    /* Disable VregB, VregD in BACKUP */
    MXC_LP_SIMOVregBPowerDown();
    MXC_LP_SIMOVregDPowerDown();

    /* Set SIMO clock, VDDCSW, VregO_C voltage for Backup */
    /* Prevent SIMO soft start on wakeup */
    MXC_LP_FastWakeupDisable();

    /* Enable VDDCSWEN=1 prior to enter BACKUP */
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_VDDCSWEN;

    /* SIMO softstart workaround: clock 8KHz/16 for BACKUP, 30KHz/1 in ACTIVE */
    *(volatile int *)0x40005434 = 3; /* SIMOCLKDIV [1:0] : 0=div1; 1=div8; 2=div1 3=div16 */
    /* BUCK_CLKSEL [25:24] : 0=8K; 1=16K; 2=30K; 3=RFU */
    *(volatile int *)0x40005440 = (*(volatile int *)0x40005440 & (~(0x3 << 24))) | (0x2 << 24);
    /* BUCK_CLKSEL_LP [7:6] : 0=8K; 1=16K; 2=30K; 3=RFU */
    *(volatile int *)0x40005444 = (*(volatile int *)0x40005444 & (~(0x3 << 6))) | (0x0 << 6);

    /* Move VCORE switch to VCOREB (< VCOREA) */
    MXC_MCR->ctrl = (MXC_MCR->ctrl & ~(MXC_F_MCR_CTRL_VDDCSW)) | (0x2 << MXC_F_MCR_CTRL_VDDCSW_POS);

    /* Lower VCOREA to save power */
    MXC_SIMO_SetVregO_C(850);

    /* Wait for VCOREA ready. */
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC)) {}

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
    while (1) {}
    // Should never reach this line - device will jump to backup vector on exit from background mode.
}
