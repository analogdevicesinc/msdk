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

/* **** Includes **** */
#include "lp.h"
#include "pwrseq_regs.h"
#include "mxc_errors.h"
#include "gcr_regs.h"
#include "usbhs_regs.h"
#include "mxc_sys.h"

/* **** Variable Declaration **** */
void MXC_LP_ClearWakeStatus(void)
{
    /* Write 1 to clear */
    MXC_PWRSEQ->gpio0_wk_fl = 0xFFFFFFFF;
    MXC_PWRSEQ->gpio1_wk_fl = 0xFFFFFFFF;
    MXC_PWRSEQ->gpio2_wk_fl = 0xFFFFFFFF;
    MXC_PWRSEQ->gpio3_wk_fl = 0xFFFFFFFF;
    MXC_PWRSEQ->usb_wk_fl = 0xFFFFFFFF;
}

/* ************************************************************************** */
void MXC_LP_EnableROM(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_ROMSD;
}

/* ************************************************************************** */
void MXC_LP_DisableROM(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_ROMSD;
}

/* ************************************************************************** */
void MXC_LP_EnableUSBFIFO(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD;
}

/* ************************************************************************** */
void MXC_LP_DisableUSBFIFO(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_USBFIFOSD;
}

/* ************************************************************************** */
void MXC_LP_EnableCryptoRAM(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD;
}

/* ************************************************************************** */
void MXC_LP_DisableCryptoRAM(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_CRYPTOSD;
}

/* ************************************************************************** */
void MXC_LP_EnableSCache(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SCACHESD;
}

/* ************************************************************************** */
void MXC_LP_DisableSCache(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SCACHESD;
}

/* ************************************************************************** */
void MXC_LP_EnableICacheXIP(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD;
}

/* ************************************************************************** */
void MXC_LP_DisableICacheXIP(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_ICACHEXIPSD;
}

/* ************************************************************************** */
void MXC_LP_EnableICache(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_ICACHESD;
}

/* ************************************************************************** */
void MXC_LP_DisableICache(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_ICACHESD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM6(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM6SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM6(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM6SD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM5(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM5SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM5(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM5SD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM4(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM4SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM4(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM4SD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM3(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM3SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM3(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM3SD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM2(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM2SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM2(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM2SD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM1(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM1SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM1(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM1SD;
}

/* ************************************************************************** */
void MXC_LP_EnableSRAM0(void)
{
    MXC_PWRSEQ->mem_pwr &= ~MXC_F_PWRSEQ_MEM_PWR_SRAM0SD;
}

/* ************************************************************************** */
void MXC_LP_DisableSRAM0(void)
{
    MXC_PWRSEQ->mem_pwr |= MXC_F_PWRSEQ_MEM_PWR_SRAM0SD;
}

/* ************************************************************************** */
void MXC_LP_EnableROMLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableROMLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_ROMLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableUSBFIFOLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_USBLS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_USBLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableUSBFIFOLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_USBLS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_USBLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableCryptoRAMLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

void MXC_LP_DisableCryptoRAMLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_CRYPTOLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSCacheLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSCacheLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SCACHELS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableICacheXIPLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableICacheXIPLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_ICACHEXIPLS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableICacheLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableICacheLightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_ICACHELS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM6LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM6LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM6LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM5LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM5LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM5LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM4LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP;

    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM4LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM4LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM3LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM3LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM3LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM2LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM2LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM2LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM1LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM1LS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM1LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM1LightSleep(void)
{
    if (ChipRevision > 0xA1) {
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM1LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableSysRAM0LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk |= MXC_S_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP;
    } else {
        MXC_GCR->mem_clk |= (MXC_S_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_DisableSysRAM0LightSleep(void)
{
    if (ChipRevision > 0xA1) {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP);
    } else {
        MXC_GCR->mem_clk &= ~(MXC_S_GCR_MEM_CLK_SYSRAM0LS_LIGHT_SLEEP >> 3);
        /* Writes to the register have a flaw in HW logic.  All writes to this register must be offset by 3 bits. */
    }
}

/* ************************************************************************** */
void MXC_LP_EnableUSBWakeup(mxc_lp_usb_event_t wu_evt)
{
    MXC_GCR->pmr |= MXC_F_GCR_PMR_USBWKEN;
    MXC_PWRSEQ->usb_wk_en |= wu_evt;
}

/* ************************************************************************** */
void MXC_LP_DisableUSBWakeup(mxc_lp_usb_event_t wu_evt)
{
    MXC_PWRSEQ->usb_wk_en &= ~wu_evt;
    if ((MXC_PWRSEQ->usb_wk_en &
         (MXC_F_PWRSEQ_USB_WK_EN_USBLSWKEN | MXC_F_PWRSEQ_USB_WK_EN_USBVBUSWKEN)) == 0) {
        MXC_GCR->pmr &= ~MXC_F_GCR_PMR_USBWKEN;
    }
}

/* ************************************************************************** */
void MXC_LP_EnableRTCAlarmWakeup(void)
{
    MXC_GCR->pmr |= MXC_F_GCR_PMR_RTCWKEN;
}

/* ************************************************************************** */
void MXC_LP_DisableRTCAlarmWakeup(void)
{
    MXC_GCR->pmr &= ~MXC_F_GCR_PMR_RTCWKEN;
}

/* ************************************************************************** */
void MXC_LP_EnableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
{
    MXC_GCR->pmr |= MXC_F_GCR_PMR_GPIOWKEN;
    switch (MXC_GPIO_GET_IDX(wu_pins->port)) {
    case 0:
        MXC_PWRSEQ->gpio0_wk_en |= wu_pins->mask;
        break;
    case 1:
        MXC_PWRSEQ->gpio1_wk_en |= wu_pins->mask;
        break;
    case 2:
        MXC_PWRSEQ->gpio2_wk_en |= wu_pins->mask;
        break;
    case 3:
        MXC_PWRSEQ->gpio3_wk_en |= wu_pins->mask;
        break;
    }
}

/* ************************************************************************** */
void MXC_LP_DisableGPIOWakeup(mxc_gpio_cfg_t *wu_pins)
{
    switch (MXC_GPIO_GET_IDX(wu_pins->port)) {
    case 0:
        MXC_PWRSEQ->gpio0_wk_en &= ~wu_pins->mask;
        break;
    case 1:
        MXC_PWRSEQ->gpio1_wk_en &= ~wu_pins->mask;
        break;
    case 2:
        MXC_PWRSEQ->gpio2_wk_en &= ~wu_pins->mask;
        break;
    case 3:
        MXC_PWRSEQ->gpio3_wk_en &= ~wu_pins->mask;
        break;
    }

    if ((MXC_PWRSEQ->gpio0_wk_en == 0) && (MXC_PWRSEQ->gpio1_wk_en == 0) &&
        (MXC_PWRSEQ->gpio2_wk_en == 0) && (MXC_PWRSEQ->gpio3_wk_en == 0)) {
        MXC_GCR->pmr &= ~MXC_F_GCR_PMR_GPIOWKEN;
    }
}

/* ************************************************************************** */
void MXC_LP_EnterSleepMode(void)
{
    MXC_LP_ClearWakeStatus();

    /* Clear SLEEPDEEP bit */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Go into Sleep mode and wait for an interrupt to wake the processor */
    __WFI();
}

/* ************************************************************************** */
void MXC_LP_EnterBackgroundMode(void)
{
    int restoreSysClock = 0;
    int restoreHBClock = 0;
    int restoreSCacheClock = 0;
    int restoreSPIXClock = 0;
    uint32_t lpcn = 0;
    MXC_LP_ClearWakeStatus();

    //make sure power monitors are in reset mode.
    lpcn = MXC_PWRSEQ->ctrl &
           ~(MXC_F_PWRSEQ_CTRL_VDDIOHMD | MXC_F_PWRSEQ_CTRL_VCOREMD | MXC_F_PWRSEQ_CTRL_PORVDDIOMD |
             MXC_F_PWRSEQ_CTRL_VDDBMD | MXC_F_PWRSEQ_CTRL_VRTCMD | MXC_F_PWRSEQ_CTRL_PORVDDIOHMD |
             MXC_F_PWRSEQ_CTRL_VDDAMD | MXC_F_PWRSEQ_CTRL_VDDIOMD);

    /* Set background mode enable. */
    lpcn |= MXC_F_PWRSEQ_CTRL_BKGRND;
    MXC_PWRSEQ->ctrl = lpcn;

    /* Set SLEEPDEEP bit */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Cannot enter BACKGROUND mode with a system clock faster than 2x of crypto clock. */
    /* Divide down if necessary. */
    if ((MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_SYSOSC_SEL) ==
        MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96) {
        if ((MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE) ==
            MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV1) {
            restoreSysClock = 1;
            MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE,
                         MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV2);
        }
    }
    /* These clocks need to be on during BACKGROUND mode. */
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_HBC)) {
        restoreHBClock = 1;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HBC);
    }
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_SCACHE)) {
        restoreSCacheClock = 1;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    }
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_SPIXIPR)) {
        restoreSPIXClock = 1;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
    }

    /* Go into Sleep mode and wait for an interrupt to wake the processor */
    __WFI();

    /* Restore to the original clock settings. */
    if (restoreSysClock) {
        MXC_SETFIELD(MXC_GCR->clk_ctrl, MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE,
                     MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV1);
    }
    if (restoreHBClock) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_HBC);
    }
    if (restoreSCacheClock) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    }
    if (restoreSPIXClock) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
    }
}

/* ************************************************************************** */
void MXC_LP_EnterDeepSleepMode(void)
{
    int restoreHIRC = 0;
    int restoreHBClock = 0;
    int restoreSCacheClock = 0;
    int restoreSPIXClock = 0;
    uint32_t lpcn = 0;

    MXC_LP_ClearWakeStatus();

    //make sure power monitors are in reset mode.
    lpcn = MXC_PWRSEQ->ctrl &
           ~(MXC_F_PWRSEQ_CTRL_VDDIOHMD | MXC_F_PWRSEQ_CTRL_VCOREMD | MXC_F_PWRSEQ_CTRL_PORVDDIOMD |
             MXC_F_PWRSEQ_CTRL_VDDBMD | MXC_F_PWRSEQ_CTRL_VRTCMD | MXC_F_PWRSEQ_CTRL_PORVDDIOHMD |
             MXC_F_PWRSEQ_CTRL_VDDAMD | MXC_F_PWRSEQ_CTRL_VDDIOMD);

    /* Clear background mode enable, enable power failure monitor, Bandgap is always on. */
    lpcn &= ~(MXC_F_PWRSEQ_CTRL_BKGRND | MXC_F_PWRSEQ_CTRL_BGOFF | MXC_F_PWRSEQ_CTRL_PORVCOREMD);
    MXC_PWRSEQ->ctrl = lpcn;

    /* Set SLEEPDEEP bit */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Enable the clocks that must be turned on during DEEPSLEEP */
    if ((MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CRYPTO_EN) == 0) {
        restoreHIRC = 1;
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_CRYPTO_EN;
    }
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_HBC)) {
        restoreHBClock = 1;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HBC);
    }
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_SCACHE)) {
        restoreSCacheClock = 1;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    }
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_SPIXIPR)) {
        restoreSPIXClock = 1;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
    }

    /* Go into Deepsleep mode and wait for an interrupt to wake the processor */
    __WFI();

    /* Disable the clocks that were disabled prior to calling this function. */
    if (restoreHIRC) {
        MXC_GCR->clk_ctrl &= ~MXC_F_GCR_CLK_CTRL_CRYPTO_EN;
    }
    if (restoreHBClock) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_HBC);
    }
    if (restoreSCacheClock) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    }
    if (restoreSPIXClock) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
    }
}

/* ************************************************************************** */
void MXC_LP_EnterBackupMode(void)
{
    MXC_LP_ClearWakeStatus();

    MXC_GCR->pmr &= ~MXC_F_GCR_PMR_MODE;
    MXC_GCR->pmr |= MXC_S_GCR_PMR_MODE_BACKUP;
    while (1) {}
}

/* ************************************************************************** */
void MXC_LP_USBClearPONRST(void)
{
    // This register is used during the power-on stage or used as a global reset
    // for the USB block. For this case, the PONRST is used as a global reset
    // and setting this register to 0 will force the USB block to its initial
    // state - where the operating current is at its minimum.
    MXC_USBHS->m31_phy_ponrst = 0;
}

/* ************************************************************************** */
void MXC_LP_USBSetPONRST(void)
{
    // Re-enables the clock generator and the USB block if PONRST was used as
    // a global reset for operating in low power modes.
    MXC_USBHS->m31_phy_ponrst = 1;
}
