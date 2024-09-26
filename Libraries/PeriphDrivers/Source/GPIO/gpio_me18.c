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
#include <stddef.h>
#include "gpio.h"
#include "gpio_common.h"
#include "gpio_reva.h"
// #include "lpgcr_regs.h"
#include "mcr_regs.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "pwrseq_regs.h"

/* **** Definitions **** */
#define GPIO4_PIN_MASK 0x00000003
#define GPIO4_RESET_MASK 0xFFFFFF77
#define GPIO4_OUTEN_MASK(mask)                                \
    (((mask & (1 << 0)) << MXC_F_MCR_GPIO4_CTRL_P40_OE_POS) | \
     ((mask & (1 << 1)) << (MXC_F_MCR_GPIO4_CTRL_P41_OE_POS - 1)))
#define GPIO4_PULLDIS_MASK(mask)                              \
    (((mask & (1 << 0)) << MXC_F_MCR_GPIO4_CTRL_P40_PE_POS) | \
     ((mask & (1 << 1)) << (MXC_F_MCR_GPIO4_CTRL_P41_PE_POS - 1)))
#define GPIO4_DATAOUT_MASK(mask)                              \
    (((mask & (1 << 0)) << MXC_F_MCR_GPIO4_CTRL_P40_DO_POS) | \
     ((mask & (1 << 1)) << (MXC_F_MCR_GPIO4_CTRL_P41_DO_POS - 1)))
#define GPIO4_DATAOUT_GET_MASK(mask)                                                             \
    ((((MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P40_DO) >> MXC_F_MCR_GPIO4_CTRL_P40_DO_POS) | \
      ((MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P41_DO) >>                                    \
       (MXC_F_MCR_GPIO4_CTRL_P41_DO_POS - 1))) &                                                 \
     mask)
#define GPIO4_DATAIN_MASK(mask)                                                                  \
    ((((MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P40_IN) >> MXC_F_MCR_GPIO4_CTRL_P40_IN_POS) | \
      ((MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P41_IN) >>                                    \
       (MXC_F_MCR_GPIO4_CTRL_P41_IN_POS - 1))) &                                                 \
     mask)
#define GPIO4_AFEN_MASK(mask)                                  \
    (((mask & (1 << 0)) << MXC_F_MCR_OUTEN_PDOWN_OUT_EN_POS) | \
     ((mask & (1 << 1)) >> (MXC_F_MCR_OUTEN_SQWOUT_EN_POS + 1)))

/* **** Globals **** */

/* **** Functions **** */
int MXC_GPIO_Init(uint32_t portmask)
{
    if (portmask & 0x1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    }

    if (portmask & 0x2) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    }

    if (portmask & 0x4) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    }

    if (portmask & 0x8) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO3);
    }

    return MXC_GPIO_Common_Init(portmask);
}

int MXC_GPIO_Shutdown(uint32_t portmask)
{
    if (portmask & 0x1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    }

    if (portmask & 0x2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    }

    if (portmask & 0x4) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    }

    if (portmask & 0x8) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO3);
    }

    return E_NO_ERROR;
}

int MXC_GPIO_Reset(uint32_t portmask)
{
    if (portmask & 0x1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_GPIO0);
    }

    if (portmask & 0x2) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_GPIO1);
    }

    if (portmask & 0x4) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_GPIO2);
    }

    if (portmask & 0x8) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO3);
    }

    if (portmask & 0x10) {
        MXC_MCR->gpio4_ctrl &= ~GPIO4_RESET_MASK;
    }

    return E_NO_ERROR;
}

int MXC_GPIO_Config(const mxc_gpio_cfg_t *cfg)
{
    int port, error;
    mxc_gpio_regs_t *gpio = cfg->port;

    port = MXC_GPIO_GET_IDX(cfg->port);
    if (port == -1) {
        return E_BAD_PARAM;
    }

    // Initialize callback function pointers
    MXC_GPIO_Init(1 << port);

    if (MXC_GPIO_GetConfigLock() == MXC_GPIO_CONFIG_LOCKED) {
        // Configuration is locked.  Ignore any attempts to change it.
        return E_NO_ERROR;
    }

    // Configure the vssel
    if (port < 4) {
        error = MXC_GPIO_SetVSSEL(gpio, cfg->vssel, cfg->mask);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    // Configure alternate function
    if (port < 4) {
        error = MXC_GPIO_RevA_SetAF((mxc_gpio_reva_regs_t *)gpio, cfg->func, cfg->mask);
    } else {
        error = E_NO_ERROR;

        switch (cfg->func) {
        case MXC_GPIO_FUNC_ALT1:
            // Set GPIO(s) to AF1
            MXC_MCR->gpio4_ctrl |= GPIO4_OUTEN_MASK(cfg->mask);
            MXC_MCR->outen |= GPIO4_AFEN_MASK(cfg->mask);
            break;

        case MXC_GPIO_FUNC_OUT:
            // Set GPIO(s) to output mode
            MXC_MCR->gpio4_ctrl |= GPIO4_OUTEN_MASK(cfg->mask);
            MXC_MCR->outen &= ~GPIO4_AFEN_MASK(cfg->mask);
            break;

        case MXC_GPIO_FUNC_IN:
            // Set GPIO(s) to input mode
            MXC_MCR->gpio4_ctrl &= ~GPIO4_OUTEN_MASK(cfg->mask);
            MXC_MCR->outen &= ~GPIO4_AFEN_MASK(cfg->mask);
            break;

        default:
            error = E_NOT_SUPPORTED;
            break;
        }
    }

    if (error != E_NO_ERROR) {
        return error;
    }

    // Configure the pad
    if (port < 4) {
        switch (cfg->pad) {
        case MXC_GPIO_PAD_NONE:
            gpio->padctrl0 &= ~cfg->mask;
            gpio->padctrl1 &= ~cfg->mask;
            break;

        // Note: for "ps" field set 1 for weak and 0 for strong.
        // As of 8-28-2024 most UG tables have this flipped the wrong way
        case MXC_GPIO_PAD_WEAK_PULL_UP:
            gpio->padctrl0 |= cfg->mask;
            gpio->padctrl1 &= ~cfg->mask;
            gpio->ps |= cfg->mask;
            break;

        case MXC_GPIO_PAD_PULL_UP:
            gpio->padctrl0 |= cfg->mask;
            gpio->padctrl1 &= ~cfg->mask;
            gpio->ps &= ~cfg->mask;
            break;

        case MXC_GPIO_PAD_WEAK_PULL_DOWN:
            gpio->padctrl0 &= ~cfg->mask;
            gpio->padctrl1 |= cfg->mask;
            gpio->ps |= cfg->mask;
            break;

        case MXC_GPIO_PAD_PULL_DOWN:
            gpio->padctrl0 &= ~cfg->mask;
            gpio->padctrl1 |= cfg->mask;
            gpio->ps &= ~cfg->mask;
            break;

        default:
            return E_BAD_PARAM;
        }
    } else {
        switch (cfg->pad) {
        case MXC_GPIO_PAD_NONE:
            // Disable pull-up/down resistors
            MXC_MCR->gpio4_ctrl |= GPIO4_PULLDIS_MASK(cfg->mask);
            break;

        case MXC_GPIO_PAD_WEAK_PULL_UP:
        case MXC_GPIO_PAD_PULL_UP:
            // Set to input mode, enable pull-up/down resistors
            MXC_MCR->gpio4_ctrl &= ~(GPIO4_OUTEN_MASK(cfg->mask) | GPIO4_PULLDIS_MASK(cfg->mask));

            // Set to pullup mode
            MXC_MCR->gpio4_ctrl |= GPIO4_DATAOUT_MASK(cfg->mask);
            break;

        case MXC_GPIO_PAD_WEAK_PULL_DOWN:
        case MXC_GPIO_PAD_PULL_DOWN:
            // Set to input mode, enable pull-up/down resistors, set to pulldown mode
            MXC_MCR->gpio4_ctrl &= ~(GPIO4_OUTEN_MASK(cfg->mask) | GPIO4_PULLDIS_MASK(cfg->mask) |
                                     GPIO4_DATAOUT_MASK(cfg->mask));
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    // Configure the drive strength
    if (cfg->func == MXC_GPIO_FUNC_IN) {
        return E_NO_ERROR;
    } else {
        return MXC_GPIO_SetDriveStrength(gpio, cfg->drvstr, cfg->mask);
    }
}

/* ************************************************************************** */
uint32_t MXC_GPIO_InGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        uint32_t gpio40 = (MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P40_IN) >>
                          MXC_F_MCR_GPIO4_CTRL_P40_IN_POS;
        uint32_t gpio41 = (MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P41_IN) >>
                          (MXC_F_MCR_GPIO4_CTRL_P41_IN_POS - 1);
        return ((gpio40 | gpio41) & mask);
    }

    return MXC_GPIO_RevA_InGet((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutSet(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        MXC_MCR->gpio4_ctrl |= GPIO4_DATAOUT_MASK(mask);
        return;
    }

    MXC_GPIO_RevA_OutSet((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutClr(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        MXC_MCR->gpio4_ctrl &= ~GPIO4_DATAOUT_MASK(mask);
        return;
    }

    MXC_GPIO_RevA_OutClr((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_OutGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        uint32_t gpio40 = (MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P40_DO) >>
                          MXC_F_MCR_GPIO4_CTRL_P40_DO_POS;
        uint32_t gpio41 = (MXC_MCR->gpio4_ctrl & MXC_F_MCR_GPIO4_CTRL_P41_DO) >>
                          (MXC_F_MCR_GPIO4_CTRL_P41_DO_POS - 1);
        return ((gpio40 | gpio41) & mask);
    }

    return MXC_GPIO_RevA_OutGet((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutPut(mxc_gpio_regs_t *port, uint32_t mask, uint32_t val)
{
    if (port == MXC_GPIO4) {
        uint32_t gpio4_cp = MXC_MCR->gpio4_ctrl;

        MXC_MCR->gpio4_ctrl = (gpio4_cp & ~GPIO4_DATAOUT_MASK(mask)) |
                              GPIO4_DATAOUT_MASK((mask & val));
        return;
    }

    MXC_GPIO_RevA_OutPut((mxc_gpio_reva_regs_t *)port, mask, val);
}

/* ************************************************************************** */
void MXC_GPIO_OutToggle(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        MXC_MCR->gpio4_ctrl ^= GPIO4_DATAOUT_MASK(mask);
        return;
    }

    MXC_GPIO_RevA_OutToggle((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
int MXC_GPIO_IntConfig(const mxc_gpio_cfg_t *cfg, mxc_gpio_int_pol_t pol)
{
    if (cfg->port == MXC_GPIO4) {
        if (pol != MXC_GPIO_INT_BOTH) {
            return E_NOT_SUPPORTED;
        }

        return E_NO_ERROR;
    }

    return MXC_GPIO_RevA_IntConfig(cfg, pol);
}

/* ************************************************************************** */
void MXC_GPIO_EnableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        MXC_PWRSEQ->lpwken4 |= (mask & GPIO4_PIN_MASK);
        return;
    }

    MXC_GPIO_RevA_EnableInt((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_DisableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        MXC_PWRSEQ->lpwken4 &= ~(mask & GPIO4_PIN_MASK);
        return;
    }

    MXC_GPIO_RevA_DisableInt((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_RegisterCallback(const mxc_gpio_cfg_t *cfg, mxc_gpio_callback_fn func, void *cbdata)
{
    MXC_GPIO_Common_RegisterCallback(cfg, func, cbdata);
}

/* ************************************************************************** */
void MXC_GPIO_Handler(unsigned int port)
{
    MXC_GPIO_Common_Handler(port);
}

/* ************************************************************************** */
void MXC_GPIO_ClearFlags(mxc_gpio_regs_t *port, uint32_t flags)
{
    if (port == MXC_GPIO4) {
        MXC_PWRSEQ->lpwkst4 = flags & GPIO4_PIN_MASK;
        return;
    }

    MXC_GPIO_RevA_ClearFlags((mxc_gpio_reva_regs_t *)port, flags);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_GetFlags(mxc_gpio_regs_t *port)
{
    if (port == MXC_GPIO4) {
        return MXC_PWRSEQ->lpwkst4 & GPIO4_PIN_MASK;
    }

    return MXC_GPIO_RevA_GetFlags((mxc_gpio_reva_regs_t *)port);
}

/* ************************************************************************** */
int MXC_GPIO_SetVSSEL(mxc_gpio_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        return E_NOT_SUPPORTED;
    }

    return MXC_GPIO_RevA_SetVSSEL((mxc_gpio_reva_regs_t *)port, vssel, mask);
}

/* ************************************************************************** */
void MXC_GPIO_SetWakeEn(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        return;
    }

    MXC_GPIO_RevA_SetWakeEn((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_ClearWakeEn(mxc_gpio_regs_t *port, uint32_t mask)
{
    if (port == MXC_GPIO4) {
        return;
    }

    MXC_GPIO_RevA_ClearWakeEn((mxc_gpio_reva_regs_t *)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_GetWakeEn(mxc_gpio_regs_t *port)
{
    // GPIO Port 4 not a wakeup source. Return 0.
    //  Can't return error code (negative values) due to return type.
    if (port == MXC_GPIO4) {
        return 0;
    }

    return MXC_GPIO_RevA_GetWakeEn((mxc_gpio_reva_regs_t *)port);
}

/* ************************************************************************** */
int MXC_GPIO_SetDriveStrength(mxc_gpio_regs_t *port, mxc_gpio_drvstr_t drvstr, uint32_t mask)
{
    return MXC_GPIO_RevA_SetDriveStrength((mxc_gpio_reva_regs_t *)port, drvstr, mask);
}

/* ************************************************************************** */
void MXC_GPIO_SetConfigLock(mxc_gpio_config_lock_t locked)
{
    MXC_GPIO_Common_SetConfigLock(locked);
}

/* ************************************************************************** */
mxc_gpio_config_lock_t MXC_GPIO_GetConfigLock(void)
{
    return MXC_GPIO_Common_GetConfigLock();
}
