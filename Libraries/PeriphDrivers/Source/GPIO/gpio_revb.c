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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "gpio.h"
#include "gpio_revb.h"
#include "gpio_common.h"

/* **** Functions **** */
// NOTE(JC): This function doesn't actually seem to be used anywhere.  The MEXX parts re-implement this...
// TODO(JC): Consolidate to actually use this (would help with code repetition) but psMask seems dubious
int MXC_GPIO_RevB_Config(const mxc_gpio_cfg_t *cfg, uint8_t psMask)
{
    mxc_gpio_regs_t *gpio = cfg->port;

    if (MXC_GPIO_GetConfigLock() == MXC_GPIO_CONFIG_LOCKED) {
        // Configuration is locked.  Ignore any attempts to change it.
        return E_NO_ERROR;
    }

    // Set the GPIO type
    switch (cfg->func) {
    case MXC_GPIO_FUNC_IN:
        gpio->outen_clr = cfg->mask;
        gpio->en0_set = cfg->mask;
        gpio->en1_clr = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_OUT:
        gpio->outen_set = cfg->mask;
        gpio->en0_set = cfg->mask;
        gpio->en1_clr = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_ALT1:
        gpio->en0_clr = cfg->mask;
        gpio->en1_clr = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_ALT2:
        gpio->en0_clr = cfg->mask;
        gpio->en1_set = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    case MXC_GPIO_FUNC_ALT3:
        gpio->en0_set = cfg->mask;
        gpio->en1_set = cfg->mask;
        gpio->en2_clr = cfg->mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    // Configure the pad
    // Note: for "ps" field set 1 for weak and 0 for strong.
    // As of 8-28-2024 most UG tables have this flipped the wrong way
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        gpio->ps &= ~cfg->mask;
        break;

    case MXC_GPIO_PAD_PULL_UP:
        gpio->padctrl0 |= cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        gpio->ps &= ~cfg->mask;
        break;

    case MXC_GPIO_PAD_PULL_DOWN:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 |= cfg->mask;
        gpio->ps |= cfg->mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    // Configure the vssel
    MXC_GPIO_SetVSSEL(gpio, cfg->vssel, cfg->mask);

    return E_NO_ERROR;
}

uint32_t MXC_GPIO_RevB_InGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    return (port->in & mask);
}

void MXC_GPIO_RevB_OutSet(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->out_set = mask;
}

void MXC_GPIO_RevB_OutClr(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->out_clr = mask;
}

uint32_t MXC_GPIO_RevB_OutGet(mxc_gpio_regs_t *port, uint32_t mask)
{
    return (port->out & mask);
}

void MXC_GPIO_RevB_OutPut(mxc_gpio_regs_t *port, uint32_t mask, uint32_t val)
{
    port->out = (port->out & ~mask) | (val & mask);
}

void MXC_GPIO_RevB_OutToggle(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->out ^= mask;
}

int MXC_GPIO_RevB_IntConfig(const mxc_gpio_cfg_t *cfg, mxc_gpio_int_pol_t pol)
{
    mxc_gpio_regs_t *gpio = cfg->port;

    switch (pol) {
    case MXC_GPIO_INT_HIGH:
        gpio->intpol &= ~cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode &= ~cfg->mask;
        break;

    case MXC_GPIO_INT_FALLING: /* MXC_GPIO_INT_HIGH */
        gpio->intpol &= ~cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    case MXC_GPIO_INT_LOW: /* MXC_GPIO_INT_LOW */
        gpio->intpol |= cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode &= ~cfg->mask;
        break;

    case MXC_GPIO_INT_RISING: /* MXC_GPIO_INT_LOW */
        gpio->intpol |= cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    case MXC_GPIO_INT_BOTH:
        gpio->dualedge |= cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

void MXC_GPIO_RevB_EnableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->inten_set = mask;
}

void MXC_GPIO_RevB_DisableInt(mxc_gpio_regs_t *port, uint32_t mask)
{
    port->inten_clr = mask;
}

void MXC_GPIO_RevB_ClearFlags(mxc_gpio_regs_t *port, uint32_t flags)
{
    port->intfl_clr = flags;
}

uint32_t MXC_GPIO_RevB_GetFlags(mxc_gpio_regs_t *port)
{
    return port->intfl;
}

int MXC_GPIO_RevB_SetVSSEL(mxc_gpio_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
    if (MXC_GPIO_GetConfigLock() == MXC_GPIO_CONFIG_LOCKED) {
        // Configuration is locked.  Ignore any attempts to change it.
        return E_NO_ERROR;
    }

    // Configure the vssel
    switch (vssel) {
    case MXC_GPIO_VSSEL_VDDIO:
        port->vssel &= ~mask;
        break;

    case MXC_GPIO_VSSEL_VDDIOH:
        port->vssel |= mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}
