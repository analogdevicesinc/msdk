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
#include "mxc_errors.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"

/* **** Functions **** */
uint32_t MXC_GPIO_RevA_InGet(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    return (port->in & mask);
}

void MXC_GPIO_RevA_OutSet(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->out_set = mask;
}

void MXC_GPIO_RevA_OutClr(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->out_clr = mask;
}

uint32_t MXC_GPIO_RevA_OutGet(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    return (port->out & mask);
}

void MXC_GPIO_RevA_OutPut(mxc_gpio_reva_regs_t *port, uint32_t mask, uint32_t val)
{
    port->out = (port->out & ~mask) | (val & mask);
}

void MXC_GPIO_RevA_OutToggle(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->out ^= mask;
}

int MXC_GPIO_RevA_IntConfig(const mxc_gpio_cfg_t *cfg, mxc_gpio_int_pol_t pol)
{
    mxc_gpio_reva_regs_t *gpio = (mxc_gpio_reva_regs_t *)cfg->port;

    switch (pol) {
    case MXC_GPIO_INT_HIGH:
        gpio->intpol |= cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode &= ~cfg->mask;
        break;

    case MXC_GPIO_INT_FALLING: /* MXC_GPIO_INT_HIGH */
        gpio->intpol &= ~cfg->mask;
        gpio->dualedge &= ~cfg->mask;
        gpio->intmode |= cfg->mask;
        break;

    case MXC_GPIO_INT_LOW: /* MXC_GPIO_INT_LOW */
        gpio->intpol &= ~cfg->mask;
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

void MXC_GPIO_RevA_EnableInt(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->inten_set = mask;
}

void MXC_GPIO_RevA_DisableInt(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->inten_clr = mask;
}

void MXC_GPIO_RevA_ClearFlags(mxc_gpio_reva_regs_t *port, uint32_t flags)
{
    port->intfl_clr = flags;
}

uint32_t MXC_GPIO_RevA_GetFlags(mxc_gpio_reva_regs_t *port)
{
    return port->intfl;
}

int MXC_GPIO_RevA_SetVSSEL(mxc_gpio_reva_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask)
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

int MXC_GPIO_RevA_SetAF(mxc_gpio_reva_regs_t *port, mxc_gpio_func_t func, uint32_t mask)
{
    if (MXC_GPIO_GetConfigLock() == MXC_GPIO_CONFIG_LOCKED) {
        // Configuration is locked.  Ignore any attempts to change it.
        return E_NO_ERROR;
    }

    //This is required for new devices going forward.
    port->inen |= mask;

    //Switch to I/O mode first
    port->en0_set = mask;

    switch (func) {
    case MXC_GPIO_FUNC_IN:
        port->outen_clr = mask;
        port->en0_set = mask;
        port->en1_clr = mask;
        port->en2_clr = mask;
        port->en3_clr = mask;
        break;

    case MXC_GPIO_FUNC_OUT:
        port->outen_set = mask;
        port->en0_set = mask;
        port->en1_clr = mask;
        port->en2_clr = mask;
        port->en3_clr = mask;
        break;

    case MXC_GPIO_FUNC_ALT1:
        port->en3_clr = mask;
        port->en2_clr = mask;
        port->en1_clr = mask;
        port->en0_clr = mask;
        break;

    case MXC_GPIO_FUNC_ALT2:
        port->en3_clr = mask;
        port->en2_clr = mask;
        port->en1_set = mask;
        port->en0_clr = mask;
        break;

#if TARGET_NUM != 32650
    case MXC_GPIO_FUNC_ALT3:
        port->en3_clr = mask;
        port->en2_set = mask;
        port->en1_clr = mask;
        port->en0_clr = mask;
        break;

    case MXC_GPIO_FUNC_ALT4:
        port->en3_clr = mask;
        port->en2_set = mask;
        port->en1_set = mask;
        port->en0_clr = mask;
        break;

#if TARGET_NUM == 32662
    case MXC_GPIO_FUNC_ALT5:
        port->en3_set = mask;
        port->en2_clr = mask;
        port->en1_clr = mask;
        port->en0_clr = mask;
        break;
#endif
#endif
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

void MXC_GPIO_RevA_SetWakeEn(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->wken_set = mask;
}

void MXC_GPIO_RevA_ClearWakeEn(mxc_gpio_reva_regs_t *port, uint32_t mask)
{
    port->wken_clr = mask;
}

uint32_t MXC_GPIO_RevA_GetWakeEn(mxc_gpio_reva_regs_t *port)
{
    return port->wken;
}

int MXC_GPIO_RevA_SetDriveStrength(mxc_gpio_reva_regs_t *port, mxc_gpio_drvstr_t drvstr,
                                   uint32_t mask)
{
    if (MXC_GPIO_GetConfigLock() == MXC_GPIO_CONFIG_LOCKED) {
        // Configuration is locked.  Ignore any attempts to change it.
        return E_NO_ERROR;
    }

    // Configure the drive strength.
    switch (drvstr) {
    case MXC_GPIO_DRVSTR_0:
        port->ds0 &= ~mask;
        port->ds1 &= ~mask;
        break;

    case MXC_GPIO_DRVSTR_1:
        port->ds0 |= mask;
        port->ds1 &= ~mask;
        break;

    case MXC_GPIO_DRVSTR_2:
        port->ds0 &= ~mask;
        port->ds1 |= mask;
        break;

    case MXC_GPIO_DRVSTR_3:
        port->ds0 |= mask;
        port->ds1 |= mask;
        break;

    default:
        // Set default drive strength to type 0.
        port->ds0 &= ~mask;
        port->ds1 &= ~mask;
    }

    return E_NO_ERROR;
}
