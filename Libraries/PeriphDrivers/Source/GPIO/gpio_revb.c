/* *****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 **************************************************************************** */

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_assert.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"
#include <stddef.h>

/* **** Functions **** */
int MXC_GPIO_RevB_Config(const mxc_gpio_cfg_t *cfg, uint8_t psMask)
{
    mxc_gpio_regs_t *gpio = cfg->port;

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
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        if (psMask == MXC_GPIO_PS_PULL_SELECT) {
            gpio->ps &= ~cfg->mask;
        }
        break;

    case MXC_GPIO_PAD_PULL_UP:
        gpio->padctrl0 |= cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        if (psMask == MXC_GPIO_PS_PULL_SELECT) {
            gpio->ps |= cfg->mask;
        }
        break;

    case MXC_GPIO_PAD_PULL_DOWN:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 |= cfg->mask;
        if (psMask == MXC_GPIO_PS_PULL_SELECT) {
            gpio->ps &= ~cfg->mask;
        }
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

//TODO: Unnecessary?
void MXC_GPIO_RevB_ClearFlags(mxc_gpio_regs_t *port, uint32_t flags)
{
    port->intfl_clr = flags;
}

//TODO: Unnecessary?
uint32_t MXC_GPIO_RevB_GetFlags(mxc_gpio_regs_t *port)
{
    return port->intfl;
}

//TODO: Unnecessary?
int MXC_GPIO_RevB_SetVSSEL(mxc_gpio_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
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
