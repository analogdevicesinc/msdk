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
 * $Date: 2018-12-18 15:37:22 -0600 (Tue, 18 Dec 2018) $
 * $Revision: 40072 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include "gpio.h"
#include "gpio_common.h"
#include "gpio_reva.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include <stddef.h>

/* **** Definitions **** */

/* **** Globals **** */

// static void (*callback[MXC_CFG_GPIO_INSTANCES][MXC_CFG_GPIO_PINS_PORT])(void *);
// static void *cbparam[MXC_CFG_GPIO_INSTANCES][MXC_CFG_GPIO_PINS_PORT];

/* **** Functions **** */

int MXC_GPIO_Init(uint32_t port)
{
    if (port == MXC_GPIO_PORT_0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    } else if (port == MXC_GPIO_PORT_1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    } else if (port == MXC_GPIO_PORT_2) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    } else if (port == MXC_GPIO_PORT_3) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO3);
    } else {
        return E_BAD_PARAM;
    }
    return MXC_GPIO_Common_Init(port);
}

/* ************************************************************************** */
int MXC_GPIO_Shutdown(uint32_t port)
{
    if (port == MXC_GPIO_PORT_0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    } else if (port == MXC_GPIO_PORT_1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    } else if (port == MXC_GPIO_PORT_2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    } else if (port == MXC_GPIO_PORT_3) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_GPIO3);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_GPIO_Reset(uint32_t port)
{
    if (port == MXC_GPIO_PORT_0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO0);
    } else if (port == MXC_GPIO_PORT_1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO1);
    } else if (port == MXC_GPIO_PORT_2) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO2);
    } else if (port == MXC_GPIO_PORT_3) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_GPIO3);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
/*
 *       GPIO_EN2 |  GPIO_EN1           |  GPIO_EN            |   Function
 *  --------------|---------------------|---------------------|----------------------
 *     0          |          0          |          0          |     Alternative 1
 *     0          |          1          |          0          |     Alternative 2
 *     0          |          0          |          1          |     GPIO (default)
 */
int MXC_GPIO_Config(const mxc_gpio_cfg_t* cfg)
{
    int err;
    mxc_gpio_regs_t* gpio = cfg->port;

    // Set the GPIO type
    if ((err = MXC_GPIO_RevA_SetAF((mxc_gpio_reva_regs_t*)gpio, cfg->func, cfg->mask))
        != E_NO_ERROR) {
        return err;
    }

    // Configure the pad
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
        gpio->pdpu_sel0 &= ~cfg->mask;
        gpio->pdpu_sel1 &= ~cfg->mask;
        gpio->pssel &= ~cfg->mask;
        break;
    case MXC_GPIO_PAD_WEAK_PULL_UP:
        gpio->pdpu_sel0 |= cfg->mask;
        gpio->pdpu_sel1 &= ~cfg->mask;
        gpio->pssel |= cfg->mask;
        break;
    case MXC_GPIO_PAD_WEAK_PULL_DOWN:
        gpio->pdpu_sel0 &= ~cfg->mask;
        gpio->pdpu_sel1 |= cfg->mask;
        gpio->pssel |= cfg->mask;
        break;
    case MXC_GPIO_PAD_STRONG_PULL_UP:
        gpio->pdpu_sel0 |= cfg->mask;
        gpio->pdpu_sel1 &= ~cfg->mask;
        gpio->pssel &= ~cfg->mask;
        break;
    case MXC_GPIO_PAD_STRONG_PULL_DOWN:
        gpio->pdpu_sel0 &= ~cfg->mask;
        gpio->pdpu_sel1 |= cfg->mask;
        gpio->pssel &= ~cfg->mask;
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_GPIO_SetVSSEL(gpio, cfg->vssel, cfg->mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_InGet(mxc_gpio_regs_t* port, uint32_t mask)
{
    return MXC_GPIO_RevA_InGet((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutSet(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_OutSet((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutClr(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_OutClr((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_OutGet(mxc_gpio_regs_t* port, uint32_t mask)
{
    return MXC_GPIO_RevA_OutGet((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutPut(mxc_gpio_regs_t* port, uint32_t mask, uint32_t val)
{
    return MXC_GPIO_RevA_OutPut((mxc_gpio_reva_regs_t*)port, mask, val);
}

/* ************************************************************************** */
void MXC_GPIO_OutToggle(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_OutToggle((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
int MXC_GPIO_IntConfig(const mxc_gpio_cfg_t* cfg, mxc_gpio_int_pol_t pol)
{
    return MXC_GPIO_RevA_IntConfig(cfg, pol);
}

/* ************************************************************************** */
void MXC_GPIO_EnableInt(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_EnableInt((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_DisableInt(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_DisableInt((mxc_gpio_reva_regs_t*)port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_GetFlags(mxc_gpio_regs_t* port)
{
    return MXC_GPIO_RevA_GetFlags((mxc_gpio_reva_regs_t*)port);
}

/* ************************************************************************** */
void MXC_GPIO_ClearFlags(mxc_gpio_regs_t* port, uint32_t flags)
{
    MXC_GPIO_RevA_ClearFlags((mxc_gpio_reva_regs_t*)port, flags);
}

/* ************************************************************************** */
void MXC_GPIO_RegisterCallback(const mxc_gpio_cfg_t* cfg, mxc_gpio_callback_fn func, void* cbdata)
{
    MXC_GPIO_Common_RegisterCallback(cfg, func, cbdata);
}

/* ************************************************************************** */
void MXC_GPIO_Handler(unsigned int port)
{
    MXC_GPIO_Common_Handler(port);
}

/* ************************************************************************** */
int MXC_GPIO_SetVSSEL(mxc_gpio_regs_t* port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
    return MXC_GPIO_RevA_SetVSSEL((mxc_gpio_reva_regs_t*)port, vssel, mask);
}
