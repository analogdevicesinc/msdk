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
 * $Date: 2018-08-28 17:03:02 -0500 (Tue, 28 Aug 2018) $
 * $Revision: 37424 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_assert.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"
#include <stddef.h>
#include "mxc_sys.h"
#include "lpgcr_regs.h"
#include "mcr_regs.h"

/* **** Definitions **** */

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
    
    return E_NO_ERROR;
}

int MXC_GPIO_Config(const mxc_gpio_cfg_t* cfg)
{
    int port, error;
    mxc_gpio_regs_t *gpio = cfg->port;

    port = MXC_GPIO_GET_IDX(cfg->port);
    
    if (cfg->port == MXC_GPIO3) {
        if (cfg->mask & MXC_GPIO_PIN_0) {
            switch (cfg->func) {
            case MXC_GPIO_FUNC_IN:
                MXC_MCR->gpio3_ctrl &= ~(MXC_F_MCR_GPIO3_CTRL_P30_OE);
                break;
                
            case MXC_GPIO_FUNC_OUT:
                MXC_MCR->gpio3_ctrl |= MXC_F_MCR_GPIO3_CTRL_P30_OE;
                break;
                
            default:
                return E_NOT_SUPPORTED;
            }
            
            switch (cfg->pad) {
            case MXC_GPIO_PAD_NONE:
                MXC_MCR->gpio3_ctrl &= ~(MXC_F_MCR_GPIO3_CTRL_P30_PE);
                break;
                
            case MXC_GPIO_PAD_PULL_UP:
                MXC_MCR->gpio3_ctrl |= MXC_F_MCR_GPIO3_CTRL_P30_PE;
                break;
                
            default:
                return E_NOT_SUPPORTED;
            }
            
        }
        
        if (cfg->mask & MXC_GPIO_PIN_1) {
            switch (cfg->func) {
            case MXC_GPIO_FUNC_IN:
                MXC_MCR->gpio3_ctrl &= ~(MXC_F_MCR_GPIO3_CTRL_P31_OE);
                break;
                
            case MXC_GPIO_FUNC_OUT:
                MXC_MCR->gpio3_ctrl |= MXC_F_MCR_GPIO3_CTRL_P31_OE;
                break;
                
            default:
                return E_NOT_SUPPORTED;
            }
            
            switch (cfg->pad) {
            case MXC_GPIO_PAD_NONE:
                MXC_MCR->gpio3_ctrl &= ~(MXC_F_MCR_GPIO3_CTRL_P31_PE);
                break;
                
            case MXC_GPIO_PAD_PULL_UP:
                MXC_MCR->gpio3_ctrl |= MXC_F_MCR_GPIO3_CTRL_P31_PE;
                break;
                
            default:
                return E_NOT_SUPPORTED;
            }
            
        }
        
        return E_NO_ERROR;
    }
    else {
    MXC_GPIO_Init(1 << port);
    }

    // Configure alternate function
    error = MXC_GPIO_RevA_SetAF ((mxc_gpio_reva_regs_t*)gpio, cfg->func, cfg->mask);
    
    if(error != E_NO_ERROR) {
        return error;
    }

    // Configure the pad
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
    	gpio->padctrl0 &= ~cfg->mask;
    	gpio->padctrl1 &= ~cfg->mask;
        break;

    case MXC_GPIO_PAD_WEAK_PULL_UP:
    	gpio->padctrl0 |=  cfg->mask;
    	gpio->padctrl1 &= ~cfg->mask;
        gpio->ps &= ~cfg->mask;
        break;

    case MXC_GPIO_PAD_PULL_UP:
    	gpio->padctrl0 |=  cfg->mask;
    	gpio->padctrl1 &= ~cfg->mask;
        gpio->ps |= cfg->mask;
        break;

    case MXC_GPIO_PAD_WEAK_PULL_DOWN:
    	gpio->padctrl0 &= ~cfg->mask;
    	gpio->padctrl1 |=  cfg->mask;
        gpio->ps &= ~cfg->mask;
        break;

    case MXC_GPIO_PAD_PULL_DOWN:
    	gpio->padctrl0 &= ~cfg->mask;
    	gpio->padctrl1 |=  cfg->mask;
        gpio->ps |= cfg->mask;
        break;
        
    default:
        return E_BAD_PARAM;
    }
    
    // Configure the vssel
    return MXC_GPIO_SetVSSEL (gpio, cfg->vssel, cfg->mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_InGet(mxc_gpio_regs_t* port, uint32_t mask)
{
    uint32_t results;
    
    if (port == MXC_GPIO3) {
        if ( (mask & MXC_GPIO_PIN_0) && (mask & MXC_GPIO_PIN_1)) {
            results = 0;
            results |= (!!(MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P30_IN) << 0);
            results |= (!!(MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P31_IN) << 1);
            
            return results;
        }
        
        if (mask & MXC_GPIO_PIN_0) {
            return MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P30_IN;
        }
        
        if (mask & MXC_GPIO_PIN_1) {
            return MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P31_IN;
        }
    }
    
    return MXC_GPIO_RevA_InGet((mxc_gpio_reva_regs_t*) port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutSet(mxc_gpio_regs_t* port, uint32_t mask)
{
    if (port == MXC_GPIO3) {
        if (mask & MXC_GPIO_PIN_0) {
            MXC_MCR->gpio3_ctrl |= MXC_F_MCR_GPIO3_CTRL_P30_DO;
        }
        
        if (mask & MXC_GPIO_PIN_1) {
            MXC_MCR->gpio3_ctrl |= MXC_F_MCR_GPIO3_CTRL_P31_DO;
        }
        
        return;
    }
    
    MXC_GPIO_RevA_OutSet((mxc_gpio_reva_regs_t*) port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutClr(mxc_gpio_regs_t* port, uint32_t mask)
{
    if (port == MXC_GPIO3) {
        if (mask & MXC_GPIO_PIN_0) {
            MXC_MCR->gpio3_ctrl &= ~(MXC_F_MCR_GPIO3_CTRL_P30_DO);
        }
        
        if (mask & MXC_GPIO_PIN_1) {
            MXC_MCR->gpio3_ctrl &= ~(MXC_F_MCR_GPIO3_CTRL_P31_DO);
        }
        
        return;
    }
    
    MXC_GPIO_RevA_OutClr((mxc_gpio_reva_regs_t*) port, mask);
}

/* ************************************************************************** */
uint32_t MXC_GPIO_OutGet(mxc_gpio_regs_t* port, uint32_t mask)
{
    uint32_t results;
    
    if (port == MXC_GPIO3) {
        if (mask & (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1)) {
            results = 0;
            results |= (!!(MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P30_DO) << 0);
            results |= (!!(MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P31_DO) << 1);
            
            return results;
        }
        
        if (mask & MXC_GPIO_PIN_0) {
            return MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P30_DO;
        }
        
        if (mask & MXC_GPIO_PIN_1) {
            return MXC_MCR->gpio3_ctrl & MXC_F_MCR_GPIO3_CTRL_P31_DO;
        }
    }
    
    return MXC_GPIO_RevA_OutGet((mxc_gpio_reva_regs_t*) port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_OutPut(mxc_gpio_regs_t* port, uint32_t mask, uint32_t val)
{
    MXC_GPIO_RevA_OutPut((mxc_gpio_reva_regs_t*) port, mask, val);
}

/* ************************************************************************** */
void MXC_GPIO_OutToggle(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_OutToggle((mxc_gpio_reva_regs_t*) port, mask);
}

/* ************************************************************************** */
int MXC_GPIO_IntConfig(const mxc_gpio_cfg_t* cfg, mxc_gpio_int_pol_t pol)
{
    return MXC_GPIO_RevA_IntConfig(cfg, pol);
}

/* ************************************************************************** */
void MXC_GPIO_EnableInt(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_EnableInt((mxc_gpio_reva_regs_t*) port, mask);
}

/* ************************************************************************** */
void MXC_GPIO_DisableInt(mxc_gpio_regs_t* port, uint32_t mask)
{
    MXC_GPIO_RevA_DisableInt((mxc_gpio_reva_regs_t*) port, mask);
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

void MXC_GPIO_ClearFlags(mxc_gpio_regs_t* port, uint32_t flags)
{
    MXC_GPIO_RevA_ClearFlags((mxc_gpio_reva_regs_t*) port, flags);
}

uint32_t MXC_GPIO_GetFlags(mxc_gpio_regs_t* port)
{
    return MXC_GPIO_RevA_GetFlags((mxc_gpio_reva_regs_t*) port);
}

int MXC_GPIO_SetVSSEL(mxc_gpio_regs_t* port, mxc_gpio_vssel_t vssel, uint32_t mask)
{
    return MXC_GPIO_RevA_SetVSSEL((mxc_gpio_reva_regs_t*) port, vssel, mask);
}
