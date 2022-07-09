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
#include <stddef.h>
#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"
#include "uart.h"
#include "uart_revb.h"
#include "uart_common.h"
#include "mcr_regs.h"
#include "dma.h"

/* **** Functions **** */

int MXC_AFE_GPIO_Config(const mxc_gpio_cfg_t* cfg)
{
    int error;
    mxc_gpio_regs_t *gpio = cfg->port;

    // Configure alternate function
    error = MXC_GPIO_RevA_SetAF ((mxc_gpio_reva_regs_t*)gpio, cfg->func, cfg->mask);
    
    if(error != E_NO_ERROR) {
        return error;
    }

    // Configure the pad
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
    	gpio->padctrl0 &= ~cfg->mask;
        break;
        
    case MXC_GPIO_PAD_PULL_UP:
    	gpio->padctrl0 |=  cfg->mask;
        gpio->ps |=  cfg->mask;
        break;
        
    case MXC_GPIO_PAD_PULL_DOWN:
    	gpio->padctrl0 |=  cfg->mask;
        gpio->ps &= ~cfg->mask;
        break;
        
    default:
        return E_BAD_PARAM;
    }
    
    return  E_NO_ERROR;
}

