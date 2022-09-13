/*******************************************************************************
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
******************************************************************************/

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "cameraif.h"
#include "cameraif_reva.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

int MXC_PCIF_Init(mxc_pcif_gpio_datawidth_t gpioDataWidth)
{
    if ((gpioDataWidth != MXC_PCIF_GPIO_DATAWIDTH_8_BIT) &&
        (gpioDataWidth != MXC_PCIF_GPIO_DATAWIDTH_10_BIT) &&
        (gpioDataWidth != MXC_PCIF_GPIO_DATAWIDTH_12_BIT)) {
        return E_BAD_PARAM;
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PCIF);

    switch (gpioDataWidth) {
    case MXC_PCIF_DATAWIDTH_8_BIT:
        MXC_GPIO_Config(&gpio_cfg_pcif_P0_BITS_0_7);
        break;

    case MXC_PCIF_DATAWIDTH_10_BIT:
        MXC_GPIO_Config(&gpio_cfg_pcif_P0_BITS_0_7);
        MXC_GPIO_Config(&gpio_cfg_pcif_P0_BITS_8_9);
        break;

    case MXC_PCIF_DATAWIDTH_12_BIT:
        MXC_GPIO_Config(&gpio_cfg_pcif_P0_BITS_0_7);
        MXC_GPIO_Config(&gpio_cfg_pcif_P0_BITS_8_9);
        MXC_GPIO_Config(&gpio_cfg_pcif_P1_BITS_10_11);
        break;
    }

    MXC_GPIO_Config(&gpio_cfg_pcif_vsync);
    MXC_GPIO_Config(&gpio_cfg_pcif_hsync);
    MXC_GPIO_Config(&gpio_cfg_pcif_xclk);
    return E_NO_ERROR;
}

void MXC_PCIF_SetDataWidth(mxc_pcif_datawidth_t gpioDatawidth)
{
    MXC_PCIF_RevA_SetDatawidth((mxc_cameraif_reva_regs_t *)MXC_PCIF, gpioDatawidth);
}

void MXC_PCIF_SetTimingSel(mxc_pcif_timingsel_t timingsel)
{
    MXC_PCIF_RevA_SetTimingSel((mxc_cameraif_reva_regs_t *)MXC_PCIF, timingsel);
}

void MXC_PCIF_SetThreshold(int fifo_thrsh)
{
    MXC_PCIF_RevA_SetThreshold((mxc_cameraif_reva_regs_t *)MXC_PCIF, fifo_thrsh);
}

void MXC_PCIF_EnableInt(uint32_t flags)
{
    MXC_PCIF_RevA_EnableInt((mxc_cameraif_reva_regs_t *)MXC_PCIF, flags);
}

void MXC_PCIF_DisableInt(uint32_t flags)
{
    MXC_PCIF_RevA_DisableInt((mxc_cameraif_reva_regs_t *)MXC_PCIF, flags);
}

void MXC_PCIF_Start(mxc_pcif_readmode_t readmode)
{
    MXC_PCIF_RevA_Start((mxc_cameraif_reva_regs_t *)MXC_PCIF, readmode);
}

void MXC_PCIF_Stop(void)
{
    MXC_PCIF_RevA_Stop((mxc_cameraif_reva_regs_t *)MXC_PCIF);
}

unsigned int MXC_PCIF_GetData(void)
{
    return MXC_PCIF_RevA_GetData((mxc_cameraif_reva_regs_t *)MXC_PCIF);
}

/**@} end of group cameraif */
