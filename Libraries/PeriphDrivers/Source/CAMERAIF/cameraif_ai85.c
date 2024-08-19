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
