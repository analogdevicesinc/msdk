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

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spimss_reva_regs.h"
#include "spimss_reva.h"

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SPIMSS_Init(mxc_spimss_regs_t *spi, unsigned mode, unsigned freq)
{
    if (mode > 3) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if (freq > PeripheralClock) {
        return E_BAD_PARAM;
    }

    // Configure GPIO for spimss
    if (spi == MXC_SPIMSS) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_I2S);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2S);
        MXC_GPIO_Config(&gpio_cfg_i2s);
    } else {
        return E_NO_DEVICE;
    }

    return MXC_SPIMSS_RevA_Init((mxc_spimss_reva_regs_t *)spi, mode, freq);
}
/* ************************************************************************* */
int MXC_SPIMSS_Shutdown(mxc_spimss_regs_t *spi)
{
    MXC_SPIMSS_RevA_Shutdown((mxc_spimss_reva_regs_t *)spi);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2S);
    return E_NO_ERROR;
}
/* ************************************************************************** */
void MXC_SPIMSS_Handler(mxc_spimss_regs_t *spi) // From the IRQ
{
    MXC_SPIMSS_RevA_Handler((mxc_spimss_reva_regs_t *)spi);
}

/* ************************************************************************** */
int MXC_SPIMSS_MasterTrans(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req)
{
    return MXC_SPIMSS_RevA_MasterTrans((mxc_spimss_reva_regs_t *)spi, (spimss_reva_req_t *)req);
}

/* ************************************************************************** */
int MXC_SPIMSS_SlaveTrans(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req)
{
    return MXC_SPIMSS_RevA_SlaveTrans((mxc_spimss_reva_regs_t *)spi, (spimss_reva_req_t *)req);
}

/* ************************************************************************** */
int MXC_SPIMSS_MasterTransAsync(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req)
{
    return MXC_SPIMSS_RevA_MasterTransAsync((mxc_spimss_reva_regs_t *)spi,
                                            (spimss_reva_req_t *)req);
}

/* ************************************************************************** */
int MXC_SPIMSS_SlaveTransAsync(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req)
{
    return MXC_SPIMSS_RevA_SlaveTransAsync((mxc_spimss_reva_regs_t *)spi, (spimss_reva_req_t *)req);
}

/* ************************************************************************* */
int MXC_SPIMSS_AbortAsync(mxc_spimss_req_t *req)
{
    return MXC_SPIMSS_RevA_AbortAsync((spimss_reva_req_t *)req);
}

/* ************************************************************************** */
int MXC_SPIMSS_MasterTransDMA(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req)
{
    return MXC_SPIMSS_RevA_MasterTransDMA((mxc_spimss_reva_regs_t *)spi, (spimss_reva_req_t *)req);
}

/* ************************************************************************* */
int MXC_SPIMSS_SetAutoDMAHandlers(mxc_spimss_regs_t *spi, bool enable)
{
    return MXC_SPIMSS_RevA_SetAutoDMAHandlers((mxc_spimss_reva_regs_t *)spi, enable);
}

/* ************************************************************************* */
int MXC_SPIMSS_SetTXDMAChannel(mxc_spimss_regs_t *spi, unsigned int channel)
{
    return MXC_SPIMSS_RevA_SetTXDMAChannel((mxc_spimss_reva_regs_t *)spi, channel);
}

/* ************************************************************************* */
int MXC_SPIMSS_GetTXDMAChannel(mxc_spimss_regs_t *spi)
{
    return MXC_SPIMSS_RevA_GetTXDMAChannel((mxc_spimss_reva_regs_t *)spi);
}

/* ************************************************************************* */
int MXC_SPIMSS_SetRXDMAChannel(mxc_spimss_regs_t *spi, unsigned int channel)
{
    return MXC_SPIMSS_RevA_SetRXDMAChannel((mxc_spimss_reva_regs_t *)spi, channel);
}

/* ************************************************************************* */
int MXC_SPIMSS_GetRXDMAChannel(mxc_spimss_regs_t *spi)
{
    return MXC_SPIMSS_RevA_GetRXDMAChannel((mxc_spimss_reva_regs_t *)spi);
}
