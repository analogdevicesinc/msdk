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

#include <stddef.h>
#include <stdint.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_pins.h"
#include "gpio.h"
#include "dma.h"
#include "i2s.h"
#include "i2s_reva.h"

int dma_channel = -1;
mxc_i2s_direction_t dir;

/* ************************************************************************* */
int MXC_I2S_Init(const mxc_i2s_config_t *cfg, void (*dma_ctz_cb)(int, int))
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2S);
    MXC_GPIO_Config(&gpio_cfg_i2s);
#endif

    return MXC_I2S_RevA_Init((mxc_spimss_reva_regs_t *)MXC_SPIMSS, cfg, dma_ctz_cb);
}

/* ************************************************************************* */
int MXC_I2S_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2S);
    return MXC_I2S_RevA_Shutdown((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_Mute(void)
{
    return MXC_I2S_RevA_Mute((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_Unmute(void)
{
    return MXC_I2S_RevA_Unmute((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_Pause(void)
{
    return MXC_I2S_RevA_Pause((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_Unpause(void)
{
    return MXC_I2S_RevA_Unpause((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_Stop(void)
{
    return MXC_I2S_RevA_Stop((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_Start(void)
{
    return MXC_I2S_RevA_Start((mxc_spimss_reva_regs_t *)MXC_SPIMSS);
}

/* ************************************************************************* */
int MXC_I2S_DMA_ClearFlags(void)
{
    return MXC_I2S_RevA_DMA_ClearFlags();
}

/* ************************************************************************* */
int MXC_I2S_DMA_SetAddrCnt(void *src_addr, void *dst_addr, unsigned int count)
{
    return MXC_I2S_RevA_DMA_SetAddrCnt(src_addr, dst_addr, count);
}

/* ************************************************************************* */
int MXC_I2S_DMA_SetReload(void *src_addr, void *dst_addr, unsigned int count)
{
    return MXC_I2S_RevA_DMA_SetReload(src_addr, dst_addr, count);
}
