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
#include "dma.h"
#include "i2s.h"
#include "i2s_reva.h"
#include "i2s_regs.h"

int MXC_I2S_Init(mxc_i2s_req_t *req)
{
    MXC_I2S_Shutdown();

    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERFO);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2S);
    MXC_GPIO_Config(&gpio_cfg_i2s0);

    return MXC_I2S_RevA_Init(req);
}

int MXC_I2S_Shutdown(void)
{
    MXC_I2S_RevA_Shutdown();

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2S);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_I2S);

    return E_NO_ERROR;
}

int MXC_I2S_ConfigData(mxc_i2s_req_t *req)
{
    return MXC_I2S_RevA_ConfigData(req);
}

void MXC_I2S_TXEnable(void)
{
    MXC_I2S_RevA_TXEnable();
}

void MXC_I2S_TXDisable(void)
{
    MXC_I2S_RevA_TXDisable();
}

void MXC_I2S_RXEnable(void)
{
    MXC_I2S_RevA_RXEnable();
}

void MXC_I2S_RXDisable(void)
{
    MXC_I2S_RevA_RXDisable();
}

int MXC_I2S_SetRXThreshold(uint8_t threshold)
{
    return MXC_I2S_RevA_SetRXThreshold(threshold);
}

int MXC_I2S_SetFrequency(mxc_i2s_ch_mode_t mode, uint16_t clkdiv)
{
    return MXC_I2S_RevA_SetFrequency(mode, clkdiv);
}

int MXC_I2S_SetSampleRate(uint32_t smpl_rate, mxc_i2s_wsize_t smpl_sz)
{
    return MXC_I2S_RevA_SetSampleRate((mxc_i2s_reva_regs_t *)MXC_I2S, smpl_rate, smpl_sz,
                                      ERFO_FREQ);
}

int MXC_I2S_GetSampleRate(void)
{
    return MXC_I2S_RevA_GetSampleRate((mxc_i2s_reva_regs_t *)MXC_I2S, ERFO_FREQ);
}

int MXC_I2S_CalculateClockDiv(uint32_t smpl_rate, mxc_i2s_wsize_t smpl_sz)
{
    return MXC_I2S_RevA_CalculateClockDiv((mxc_i2s_reva_regs_t *)MXC_I2S, smpl_rate, smpl_sz,
                                          ERFO_FREQ);
}

void MXC_I2S_Flush(void)
{
    MXC_I2S_RevA_Flush();
}

int MXC_I2S_FillTXFIFO(void *txData, mxc_i2s_wsize_t wordSize, int len, int smpl_cnt)
{
    return MXC_I2S_RevA_FillTXFIFO((mxc_i2s_reva_regs_t *)MXC_I2S, txData, wordSize, len, smpl_cnt);
}

int MXC_I2S_ReadRXFIFO(void *rxData, mxc_i2s_wsize_t wordSize, int len, int smpl_cnt)
{
    return MXC_I2S_RevA_ReadRXFIFO((mxc_i2s_reva_regs_t *)MXC_I2S, rxData, wordSize, len, smpl_cnt);
}

void MXC_I2S_EnableInt(uint32_t flags)
{
    MXC_I2S_RevA_EnableInt(flags);
}

void MXC_I2S_DisableInt(uint32_t flags)
{
    MXC_I2S_RevA_DisableInt(flags);
}

int MXC_I2S_GetFlags(void)
{
    return MXC_I2S_RevA_GetFlags();
}

void MXC_I2S_ClearFlags(uint32_t flags)
{
    MXC_I2S_RevA_ClearFlags(flags);
}

int MXC_I2S_Transaction(mxc_i2s_req_t *i2s_req)
{
    return MXC_I2S_RevA_Transaction((mxc_i2s_reva_regs_t *)MXC_I2S, i2s_req);
}

int MXC_I2S_TransactionAsync(mxc_i2s_req_t *i2s_req)
{
    return MXC_I2S_RevA_TransactionAsync((mxc_i2s_reva_regs_t *)MXC_I2S, i2s_req);
}

int MXC_I2S_TXDMAConfig(void *src_addr, int len)
{
    return MXC_I2S_RevA_TXDMAConfig(src_addr, len);
}

int MXC_I2S_RXDMAConfig(void *dest_addr, int len)
{
    return MXC_I2S_RevA_RXDMAConfig(dest_addr, len);
}

void MXC_I2S_Handler(void)
{
    MXC_I2S_RevA_Handler((mxc_i2s_reva_regs_t *)MXC_I2S);
}

void MXC_I2S_RegisterDMACallback(void (*callback)(int, int))
{
    MXC_I2S_RegisterDMACallback(callback);
}

void MXC_I2S_RegisterAsyncCallback(void (*callback)(int))
{
    MXC_I2S_RevA_RegisterAsyncCallback(callback);
}
