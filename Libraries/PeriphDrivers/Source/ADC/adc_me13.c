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
#include "adc.h"
#include "adc_regs.h"
#include "adc_reva.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"

int MXC_ADC_Init(void)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_ADC);

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_ADC);
#endif

    //turn on charge pump enable (chip specific)
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_CHGPUMP_PWR;

    return MXC_ADC_RevA_Init((mxc_adc_reva_regs_t *)MXC_ADC);
}

int MXC_ADC_Shutdown(void)
{
    // Disable ADC Charge Pump (chip specific)
    MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_CHGPUMP_PWR;

    MXC_ADC_RevA_Shutdown((mxc_adc_reva_regs_t *)MXC_ADC);

    //Disable ADC peripheral clock
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ADC);

    return E_NO_ERROR;
}

int MXC_ADC_Busy(void)
{
    return MXC_ADC_RevA_Busy((mxc_adc_reva_regs_t *)MXC_ADC);
}

void MXC_ADC_EnableInt(uint32_t flags)
{
    MXC_ADC_RevA_EnableInt((mxc_adc_reva_regs_t *)MXC_ADC, flags);
}

void MXC_ADC_DisableInt(uint32_t flags)
{
    MXC_ADC_RevA_DisableInt((mxc_adc_reva_regs_t *)MXC_ADC, flags);
}

int MXC_ADC_GetFlags(void)
{
    return MXC_ADC_RevA_GetFlags((mxc_adc_reva_regs_t *)MXC_ADC);
}

void MXC_ADC_ClearFlags(uint32_t flags)
{
    MXC_ADC_RevA_ClearFlags((mxc_adc_reva_regs_t *)MXC_ADC, flags);
}

int MXC_ADC_SetConversionSpeed(uint32_t hz)
{
    //check for overflow
    MXC_ASSERT(hz < ((uint32_t)((1U << 31) - 1) / 1024));
    uint32_t adc_clock_freq = 1024 * hz;

    if (adc_clock_freq > MXC_ADC_MAX_CLOCK) {
        return E_BAD_PARAM;
    }

    uint8_t divider = PeripheralClock / adc_clock_freq;

    if (divider > 0xf || divider < 2) {
        return E_BAD_PARAM;
    }

    //disable clock
    MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_CLK_EN;
    //clear clock divisor
    MXC_GCR->pclkdiv &= (~MXC_F_GCR_PCLKDIV_ADCFRQ);
    //load in new clock divisor
    MXC_GCR->pclkdiv |= (divider << MXC_F_GCR_PCLKDIV_ADCFRQ_POS);

    //enable clock
    MXC_ADC_RevA_SetConversionSpeed((mxc_adc_reva_regs_t *)MXC_ADC, hz);

    return MXC_ADC_GetConversionSpeed();
}

int MXC_ADC_GetConversionSpeed(void)
{
    uint8_t divider = (MXC_GCR->pclkdiv & MXC_F_GCR_PCLKDIV_ADCFRQ) >> MXC_F_GCR_PCLKDIV_ADCFRQ_POS;

    return MXC_ADC_RevA_GetConversionSpeed(divider);
}

void MXC_ADC_SetDataAlignment(int msbJustify)
{
    MXC_ADC_RevA_SetDataAlignment((mxc_adc_reva_regs_t *)MXC_ADC, msbJustify);
}

void MXC_ADC_SetExtScale(mxc_adc_scale_t scale)
{
    MXC_ADC_RevA_SetExtScale((mxc_adc_reva_regs_t *)MXC_ADC, scale);
}

void MXC_ADC_EnableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_EnableMonitor((mxc_adc_reva_regs_t *)MXC_ADC, monitor);
}

void MXC_ADC_DisableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_DisableMonitor((mxc_adc_reva_regs_t *)MXC_ADC, monitor);
}

void MXC_ADC_SetMonitorHighThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ADC_RevA_SetMonitorHighThreshold((mxc_adc_reva_regs_t *)MXC_ADC, monitor, threshold);
}

int MXC_ADC_GetMonitorHighThreshold(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorHighThreshold((mxc_adc_reva_regs_t *)MXC_ADC, monitor);
}

void MXC_ADC_SetMonitorLowThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ADC_RevA_SetMonitorLowThreshold((mxc_adc_reva_regs_t *)MXC_ADC, monitor, threshold);
}

int MXC_ADC_GetMonitorLowThreshold(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorLowThreshold((mxc_adc_reva_regs_t *)MXC_ADC, monitor);
}

void MXC_ADC_SetMonitorChannel(mxc_adc_monitor_t monitor, mxc_adc_chsel_t channel)
{
    MXC_ADC_RevA_SetMonitorChannel((mxc_adc_reva_regs_t *)MXC_ADC, monitor, channel);
}

int MXC_ADC_GetMonitorChannel(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorChannel((mxc_adc_reva_regs_t *)MXC_ADC, monitor);
}

void MXC_ADC_EnableMonitorAsync(mxc_adc_monitor_t monitor, mxc_adc_monitor_cb_t callback)
{
    MXC_ADC_RevA_EnableMonitorAsync(monitor, callback);
}

void MXC_ADC_DisableMonitorAsync(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_DisableMonitorAsync(monitor);
}

int MXC_ADC_StartConversion(mxc_adc_chsel_t channel)
{
    return MXC_ADC_RevA_StartConversion((mxc_adc_reva_regs_t *)MXC_ADC, channel);
}

int MXC_ADC_StartConversionAsync(mxc_adc_chsel_t channel, mxc_adc_complete_cb_t callback)
{
    return MXC_ADC_RevA_StartConversionAsync((mxc_adc_reva_regs_t *)MXC_ADC, channel, callback);
}

int MXC_ADC_StartConversionDMA(mxc_adc_chsel_t channel, uint16_t *data, void (*callback)(int, int))
{
    return MXC_ADC_RevA_StartConversionDMA((mxc_adc_reva_regs_t *)MXC_ADC, channel, MXC_DMA, data,
                                           callback);
}

int MXC_ADC_Handler(void)
{
    return MXC_ADC_RevA_Handler((mxc_adc_reva_regs_t *)MXC_ADC);
}

int MXC_ADC_Convert(mxc_adc_conversion_req_t *req)
{
    return MXC_ADC_RevA_Convert((mxc_adc_reva_regs_t *)MXC_ADC, req);
}

int MXC_ADC_ConvertAsync(mxc_adc_conversion_req_t *req)
{
    return MXC_ADC_RevA_ConvertAsync((mxc_adc_reva_regs_t *)MXC_ADC, req);
}

void MXC_ADC_Monitor(mxc_adc_monitor_req_t req)
{
    MXC_ADC_RevA_Monitor((mxc_adc_reva_regs_t *)MXC_ADC, req);
}

void MXC_ADC_MonitorAsync(mxc_adc_monitor_req_t req)
{
    MXC_ADC_RevA_MonitorAsync((mxc_adc_reva_regs_t *)MXC_ADC, req);
}

int MXC_ADC_GetData(uint16_t *outdata)
{
    return MXC_ADC_RevA_GetData((mxc_adc_reva_regs_t *)MXC_ADC, outdata);
}
