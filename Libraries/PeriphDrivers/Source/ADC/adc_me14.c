/* ****************************************************************************
 * Copyright(C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"), 
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
 *************************************************************************** */
#include "adc.h"
#include "adc_regs.h"
#include "adc_reva.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include <stdio.h>

int MXC_ADC_Init(void)
{
    MXC_GCR->rstr0 |= MXC_F_GCR_RSTR0_ADC;

    // Reset ADC block
    while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_ADC)
        ;

    // Enable ADC peripheral clock
    MXC_GCR->perckcn0 &= ~MXC_F_GCR_PERCKCN0_ADCD;

    return MXC_ADC_RevA_Init((mxc_adc_reva_regs_t*)MXC_ADC);
}

int MXC_ADC_Shutdown(void)
{
    MXC_ADC_RevA_Shutdown((mxc_adc_reva_regs_t*)MXC_ADC);

    //Disable ADC peripheral clock
    MXC_GCR->perckcn0 |= MXC_F_GCR_PERCKCN0_ADCD;

    return E_NO_ERROR;
}

int MXC_ADC_Busy(void)
{
    return MXC_ADC_RevA_Busy((mxc_adc_reva_regs_t*)MXC_ADC);
}

void MXC_ADC_EnableInt(uint32_t flags)
{
    MXC_ADC_RevA_EnableInt((mxc_adc_reva_regs_t*)MXC_ADC, flags);
}
void MXC_ADC_DisableInt(uint32_t flags)
{
    MXC_ADC_RevA_DisableInt((mxc_adc_reva_regs_t*)MXC_ADC, flags);
}

int MXC_ADC_GetFlags(void)
{
    return MXC_ADC_RevA_GetFlags((mxc_adc_reva_regs_t*)MXC_ADC);
}

void MXC_ADC_ClearFlags(uint32_t flags)
{
    MXC_ADC_RevA_ClearFlags((mxc_adc_reva_regs_t*)MXC_ADC, flags);
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
    MXC_GCR->pckdiv &= (~MXC_F_GCR_PCKDIV_ADCFRQ);
    //load in new clock divisor
    MXC_GCR->pckdiv |= (divider << MXC_F_GCR_PCKDIV_ADCFRQ_POS);

    //enable clock
    MXC_ADC_RevA_SetConversionSpeed((mxc_adc_reva_regs_t*)MXC_ADC, hz);

    return MXC_ADC_GetConversionSpeed();
}

int MXC_ADC_GetConversionSpeed(void)
{
    uint8_t divider = (MXC_GCR->pckdiv & MXC_F_GCR_PCKDIV_ADCFRQ) >> MXC_F_GCR_PCKDIV_ADCFRQ_POS;
    return MXC_ADC_RevA_GetConversionSpeed(divider);
}

void MXC_ADC_SetDataAlignment(int msbJustify)
{
    MXC_ADC_RevA_SetDataAlignment((mxc_adc_reva_regs_t*)MXC_ADC, msbJustify);
}

void MXC_ADC_SetExtScale(mxc_adc_scale_t scale)
{
    MXC_ADC_RevA_SetExtScale((mxc_adc_reva_regs_t*)MXC_ADC, scale);
}

void MXC_ADC_EnableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_EnableMonitor((mxc_adc_reva_regs_t*)MXC_ADC, monitor);
}

void MXC_ADC_DisableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_DisableMonitor((mxc_adc_reva_regs_t*)MXC_ADC, monitor);
}

void MXC_ADC_SetMonitorHighThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ADC_RevA_SetMonitorHighThreshold((mxc_adc_reva_regs_t*)MXC_ADC, monitor, threshold);
}

int MXC_ADC_GetMonitorHighThreshold(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorHighThreshold((mxc_adc_reva_regs_t*)MXC_ADC, monitor);
}

void MXC_ADC_SetMonitorLowThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ADC_RevA_SetMonitorLowThreshold((mxc_adc_reva_regs_t*)MXC_ADC, monitor, threshold);
}

int MXC_ADC_GetMonitorLowThreshold(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorLowThreshold((mxc_adc_reva_regs_t*)MXC_ADC, monitor);
}

void MXC_ADC_SetMonitorChannel(mxc_adc_monitor_t monitor, mxc_adc_chsel_t channel)
{
    MXC_ADC_RevA_SetMonitorChannel((mxc_adc_reva_regs_t*)MXC_ADC, monitor, channel);
}

int MXC_ADC_GetMonitorChannel(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorChannel((mxc_adc_reva_regs_t*)MXC_ADC, monitor);
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
    return MXC_ADC_RevA_StartConversion((mxc_adc_reva_regs_t*)MXC_ADC, channel);
}

int MXC_ADC_StartConversionAsync(mxc_adc_chsel_t channel, mxc_adc_complete_cb_t callback)
{
    return MXC_ADC_RevA_StartConversionAsync((mxc_adc_reva_regs_t*)MXC_ADC, channel, callback);
}

int MXC_ADC_StartConversionDMA(mxc_adc_chsel_t channel, mxc_dma_regs_t* dma, uint16_t* data,
                               void (*callback)(int, int))
{
    if (MXC_DMA_GET_IDX(dma) == -1) {
        return E_BAD_PARAM;
    }

    return MXC_ADC_RevA_StartConversionDMA((mxc_adc_reva_regs_t*)MXC_ADC, channel, dma, data,
                                           callback);
}

int MXC_ADC_Handler(void)
{
    return MXC_ADC_RevA_Handler((mxc_adc_reva_regs_t*)MXC_ADC);
}

int MXC_ADC_Convert(mxc_adc_conversion_req_t* req)
{
    return MXC_ADC_RevA_Convert((mxc_adc_reva_regs_t*)MXC_ADC, req);
}

int MXC_ADC_ConvertAsync(mxc_adc_conversion_req_t* req)
{
    return MXC_ADC_RevA_ConvertAsync((mxc_adc_reva_regs_t*)MXC_ADC, req);
}

void MXC_ADC_Monitor(mxc_adc_monitor_req_t req)
{
    MXC_ADC_RevA_Monitor((mxc_adc_reva_regs_t*)MXC_ADC, req);
}

void MXC_ADC_MonitorAsync(mxc_adc_monitor_req_t req)
{
    MXC_ADC_RevA_MonitorAsync((mxc_adc_reva_regs_t*)MXC_ADC, req);
}

int MXC_ADC_GetData(uint16_t* outdata)
{
    return MXC_ADC_RevA_GetData((mxc_adc_reva_regs_t*)MXC_ADC, outdata);
}
