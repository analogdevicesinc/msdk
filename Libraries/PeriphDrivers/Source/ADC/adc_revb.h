/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#include "adc.h"
#include "adc_revb_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include <stdio.h>

int MXC_ADC_RevB_Init(mxc_adc_revb_regs_t *adc, mxc_adc_req_t *req);

int MXC_ADC_RevB_Shutdown(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_EnableInt(mxc_adc_revb_regs_t *adc, uint32_t flags);

void MXC_ADC_RevB_DisableInt(mxc_adc_revb_regs_t *adc, uint32_t flags);

int MXC_ADC_RevB_GetFlags(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_ClearFlags(mxc_adc_revb_regs_t *adc, uint32_t flags);

void MXC_ADC_RevB_ClockSelect(mxc_adc_revb_regs_t *adc, mxc_adc_clock_t clock);

int MXC_ADC_RevB_StartConversion(mxc_adc_revb_regs_t *adcq);

int MXC_ADC_RevB_StartConversionAsync(mxc_adc_revb_regs_t *adc, mxc_adc_complete_cb_t callback);

int MXC_ADC_RevB_StartConversionDMA(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req,
                                    int *data, void (*callback)(int, int));

int MXC_ADC_RevB_Handler(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_EnableConversion(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_DisableConversion(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_TS_SelectEnable(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_TS_SelectDisable(mxc_adc_revb_regs_t *adc);

int MXC_ADC_RevB_GetData(mxc_adc_revb_regs_t *adc, int *outdata);

uint16_t MXC_ADC_RevB_FIFO_Level(mxc_adc_revb_regs_t *adc);

int MXC_ADC_RevB_FIFO_Threshold_Config(mxc_adc_revb_regs_t *adc, uint32_t fifo_threshold);

int MXC_ADC_RevB_AverageConfig(mxc_adc_revb_regs_t *adc, mxc_adc_avg_t avg_number);

void MXC_ADC_RevB_Clear_ChannelSelect(mxc_adc_revb_regs_t *adc);

void MXC_ADC_RevB_TriggerConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req);

void MXC_ADC_RevB_ConversionModeConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req);

int MXC_ADC_RevB_SetConversionDelay(mxc_adc_revb_regs_t *adc, int delay);

int MXC_ADC_RevB_SlotsConfig(mxc_adc_revb_regs_t *adc, mxc_adc_conversion_req_t *req);

int MXC_ADC_RevB_ChSelectConfig(mxc_adc_revb_regs_t *adc, mxc_adc_chsel_t ch, uint32_t slot_num);

//End
