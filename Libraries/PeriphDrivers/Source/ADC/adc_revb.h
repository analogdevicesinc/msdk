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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_ADC_ADC_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_ADC_ADC_REVB_H_

#include <stdio.h>
#include <stdbool.h>
#include "adc.h"
#include "adc_revb_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"

int MXC_ADC_RevB_Init(mxc_adc_revb_regs_t *adc, mxc_adc_req_t *req);

int MXC_ADC_RevB_SetClockSource(mxc_adc_revb_regs_t *adc, mxc_adc_clock_t clock_source);

int MXC_ADC_RevB_SetClockDiv(mxc_adc_revb_regs_t *adc, mxc_adc_clkdiv_t div);

int MXC_ADC_RevB_LockClockSource(mxc_adc_revb_regs_t *adc, bool lock);

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

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_ADC_ADC_REVB_H_
