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
#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_ADC_ADC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_ADC_ADC_REVA_H_

#include <stdio.h>
#include "adc.h"
#include "adc_reva_regs.h"
#include "dma_reva_regs.h"
#include "dma_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_lock.h"

/***** MACROS *****/
#define AIN0 0x0
#define AIN1 0x1
#define AIN2 0x2
#define AIN3 0x3
#define AIN4 0x4
#define AIN5 0x5
#define AIN6 0x6
#define AIN7 0x7
#define AIN8 0x8
#define AIN9 0x9
#define AIN10 0xA
#define AIN11 0xB
#define AIN12 0xC
#define AIN13 0xD
#define AIN14 0xE
#define AIN15 0xF
#define AIN16 0x10

/***** FUNCTIONS *****/
int MXC_ADC_RevA_Init(mxc_adc_reva_regs_t *adc);
int MXC_ADC_RevA_Shutdown(mxc_adc_reva_regs_t *adc);
int MXC_ADC_RevA_Busy(mxc_adc_reva_regs_t *adc);
void MXC_ADC_RevA_EnableInt(mxc_adc_reva_regs_t *adc, uint32_t flags);
void MXC_ADC_RevA_DisableInt(mxc_adc_reva_regs_t *adc, uint32_t flags);
int MXC_ADC_RevA_GetFlags(mxc_adc_reva_regs_t *adc);
void MXC_ADC_RevA_ClearFlags(mxc_adc_reva_regs_t *adc, uint32_t flags);
int MXC_ADC_RevA_SetConversionSpeed(mxc_adc_reva_regs_t *adc, uint32_t hz);
int MXC_ADC_RevA_GetConversionSpeed(uint8_t divider);
void MXC_ADC_RevA_SetDataAlignment(mxc_adc_reva_regs_t *adc, int msbJustify);
void MXC_ADC_RevA_SetExtScale(mxc_adc_reva_regs_t *adc, mxc_adc_scale_t scale);
void MXC_ADC_RevA_EnableMonitor(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_DisableMonitor(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_SetMonitorHighThreshold(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor,
                                          uint32_t threshold);
int MXC_ADC_RevA_GetMonitorHighThreshold(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_SetMonitorLowThreshold(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor,
                                         uint32_t threshold);
int MXC_ADC_RevA_GetMonitorLowThreshold(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_SetMonitorChannel(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor,
                                    mxc_adc_chsel_t channel);
int MXC_ADC_RevA_GetMonitorChannel(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_EnableMonitorAsync(mxc_adc_monitor_t monitor, mxc_adc_monitor_cb_t callback);
void MXC_ADC_RevA_DisableMonitorAsync(mxc_adc_monitor_t monitor);
int MXC_ADC_RevA_StartConversion(mxc_adc_reva_regs_t *adc, mxc_adc_chsel_t channel);
int MXC_ADC_RevA_StartConversionAsync(mxc_adc_reva_regs_t *adc, mxc_adc_chsel_t channel,
                                      mxc_adc_complete_cb_t callback);
int MXC_ADC_RevA_StartConversionDMA(mxc_adc_reva_regs_t *adc, mxc_adc_chsel_t channel,
                                    mxc_dma_regs_t *dma, uint16_t *data,
                                    void (*callback)(int, int));
int MXC_ADC_RevA_Handler(mxc_adc_reva_regs_t *adc);
int MXC_ADC_RevA_Convert(mxc_adc_reva_regs_t *adc, mxc_adc_conversion_req_t *req);
int MXC_ADC_RevA_ConvertAsync(mxc_adc_reva_regs_t *adc, mxc_adc_conversion_req_t *req);
void MXC_ADC_RevA_Monitor(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_req_t req);
void MXC_ADC_RevA_MonitorAsync(mxc_adc_reva_regs_t *adc, mxc_adc_monitor_req_t req);
int MXC_ADC_RevA_GetData(mxc_adc_reva_regs_t *adc, uint16_t *outdata);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_ADC_ADC_REVA_H_
