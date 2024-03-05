/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SMON_SMON_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SMON_SMON_REVA_H_

#include <stddef.h>
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "smon.h"
#include "smon_reva_regs.h"

int MXC_SMON_RevA_ExtSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_ext_cfg_t *cfg,
                                  uint32_t delay);

int MXC_SMON_RevA_SetSensorFrequency(mxc_smon_reva_regs_t *smon, mxc_smon_ext_cfg_t *cfg);

int MXC_SMON_RevA_SetErrorCount(mxc_smon_reva_regs_t *smon, uint8_t errorCount);

int MXC_SMON_RevA_TempSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_temp_t threshold,
                                   uint32_t delay);

int MXC_SMON_RevA_SetTempThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_temp_t threshold);

int MXC_SMON_RevA_VoltageMonitorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_vtm_t threshold,
                                       uint32_t delay);

int MXC_SMON_RevA_SetVTMThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_vtm_t threshold);

int MXC_SMON_RevA_ActiveDieShieldEnable(mxc_smon_reva_regs_t *smon, uint32_t delay);

int MXC_SMON_RevA_SelfDestructByteEnable(mxc_smon_reva_regs_t *smon, mxc_smon_ext_cfg_t *cfg,
                                         uint32_t delay);

void MXC_SMON_RevA_EnablePUFTrimErase(mxc_smon_reva_regs_t *smon);

void MXC_SMON_RevA_DisablePUFTrimErase(mxc_smon_reva_regs_t *smon);

int MXC_SMON_RevA_DigitalFaultDetectorEnable(mxc_smon_reva_regs_t *smon,
                                             mxc_smon_interrupt_mode_t interruptMode,
                                             mxc_smon_lowpower_mode_t lowPowerMode,
                                             uint32_t delay);

uint32_t MXC_SMON_RevA_GetFlags(mxc_smon_reva_regs_t *smon);

void MXC_SMON_RevA_ClearTamper(mxc_smon_reva_regs_t *smon);

// Deprecated implementation - must clear all flags to clear alarm. Please
//  use MXC_SMON_RevA_ClearTamper instead.
void MXC_SMON_RevA_ClearFlags(mxc_smon_reva_regs_t *smon, uint32_t flags);

void MXC_SMON_RevA_ExtSensorLock(mxc_smon_reva_regs_t *smon);

void MXC_SMON_RevA_IntSensorLock(mxc_smon_reva_regs_t *smon);

int MXC_SMON_RevA_isBusy(mxc_smon_reva_regs_t *smon, mxc_smon_busy_t reg, uint32_t delay);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SMON_SMON_REVA_H_
