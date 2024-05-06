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

#include "smon_reva.h"
#include "smon.h"
#include "gpio.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"

void MXC_SMON_Init(void)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TRNG);
}

void MXC_SMON_Shutdown(void)
{
    // Not disabling TRNG clock as it may be used by other peripherals
    return;
}

int MXC_SMON_ExtSensorEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay)
{
    return MXC_SMON_RevA_ExtSensorEnable((mxc_smon_reva_regs_t *)MXC_SMON,
                                         (mxc_smon_reva_ext_cfg_t *)cfg, delay);
}

int MXC_SMON_SetSensorFrequency(mxc_smon_ext_cfg_t *cfg)
{
    return MXC_SMON_RevA_SetSensorFrequency((mxc_smon_reva_regs_t *)MXC_SMON,
                                            (mxc_smon_reva_ext_cfg_t *)cfg);
}

int MXC_SMON_SetErrorCount(uint8_t errorCount)
{
    return MXC_SMON_RevA_SetErrorCount((mxc_smon_reva_regs_t *)MXC_SMON, errorCount);
}

int MXC_SMON_TempSensorEnable(mxc_smon_temp_t threshold, uint32_t delay)
{
    return MXC_SMON_RevA_TempSensorEnable((mxc_smon_reva_regs_t *)MXC_SMON,
                                          (mxc_smon_reva_temp_t)threshold, delay);
}

int MXC_SMON_SetTempThreshold(mxc_smon_temp_t threshold)
{
    return MXC_SMON_RevA_SetTempThreshold((mxc_smon_reva_regs_t *)MXC_SMON,
                                          (mxc_smon_reva_temp_t)threshold);
}

int MXC_SMON_VoltageMonitorEnable(mxc_smon_vtm_t threshold, uint32_t delay)
{
    return MXC_SMON_RevA_VoltageMonitorEnable((mxc_smon_reva_regs_t *)MXC_SMON,
                                              (mxc_smon_reva_temp_t)threshold, delay);
}

int MXC_SMON_SetVTMThreshold(mxc_smon_vtm_t threshold)
{
    return MXC_SMON_RevA_SetVTMThreshold((mxc_smon_reva_regs_t *)MXC_SMON,
                                         (mxc_smon_reva_temp_t)threshold);
}

int MXC_SMON_ActiveDieShieldEnable(uint32_t delay)
{
    return MXC_SMON_RevA_ActiveDieShieldEnable((mxc_smon_reva_regs_t *)MXC_SMON, delay);
}

int MXC_SMON_SelfDestructByteEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay)
{
    return MXC_SMON_RevA_SelfDestructByteEnable((mxc_smon_reva_regs_t *)MXC_SMON,
                                                (mxc_smon_reva_ext_cfg_t *)cfg, delay);
}

void MXC_SMON_EnablePUFTrimErase(void)
{
    MXC_SMON_RevA_EnablePUFTrimErase((mxc_smon_reva_regs_t *)MXC_SMON);
}

void MXC_SMON_DisablePUFTrimErase(void)
{
    MXC_SMON_RevA_DisablePUFTrimErase((mxc_smon_reva_regs_t *)MXC_SMON);
}

int MXC_SMON_DigitalFaultDetectorEnable(mxc_smon_interrupt_mode_t interruptMode,
                                        mxc_smon_lowpower_mode_t lowPowerMode, uint32_t delay)
{
    return MXC_SMON_RevA_DigitalFaultDetectorEnable((mxc_smon_reva_regs_t *)MXC_SMON,
                                                    (mxc_smon_reva_interrupt_mode_t)interruptMode,
                                                    (mxc_smon_lowpower_mode_t)lowPowerMode, delay);
}

uint32_t MXC_SMON_GetFlags(void)
{
    return MXC_SMON_RevA_GetFlags((mxc_smon_reva_regs_t *)MXC_SMON);
}

void MXC_SMON_ClearFlags(uint32_t flags)
{
    MXC_SMON_RevA_ClearFlags((mxc_smon_reva_regs_t *)MXC_SMON, flags);
}

void MXC_SMON_ExtSensorLock(void)
{
    MXC_SMON_RevA_ExtSensorLock((mxc_smon_reva_regs_t *)MXC_SMON);
}

void MXC_SMON_IntSensorLock(void)
{
    MXC_SMON_RevA_IntSensorLock((mxc_smon_reva_regs_t *)MXC_SMON);
}

int MXC_SMON_isBusy(mxc_smon_busy_t reg, uint32_t delay)
{
    return MXC_SMON_RevA_isBusy((mxc_smon_reva_regs_t *)MXC_SMON, (mxc_smon_reva_busy_t)reg, delay);
}
