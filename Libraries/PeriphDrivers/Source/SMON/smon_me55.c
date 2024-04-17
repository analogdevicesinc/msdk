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

#include <stdint.h>
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

int MXC_SMON_EnableExtTampers(mxc_smon_ext_tampen_t *extTamp, bool lock)
{
    return MXC_SMON_RevA_EnableExtTampers((mxc_smon_reva_regs_t *)MXC_SMON, extTamp, lock);
}

int MXC_SMON_EnableIntTampers(mxc_smon_int_tampen_t *intTamp, bool lock)
{
    return MXC_SMON_RevA_EnableIntTampers((mxc_smon_reva_regs_t *)MXC_SMON, intTamp, lock);
}

int MXC_SMON_ExtSensorEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay)
{
    return MXC_SMON_RevA_ExtSensorEnable((mxc_smon_reva_regs_t *)MXC_SMON, cfg, delay);
}

int MXC_SMON_SetSensorFrequency(mxc_smon_ext_cfg_t *cfg)
{
    return MXC_SMON_RevA_SetSensorFrequency((mxc_smon_reva_regs_t *)MXC_SMON, cfg);
}

int MXC_SMON_SetErrorCount(uint8_t errorCount)
{
    return MXC_SMON_RevA_SetErrorCount((mxc_smon_reva_regs_t *)MXC_SMON, errorCount);
}

int MXC_SMON_TempSensorEnable(mxc_smon_temp_t threshold, uint32_t delay)
{
    return MXC_SMON_RevA_TempSensorEnable((mxc_smon_reva_regs_t *)MXC_SMON, threshold, delay);
}

int MXC_SMON_SetTempThreshold(mxc_smon_temp_t threshold)
{
    return MXC_SMON_RevA_SetTempThreshold((mxc_smon_reva_regs_t *)MXC_SMON, threshold);
}

int MXC_SMON_VoltageMonitorEnable(mxc_smon_vtm_t threshold, uint32_t delay)
{
    return MXC_SMON_RevA_VoltageMonitorEnable((mxc_smon_reva_regs_t *)MXC_SMON, threshold, delay);
}

int MXC_SMON_SetVTMThreshold(mxc_smon_vtm_t threshold)
{
    return MXC_SMON_RevA_SetVTMThreshold((mxc_smon_reva_regs_t *)MXC_SMON, threshold);
}

int MXC_SMON_ActiveDieShieldEnable(uint32_t delay)
{
    return MXC_SMON_RevA_ActiveDieShieldEnable((mxc_smon_reva_regs_t *)MXC_SMON, delay);
}

int MXC_SMON_SelfDestructByteEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay)
{
    return MXC_SMON_RevA_SelfDestructByteEnable((mxc_smon_reva_regs_t *)MXC_SMON, cfg, delay);
}

int MXC_SMON_EnablePUFTrimErase()
{
    return E_NOT_SUPPORTED;
}

int MXC_SMON_DisablePUFTrimErase()
{
    return E_NOT_SUPPORTED;
}

int MXC_SMON_DigitalFaultDetectorEnable(mxc_smon_interrupt_mode_t interruptMode,
                                        mxc_smon_lowpower_mode_t lowPowerMode, uint32_t delay)
{
    return MXC_SMON_RevA_DigitalFaultDetectorEnable((mxc_smon_reva_regs_t *)MXC_SMON, interruptMode,
                                                    lowPowerMode, delay);
}

uint32_t MXC_SMON_GetFlags(void)
{
    return MXC_SMON_RevA_GetFlags((mxc_smon_reva_regs_t *)MXC_SMON);
}

void MXC_SMON_ClearTamper(void)
{
    MXC_SMON_RevA_ClearTamper((mxc_smon_reva_regs_t *)MXC_SMON);
}

// Deprecated implementation - Must check and clear all flags to clear
//  tamper alarm. Please use MXC_SMON_ClearTamper instead.
void MXC_SMON_ClearFlags(uint32_t flags)
{
    MXC_SMON_RevA_ClearTamper((mxc_smon_reva_regs_t *)MXC_SMON);
}

uint32_t MXC_SMON_GetAlarms()
{
    return MXC_SMON_RevA_GetAlarms((mxc_smon_reva_regs_t *)MXC_SMON);
}

uint32_t MXC_SMON_GetDiagnostics()
{
    return MXC_SMON_RevA_GetDiagnostics((mxc_smon_reva_regs_t *)MXC_SMON);
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
    return MXC_SMON_RevA_isBusy((mxc_smon_reva_regs_t *)MXC_SMON, reg, delay);
}
