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
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "smon_reva.h"

int MXC_SMON_RevA_ExtSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_ext_cfg_t *cfg,
                                  uint32_t delay)
{
    int err;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    if ((err = MXC_SMON_SetSensorFrequency((mxc_smon_ext_cfg_t *)cfg)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_SMON_SetErrorCount(cfg->errorCount)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    //Enable external sensor
    smon->extsctrl |= cfg->sensorNumber;

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetSensorFrequency(mxc_smon_reva_regs_t *smon, mxc_smon_ext_cfg_t *cfg)
{
    int err;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    smon->extsctrl |= (cfg->clockDivide | cfg->freqDivide);

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetErrorCount(mxc_smon_reva_regs_t *smon, uint8_t errorCount)
{
    int err;

    if (errorCount > 31) {
        return E_BAD_PARAM;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    smon->extsctrl &= ~MXC_F_SMON_REVA_EXTSCTRL_ERRCNT;
    smon->extsctrl |= errorCount << MXC_F_SMON_REVA_EXTSCTRL_ERRCNT_POS;

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_TempSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_temp_t threshold, uint32_t delay)
{
    int err;

    if ((err = MXC_SMON_SetTempThreshold((mxc_smon_temp_t)threshold)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_TEMP_EN; //Enable Sensor

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetTempThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_temp_t threshold)
{
    int err;

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    if (threshold == MXC_SMON_TEMP_THD_NEG_50) {
        smon->intsctrl &= ~MXC_F_SMON_REVA_INTSCTRL_LOTEMP_SEL;
    } else if (threshold == MXC_SMON_TEMP_THD_NEG_30) {
        smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_LOTEMP_SEL;
    } else {
        return E_BAD_PARAM;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_VoltageMonitorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_vtm_t threshold,
                                       uint32_t delay)
{
    int err;

    if ((err = MXC_SMON_SetVTMThreshold(threshold)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_VBAT_EN; //Enable Sensor

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetVTMThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_vtm_t threshold)
{
    int err;

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    if (threshold == MXC_SMON_VTM_THD_1_6) {
        smon->intsctrl &= ~(MXC_F_SMON_REVA_INTSCTRL_VCORELO_EN | MXC_F_SMON_REVA_INTSCTRL_VCOREHI_EN);
    } else if (threshold == MXC_SMON_VTM_THD_2_2) {
        smon->intsctrl &= ~MXC_F_SMON_REVA_INTSCTRL_VCOREHI_EN;
        smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_VCORELO_EN;
    } else if (threshold == MXC_SMON_VTM_THD_2_8) {
        smon->intsctrl |= (MXC_F_SMON_REVA_INTSCTRL_VCORELO_EN | MXC_F_SMON_REVA_INTSCTRL_VCOREHI_EN);
    } else {
        return E_BAD_PARAM;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_ActiveDieShieldEnable(mxc_smon_reva_regs_t *smon, uint32_t delay)
{
    int err;

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_SHIELD_EN; //Enable Sensor

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SelfDestructByteEnable(mxc_smon_reva_regs_t *smon, mxc_smon_ext_cfg_t *cfg,
                                         uint32_t delay)
{
    int err;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    smon->sdbe &= ~MXC_F_SMON_REVA_SDBE_SDBYTE_EN;

    smon->sdbe |= cfg->data << MXC_F_SMON_REVA_SDBE_SDBYTE_POS;

    if ((err = MXC_SMON_ExtSensorEnable(cfg, delay)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    smon->sdbe |= MXC_F_SMON_REVA_SDBE_SDBYTE_EN;

    if ((err = MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

void MXC_SMON_RevA_EnablePUFTrimErase(mxc_smon_reva_regs_t *smon)
{
    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0);

    smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_PUF_TRIM_ERASE;

    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0);
}

void MXC_SMON_RevA_DisablePUFTrimErase(mxc_smon_reva_regs_t *smon)
{
    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0);

    smon->intsctrl &= ~MXC_F_SMON_REVA_INTSCTRL_PUF_TRIM_ERASE;

    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0);
}

int MXC_SMON_RevA_DigitalFaultDetectorEnable(mxc_smon_reva_regs_t *smon,
                                             mxc_smon_interrupt_mode_t interruptMode,
                                             mxc_smon_lowpower_mode_t lowPowerMode,
                                             uint32_t delay)
{
    int err;

    if ((err = MXC_SMON_isBusy(MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    if (interruptMode == MXC_SMON_DFD_INTR_NMI) {
        smon->intsctrl &= ~MXC_F_SMON_REVA_INTSCTRL_DFD_STDBY;
    } else if (interruptMode == MXC_SMON_DFD_INTR_PFW) {
        smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_DFD_STDBY;
    } else {
        return E_BAD_PARAM;
    }

    if (lowPowerMode == MXC_SMON_LP_DFD_ENABLE) {
        smon->intsctrl &= ~MXC_F_SMON_REVA_INTSCTRL_DFD_NMI_EN;
    } else if (lowPowerMode == MXC_SMON_LP_DFD_DISABLE) {
        smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_DFD_NMI_EN;
    } else {
        return E_BAD_PARAM;
    }

    smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_DFD_EN; //Enable DFD

    if ((err = MXC_SMON_isBusy(MXC_SMON_BUSY_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

uint32_t MXC_SMON_RevA_GetFlags(mxc_smon_reva_regs_t *smon)
{
    return smon->secalm;
}

void MXC_SMON_RevA_ClearTamper(mxc_smon_reva_regs_t *smon)
{
    // Must check and clear all flags to clear tamper alarm or
    //  else the device misses the NMI-DRS event.
    while (smon->secalm != 0) {
        // Must ensure no SMON register is busy before clearing flags. 
        MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_ALL, 0);

        // Design implentation calls for clearing all flags.
        smon->secalm = 0;

        MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_ALL, 0);
    }
}

// Deprecated implementation - Must clear all flags to clear alarm. Please use 
//  MXC_SMON_Reva_ClearTamper instead.
void MXC_SMON_RevA_ClearFlags(mxc_smon_reva_regs_t *smon, uint32_t flags)
{
    // Must ensure no SMON register is busy before clearing flags. 
    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_ALL, 0);

    // Design implentation calls for clearing all flags.
    smon->secalm &= ~flags;

    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_ALL, 0);
}

void MXC_SMON_RevA_ExtSensorLock(mxc_smon_reva_regs_t *smon)
{
    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, 0);

    smon->extsctrl |= MXC_F_SMON_REVA_EXTSCTRL_LOCK;

    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_EXTSENSOR, 0);
}

void MXC_SMON_RevA_IntSensorLock(mxc_smon_reva_regs_t *smon)
{
    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0);

    smon->intsctrl |= MXC_F_SMON_REVA_INTSCTRL_LOCK;

    MXC_SMON_RevA_isBusy(smon, MXC_SMON_BUSY_INTSENSOR, 0);
}

// Ensure Proper Writes by calling this function before and after each SMON register write.
//  An application may directly write to an SMON register before calling any of these SMON 
//  functions - which could result in an improper register write. 
int MXC_SMON_RevA_isBusy(mxc_smon_reva_regs_t *smon, mxc_smon_busy_t reg, uint32_t delay)
{
    if (delay == 0) {
        while (smon->secst & reg) {}

        return E_NO_ERROR;
    }

    MXC_Delay(delay);

    if (smon->secst & reg) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}
