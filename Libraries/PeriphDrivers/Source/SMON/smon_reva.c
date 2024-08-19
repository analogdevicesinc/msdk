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

int MXC_SMON_RevA_ExtSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_ext_cfg_t *cfg,
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

    //Enable external sensor
    smon->extscn |= cfg->sensorNumber;

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_EXTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetSensorFrequency(mxc_smon_reva_regs_t *smon, mxc_smon_reva_ext_cfg_t *cfg)
{
    int err;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    smon->extscn |= (cfg->clockDivide | cfg->freqDivide);

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_EXTSENSOR, 0)) != E_NO_ERROR) {
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

    smon->extscn &= ~MXC_F_SMON_REVA_EXTSCN_EXTCNT;
    smon->extscn |= errorCount << MXC_F_SMON_REVA_EXTSCN_EXTCNT_POS;

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_EXTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_TempSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_temp_t threshold,
                                   uint32_t delay)
{
    int err;

    if ((err = MXC_SMON_SetTempThreshold((mxc_smon_temp_t)threshold)) != E_NO_ERROR) {
        return err;
    }

    smon->intscn |= MXC_F_SMON_REVA_INTSCN_TEMP_EN; //Enable Sensor

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetTempThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_reva_temp_t threshold)
{
    int err;

    if (threshold == SMON_REVA_TEMP_THRESHOLD_NEG_50) {
        smon->intscn &= ~MXC_F_SMON_REVA_INTSCN_LOTEMP_SEL;
    } else if (threshold == SMON_REVA_TEMP_THRESHOLD_NEG_30) {
        smon->intscn |= MXC_F_SMON_REVA_INTSCN_LOTEMP_SEL;
    } else {
        return E_BAD_PARAM;
    }

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_VoltageMonitorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_vtm_t threshold,
                                       uint32_t delay)
{
    int err;

    if ((err = MXC_SMON_SetVTMThreshold((mxc_smon_vtm_t)threshold)) != E_NO_ERROR) {
        return err;
    }

    smon->intscn |= MXC_F_SMON_REVA_INTSCN_VBAT_EN; //Enable Sensor

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SetVTMThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_reva_vtm_t threshold)
{
    int err;

    if (threshold == SMON_REVA_VTM_THRESHOLD_1_6) {
        smon->intscn &= ~(MXC_F_SMON_REVA_INTSCN_VCORELOEN | MXC_F_SMON_REVA_INTSCN_VCOREHIEN);
    } else if (threshold == SMON_REVA_VTM_THRESHOLD_2_2) {
        smon->intscn &= ~MXC_F_SMON_REVA_INTSCN_VCOREHIEN;
        smon->intscn |= MXC_F_SMON_REVA_INTSCN_VCORELOEN;
    } else if (threshold == SMON_REVA_VTM_THRESHOLD_2_8) {
        smon->intscn |= (MXC_F_SMON_REVA_INTSCN_VCORELOEN | MXC_F_SMON_REVA_INTSCN_VCOREHIEN);
    } else {
        return E_BAD_PARAM;
    }

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, 0)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_ActiveDieShieldEnable(mxc_smon_reva_regs_t *smon, uint32_t delay)
{
    int err;

    smon->intscn |= MXC_F_SMON_REVA_INTSCN_SHIELD_EN; //Enable Sensor

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

int MXC_SMON_RevA_SelfDestructByteEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_ext_cfg_t *cfg,
                                         uint32_t delay)
{
    int err;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    smon->sdbe &= ~MXC_F_SMON_REVA_SDBE_SBDEN;

    smon->sdbe |= cfg->data << MXC_F_SMON_REVA_SDBE_DBYTE_POS;

    if ((err = MXC_SMON_ExtSensorEnable((mxc_smon_ext_cfg_t *)cfg, delay)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    smon->sdbe |= MXC_F_SMON_REVA_SDBE_SBDEN;

    return err;
}

void MXC_SMON_RevA_EnablePUFTrimErase(mxc_smon_reva_regs_t *smon)
{
    smon->intscn |= MXC_F_SMON_REVA_INTSCN_PUF_TRIM_ERASE;

    MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, 0);
}

void MXC_SMON_RevA_DisablePUFTrimErase(mxc_smon_reva_regs_t *smon)
{
    smon->intscn &= ~MXC_F_SMON_REVA_INTSCN_PUF_TRIM_ERASE;

    MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, 0);
}

int MXC_SMON_RevA_DigitalFaultDetectorEnable(mxc_smon_reva_regs_t *smon,
                                             mxc_smon_reva_interrupt_mode_t interruptMode,
                                             mxc_smon_reva_lowpower_mode_t lowPowerMode,
                                             uint32_t delay)
{
    int err;

    if (interruptMode == SMON_REVA_DFD_INTERRUPT_NMI) {
        smon->intscn &= ~MXC_F_SMON_REVA_INTSCN_DFD_STDBY;
    } else if (interruptMode == SMON_REVA_DFD_INTERRUPT_PFW) {
        smon->intscn |= MXC_F_SMON_REVA_INTSCN_DFD_STDBY;
    } else {
        return E_BAD_PARAM;
    }

    if (lowPowerMode == SMON_REVA_DFD_LOWPOWER_ENABLE) {
        smon->intscn &= ~MXC_F_SMON_REVA_INTSCN_DFD_NMI;
    } else if (lowPowerMode == SMON_REVA_DFD_LOWPOWER_DISABLE) {
        smon->intscn |= MXC_F_SMON_REVA_INTSCN_DFD_NMI;
    } else {
        return E_BAD_PARAM;
    }

    smon->intscn |= MXC_F_SMON_REVA_INTSCN_DFD_EN; //Enable DFD

    if ((err = MXC_SMON_isBusy((mxc_smon_busy_t)SMON_REVA_INTSENSOR, delay)) != E_NO_ERROR) {
        return err;
    }

    return err;
}

uint32_t MXC_SMON_RevA_GetFlags(mxc_smon_reva_regs_t *smon)
{
    return smon->secalm;
}

void MXC_SMON_RevA_ClearFlags(mxc_smon_reva_regs_t *smon, uint32_t flags)
{
    MXC_SMON_RevA_isBusy(smon, SMON_REVA_SECALARM, 0);
    smon->secalm &= ~flags;
    MXC_SMON_RevA_isBusy(smon, SMON_REVA_SECALARM, 0);
}

void MXC_SMON_RevA_ExtSensorLock(mxc_smon_reva_regs_t *smon)
{
    smon->extscn |= MXC_F_SMON_REVA_EXTSCN_LOCK;
}

void MXC_SMON_RevA_IntSensorLock(mxc_smon_reva_regs_t *smon)
{
    smon->intscn |= MXC_F_SMON_REVA_INTSCN_LOCK;
}

int MXC_SMON_RevA_isBusy(mxc_smon_reva_regs_t *smon, mxc_smon_reva_busy_t reg, uint32_t delay)
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
