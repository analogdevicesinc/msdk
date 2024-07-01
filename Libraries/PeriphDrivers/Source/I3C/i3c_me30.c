/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

/* **** Includes **** */
#include <string.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "i3c.h"
#include "i3c_reva.h"
#include "i3c_ccc.h"

/* **** Definitions **** */

/* **** Functions **** */
int MXC_I3C_Init(mxc_i3c_regs_t *i3c, int targetMode, uint8_t staticAddr, uint32_t ppHz,
    uint32_t odHz, uint32_t i2cHz)
{
    int ret;

#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_I3C);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I3C);
#endif

    if (!targetMode) {
        /* Controller mode initialization */
        /* 1. SCL frequency and duty cycle */
        ret = MXC_I3C_SetPPFrequency(i3c, ppHz);
        if (ret < 0) {
            return ret;
        }

        ret = MXC_I3C_SetODFrequency(i3c, odHz, false);
        if (ret < 0) {
            return ret;
        }

        ret = MXC_I3C_SetI2CFrequency(i3c, i2cHz);
        if (ret < 0) {
            return ret;
        }
    }

    return MXC_I3C_RevA_Init((mxc_i3c_reva_regs_t *)i3c, targetMode, staticAddr);
}

int MXC_I3C_Shutdown(mxc_i3c_regs_t *i3c)
{
    int ret;

    ret = MXC_I3C_RevA_Shutdown((mxc_i3c_reva_regs_t *)i3c);
    if (ret < 0) {
        return ret;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_I3C);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I3C);
#endif

    return E_SUCCESS;
}

int MXC_I3C_SetPPFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency)
{
    return MXC_I3C_RevA_SetPPFrequency((mxc_i3c_reva_regs_t *)i3c, frequency);
}

unsigned int MXC_I3C_GetPPFrequency(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_GetPPFrequency((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_SetODFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency, bool highPP)
{
    return MXC_I3C_RevA_SetODFrequency((mxc_i3c_reva_regs_t *)i3c, frequency, highPP);
}

unsigned int MXC_I3C_GetODFrequency(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_GetODFrequency((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_SetI2CFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency)
{
    return MXC_I3C_RevA_SetI2CFrequency((mxc_i3c_reva_regs_t *)i3c, frequency);
}

unsigned int MXC_I3C_GetI2CFrequency(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_GetI2CFrequency((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_SetSkew(mxc_i3c_regs_t *i3c, uint8_t skew)
{
    return MXC_I3C_RevA_SetSkew((mxc_i3c_reva_regs_t *)i3c, skew);
}

int MXC_I3C_SetHighKeeperMode(mxc_i3c_regs_t *i3c, mxc_i3c_high_keeper_t hkeep)
{
    return MXC_I3C_RevA_SetHighKeeperMode((mxc_i3c_reva_regs_t *)i3c, hkeep);
}

int MXC_I3C_SetI3CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *targets, uint8_t numTargets)
{
    return MXC_I3C_RevA_SetI3CTargets((mxc_i3c_reva_regs_t *)i3c, (mxc_i3c_reva_target_t *)targets,
                                      numTargets);
}

int MXC_I3C_SetI2CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_i2c_target_t *targets, uint8_t numTargets)
{
    return MXC_I3C_RevA_SetI2CTargets((mxc_i3c_reva_regs_t *)i3c,
                                      (mxc_i3c_reva_i2c_target_t *)targets, numTargets);
}

void MXC_I3C_SetIBICallback(mxc_i3c_regs_t *i3c, mxc_i3c_ibi_ack_t ackCb, mxc_i3c_ibi_req_t reqCb)
{
    MXC_I3C_RevA_SetIBICallback((mxc_i3c_reva_regs_t *)i3c, (mxc_i3c_reva_ibi_ack_t)ackCb,
                                (mxc_i3c_reva_ibi_req_t)reqCb);
}

void MXC_I3C_SetIBIPayloadCallback(mxc_i3c_regs_t *i3c, mxc_i3c_ibi_getbyte_t payloadCb)
{
    MXC_I3C_RevA_SetIBIPayloadCallback((mxc_i3c_reva_regs_t *)i3c,
                                       (mxc_i3c_reva_ibi_getbyte_t)payloadCb);
}

void MXC_I3C_SetCCCCallback(mxc_i3c_regs_t *i3c, mxc_i3c_ccc_cb_t cccCb)
{
    MXC_I3C_RevA_SetCCCCallback((mxc_i3c_reva_regs_t *)i3c, (mxc_i3c_reva_ccc_cb_t)cccCb);
}

int MXC_I3C_Start(mxc_i3c_regs_t *i3c, bool i2c, uint8_t readWrite,
        uint8_t addr, uint8_t readCount)
{
    return MXC_I3C_RevA_Start((mxc_i3c_reva_regs_t *)i3c, i2c, readWrite, addr, readCount);
}

int MXC_I3C_BroadcastCCC(mxc_i3c_regs_t *i3c, unsigned char ccc, int defByte, unsigned char *data,
                         int len)
{
    return MXC_I3C_RevA_BroadcastCCC((mxc_i3c_reva_regs_t *)i3c, ccc, defByte, data, len);
}

int MXC_I3C_PerformDAA(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_PerformDAA((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_HotJoin(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_HotJoin((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_RequestIBI(mxc_i3c_regs_t *i3c, unsigned char mdb, mxc_i3c_ibi_getbyte_t getByteCb)
{
    return MXC_I3C_RevA_RequestIBI((mxc_i3c_reva_regs_t *)i3c, mdb,
                                   (mxc_i3c_reva_ibi_getbyte_t)getByteCb);
}

int MXC_I3C_Standby(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Standby((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_Wakeup(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Wakeup((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_ReadI2CBlocking(mxc_i3c_regs_t *i3c, uint8_t staticAddr, unsigned char *bytes,
                            unsigned int *len)
{
    return MXC_I3C_RevA_ReadI2CBlocking((mxc_i3c_reva_regs_t *)i3c, staticAddr, bytes, len);
}

int MXC_I3C_WriteI2CBlocking(mxc_i3c_regs_t *i3c, unsigned char staticAddr, unsigned char *bytes,
                             unsigned int *len)
{
    return MXC_I3C_RevA_WriteI2CBlocking((mxc_i3c_reva_regs_t *)i3c, staticAddr, bytes, len);
}

int MXC_I3C_ReadSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes,
                            unsigned int *len)
{
    return MXC_I3C_RevA_ReadSDRBlocking((mxc_i3c_reva_regs_t *)i3c, dynAddr, bytes, len);
}

int MXC_I3C_WriteSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes,
                             unsigned int *len)
{
    return MXC_I3C_RevA_WriteSDRBlocking((mxc_i3c_reva_regs_t *)i3c, dynAddr, bytes, len);
}

void MXC_I3C_Stop(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Stop((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_I2CStop(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_I2CStop((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_ReadRXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len)
{
    return MXC_I3C_RevA_ReadRXFIFO((mxc_i3c_reva_regs_t *)i3c, bytes, len);
}

int MXC_I3C_WriteTXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len)
{
    return MXC_I3C_RevA_WriteTXFIFO((mxc_i3c_reva_regs_t *)i3c, bytes, len);
}

int MXC_I3C_GetError(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_GetError((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_ClearError(mxc_i3c_regs_t *i3c)
{
    MXC_I3C_RevA_ClearError((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_SetRXTXThreshold(mxc_i3c_regs_t *i3c, mxc_i3c_rx_threshold_t rxth,
                             mxc_i3c_tx_threshold_t txth)
{
    return MXC_I3C_RevA_SetRXTXThreshold((mxc_i3c_reva_regs_t *)i3c,
                                         (mxc_i3c_reva_rx_threshold_t)rxth,
                                         (mxc_i3c_reva_tx_threshold_t)txth);
}

uint8_t MXC_I3C_GetDynamicAddress(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_GetDynamicAddress((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_IRQHandler(mxc_i3c_regs_t *i3c)
{
    MXC_I3C_RevA_IRQHandler((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_ClearRXFIFO(mxc_i3c_regs_t *i3c)
{
    MXC_I3C_RevA_ClearRXFIFO((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_ClearTXFIFO(mxc_i3c_regs_t *i3c)
{
    MXC_I3C_RevA_ClearTXFIFO((mxc_i3c_reva_regs_t *)i3c);
}

unsigned int MXC_I3C_ControllerGetRXCount(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_ControllerGetRXCount((mxc_i3c_reva_regs_t *)i3c);
}

unsigned int MXC_I3C_ControllerGetTXCount(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_ControllerGetTXCount((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_ControllerEnableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_ControllerEnableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

void MXC_I3C_ControllerDisableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_ControllerDisableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

unsigned int MXC_I3C_ControllerGetFlags(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_ControllerGetFlags((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_ControllerClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_ControllerClearFlags((mxc_i3c_reva_regs_t *)i3c, mask);
}

void MXC_I3C_TargetEnableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_TargetEnableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

void MXC_I3C_TargetDisableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_TargetDisableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

int MXC_I3C_TargetGetFlags(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_TargetGetFlags((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_TargetClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_TargetClearFlags((mxc_i3c_reva_regs_t *)i3c, mask);
}
