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
int MXC_I3C_Init(mxc_i3c_regs_t *i3c, mxc_i3c_config_t *config)
{
    int ret;

#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_I3C);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I3C);
    MXC_GPIO_Config(&gpio_cfg_i3c);
#endif

    if (!config->target_mode) {
        /* Controller mode initialization */
        /* 1. SCL frequency and duty cycle */
        ret = MXC_I3C_SetPPFrequency(i3c, config->pp_hz);
        if (ret < 0) {
            return ret;
        }

        ret = MXC_I3C_SetODFrequency(i3c, config->od_hz, false);
        if (ret < 0) {
            return ret;
        }

        ret = MXC_I3C_SetI2CFrequency(i3c, config->i2c_hz);
        if (ret < 0) {
            return ret;
        }
    }

    return MXC_I3C_RevA_Init((mxc_i3c_reva_regs_t *)i3c, config->target_mode, config->static_addr);
}

int MXC_I3C_Recover(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Recover((mxc_i3c_reva_regs_t *)i3c);
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

int MXC_I3C_EmitStart(mxc_i3c_regs_t *i3c, bool isI2C, mxc_i3c_transfer_type_t xferType,
                      uint8_t addr, uint8_t readCount)
{
    return MXC_I3C_RevA_EmitStart((mxc_i3c_reva_regs_t *)i3c, isI2C, xferType, addr, readCount);
}

int MXC_I3C_ResetTarget(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_ResetTarget((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_EmitStop(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_EmitStop((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_EmitI2CStop(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_EmitI2CStop((mxc_i3c_reva_regs_t *)i3c);
}

int MXC_I3C_Controller_CCC(mxc_i3c_regs_t *i3c, const mxc_i3c_ccc_req_t *req)
{
    return MXC_I3C_RevA_Controller_CCC((mxc_i3c_reva_regs_t *)i3c, (mxc_i3c_reva_ccc_req_t *)req);
}

int MXC_I3C_Controller_Transaction(mxc_i3c_regs_t *i3c, const mxc_i3c_req_t *req)
{
    return MXC_I3C_RevA_Controller_Transaction((mxc_i3c_reva_regs_t *)i3c,
                                               (mxc_i3c_reva_req_t *)req);
}

int MXC_I3C_Controller_DAA(mxc_i3c_regs_t *i3c, uint8_t addr, uint8_t *pid, uint8_t *bcr,
                           uint8_t *dcr)
{
    return MXC_I3C_RevA_Controller_DAA((mxc_i3c_reva_regs_t *)i3c, addr, pid, bcr, dcr);
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

int MXC_I3C_ReadRXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len,
                       int timeout)
{
    return MXC_I3C_RevA_ReadRXFIFO((mxc_i3c_reva_regs_t *)i3c, bytes, len, timeout);
}

int MXC_I3C_WriteTXFIFO(mxc_i3c_regs_t *i3c, const unsigned char *bytes, unsigned int len, bool end,
                        int timeout)
{
    return MXC_I3C_RevA_WriteTXFIFO((mxc_i3c_reva_regs_t *)i3c, bytes, len, end, timeout);
}

int MXC_I3C_Controller_GetError(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Controller_GetError((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_Controller_ClearError(mxc_i3c_regs_t *i3c)
{
    MXC_I3C_RevA_Controller_ClearError((mxc_i3c_reva_regs_t *)i3c);
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

unsigned int MXC_I3C_Controller_GetRXCount(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Controller_GetRXCount((mxc_i3c_reva_regs_t *)i3c);
}

unsigned int MXC_I3C_Controller_GetTXCount(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Controller_GetTXCount((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_Controller_EnableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_Controller_EnableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

void MXC_I3C_Controller_DisableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_Controller_DisableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

unsigned int MXC_I3C_Controller_GetFlags(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Controller_GetFlags((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_Controller_ClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_Controller_ClearFlags((mxc_i3c_reva_regs_t *)i3c, mask);
}

void MXC_I3C_Target_EnableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_Target_EnableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

void MXC_I3C_Target_DisableInt(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_Target_DisableInt((mxc_i3c_reva_regs_t *)i3c, mask);
}

int MXC_I3C_Target_GetFlags(mxc_i3c_regs_t *i3c)
{
    return MXC_I3C_RevA_Target_GetFlags((mxc_i3c_reva_regs_t *)i3c);
}

void MXC_I3C_Target_ClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask)
{
    MXC_I3C_RevA_Target_ClearFlags((mxc_i3c_reva_regs_t *)i3c, mask);
}
