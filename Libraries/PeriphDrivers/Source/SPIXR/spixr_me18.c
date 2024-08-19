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

/****** Includes *******/
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "spixr.h"
#include "spixr_reva.h"

/****** Functions ******/
int MXC_SPIXR_ReadRXFIFO(uint8_t *buf, int len)
{
    return MXC_SPIXR_RevA_ReadRXFIFO((mxc_spixr_reva_regs_t *)MXC_SPIXR, buf, len);
}

int MXC_SPIXR_WriteTXFIFO(uint8_t *buf, int len)
{
    return MXC_SPIXR_RevA_WriteTXFIFO((mxc_spixr_reva_regs_t *)MXC_SPIXR, buf, len);
}

void MXC_SPIXR_SetSS(int ssIdx)
{
    MXC_SPIXR_RevA_SetSS((mxc_spixr_reva_regs_t *)MXC_SPIXR, ssIdx);
}

int MXC_SPIXR_GetSS(void)
{
    return MXC_SPIXR_RevA_GetSS((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_SetSSCtrl(int stayActive)
{
    MXC_SPIXR_RevA_SetSSCtrl((mxc_spixr_reva_regs_t *)MXC_SPIXR, stayActive);
}

int MXC_SPIXR_GetSSCtrl(void)
{
    return MXC_SPIXR_RevA_GetSSCtrl((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_Enable(void)
{
    MXC_SPIXR_RevA_Enable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_Disable(void)
{
    MXC_SPIXR_RevA_Disable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_IsEnabled(void)
{
    return MXC_SPIXR_RevA_IsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_ThreeWireModeEnable(void)
{
    MXC_SPIXR_RevA_ThreeWireModeEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_ThreeWireModeDisable(void)
{
    MXC_SPIXR_RevA_ThreeWireModeDisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_ThreeWireModeIsEnabled(void)
{
    return MXC_SPIXR_RevA_ThreeWireModeIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_GetTXFIFOCount(void)
{
    return MXC_SPIXR_RevA_GetTXFIFOCount((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_GetRXFIFOCount(void)
{
    return MXC_SPIXR_RevA_GetRXFIFOCount((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_SetWidth(mxc_spixr_width_t width)
{
    return MXC_SPIXR_RevA_SetWidth((mxc_spixr_reva_regs_t *)MXC_SPIXR,
                                   (mxc_spixr_reva_width_t)width);
}

int MXC_SPIXR_SetSPIMode(mxc_spixr_mode_t mode)
{
    return MXC_SPIXR_RevA_SetSPIMode((mxc_spixr_reva_regs_t *)MXC_SPIXR,
                                     (mxc_spixr_reva_mode_t)mode);
}

int MXC_SPIXR_SetSSPolarity(int activeLow)
{
    return MXC_SPIXR_RevA_SetSSPolarity((mxc_spixr_reva_regs_t *)MXC_SPIXR, activeLow);
}

void MXC_SPIXR_SetSSTiming(unsigned int ssIActDelay, unsigned int postActive,
                           unsigned int preActive)
{
    MXC_SPIXR_RevA_SetSSTiming((mxc_spixr_reva_regs_t *)MXC_SPIXR, ssIActDelay, postActive,
                               preActive);
}

int MXC_SPIXR_SetFrequency(int hz)
{
    return MXC_SPIXR_RevA_SetFrequency((mxc_spixr_reva_regs_t *)MXC_SPIXR, hz);
}

int MXC_SPIXR_GetFrequency(void)
{
    return MXC_SPIXR_RevA_GetFrequency((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_GetIntFlags(void)
{
    return MXC_SPIXR_RevA_GetIntFlags((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_EnableInt(int flags)
{
    MXC_SPIXR_RevA_EnableInt((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

void MXC_SPIXR_DisableInt(int flags)
{
    MXC_SPIXR_RevA_DisableInt((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

int MXC_SPIXR_GetWakeUpFlags(void)
{
    return MXC_SPIXR_RevA_GetWakeUpFlags((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_EnableWakeUp(int flags)
{
    MXC_SPIXR_RevA_EnableWakeUp((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

void MXC_SPIXR_DisableWakeUp(int flags)
{
    MXC_SPIXR_RevA_DisableWakeUp((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

void MXC_SPIXR_ExMemEnable(void)
{
    MXC_SPIXR_RevA_ExMemEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_ExMemDisable(void)
{
    MXC_SPIXR_RevA_ExMemDisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_ExMemUseDummy(int delay255)
{
    MXC_SPIXR_RevA_ExMemUseDummy((mxc_spixr_reva_regs_t *)MXC_SPIXR, delay255);
}

void MXC_SPIXR_ExMemSetWriteCommand(uint8_t command)
{
    MXC_SPIXR_RevA_ExMemSetWriteCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR, command);
}

uint8_t MXC_SPIXR_ExMemGetWriteCommand(void)
{
    return MXC_SPIXR_RevA_ExMemGetWriteCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_ExMemSetReadCommand(uint8_t command)
{
    MXC_SPIXR_RevA_ExMemSetReadCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR, command);
}

uint8_t MXC_SPIXR_ExMemGetReadCommand(void)
{
    return MXC_SPIXR_RevA_ExMemGetReadCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_Busy(void)
{
    return MXC_SPIXR_RevA_Busy((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_Init(mxc_spixr_cfg_t *cfg)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SYSCACHE);

    /* The crypto clock needs to be turned on for crypto to work. */
    if ((MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IBRO_EN) == 0) {
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IBRO_EN;

        // Check if TPU clock is ready
        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IBRO_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIP);

    MXC_GPIO_Config(&gpio_cfg_spixr);

    return MXC_SPIXR_RevA_Init((mxc_spixr_reva_regs_t *)MXC_SPIXR, (mxc_spixr_reva_cfg_t *)cfg);
}

int MXC_SPIXR_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SYSCACHE);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIP);

    return MXC_SPIXR_RevA_Shutdown((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_SendCommand(uint8_t *cmd, uint32_t length, uint32_t tx_num_char)
{
    MXC_SPIXR_RevA_SendCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR, cmd, length, tx_num_char);
}

void MXC_SPIXR_TXFIFOEnable(void)
{
    MXC_SPIXR_RevA_TXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_TXFIFODisable(void)
{
    MXC_SPIXR_RevA_TXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_TXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_TXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_DmaTXFIFOEnable(void)
{
    MXC_SPIXR_RevA_DmaTXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_DmaTXFIFODisable(void)
{
    MXC_SPIXR_RevA_DmaTXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_DmaTXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_DmaTXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_RXFIFOEnable(void)
{
    MXC_SPIXR_RevA_RXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_RXFIFODisable(void)
{
    MXC_SPIXR_RevA_RXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_RXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_RXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_DmaRXFIFOEnable(void)
{
    MXC_SPIXR_RevA_DmaRXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_DmaRXFIFODisable(void)
{
    MXC_SPIXR_RevA_DmaRXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

int MXC_SPIXR_DmaRXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_DmaRXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_TXFIFOClear(void)
{
    MXC_SPIXR_RevA_TXFIFOClear((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

void MXC_SPIXR_RXFIFOClear(void)
{
    MXC_SPIXR_RevA_RXFIFOClear((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}
