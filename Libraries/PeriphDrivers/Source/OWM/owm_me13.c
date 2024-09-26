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

/* **** Includes **** */
#include <string.h>
#include "owm_reva.h"

/* **** Definitions **** */
#define MXC_OWM_CLK_FREQ 1000000 //1-Wire requires 1MHz clock

/* **** Globals **** */

/* **** Functions **** */

int MXC_OWM_Init(const mxc_owm_cfg_t *cfg, sys_map_t map)
{
    int err = 0;
    uint32_t mxc_owm_clk, clk_div = 0;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    // Set system level configurations
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_OWIRE);

    const mxc_gpio_cfg_t *gpio;
    switch (map) {
    case MAP_A:
        gpio = &gpio_cfg_owm;
        break;
    case MAP_B:
        gpio = &gpio_cfg_owmb;
        break;
    default:
        gpio = &gpio_cfg_owm;
        break;
    }

    if ((err = MXC_GPIO_Config(gpio)) != E_NO_ERROR) {
        return err;
    }
#else
    (void)map;
#endif

    // Configure clk divisor to get 1MHz OWM clk
    mxc_owm_clk = PeripheralClock;

    if (mxc_owm_clk == 0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
        return E_UNINITIALIZED;
    }

    // Return error if clk doesn't divide evenly to 1MHz
    if (mxc_owm_clk % MXC_OWM_CLK_FREQ) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
        return E_NOT_SUPPORTED;
    }

    clk_div = (mxc_owm_clk / (MXC_OWM_CLK_FREQ));

    // Can not support lower frequencies
    if (clk_div == 0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
        return E_NOT_SUPPORTED;
    }

    err = MXC_OWM_RevA_Init((mxc_owm_reva_regs_t *)MXC_OWM, cfg);
    if (err == E_BAD_PARAM) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
    }

    return err;
}

void MXC_OWM_Shutdown(void)
{
    MXC_OWM_RevA_Shutdown((mxc_owm_reva_regs_t *)MXC_OWM);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
}

int MXC_OWM_Reset(void)
{
    return MXC_OWM_RevA_Reset((mxc_owm_reva_regs_t *)MXC_OWM);
}

int MXC_OWM_GetPresenceDetect(void)
{
    return (!!(MXC_OWM->ctrl_stat & MXC_F_OWM_CTRL_STAT_PRESENCE_DETECT));
}

int MXC_OWM_TouchByte(uint8_t data)
{
    return MXC_OWM_RevA_TouchByte((mxc_owm_reva_regs_t *)MXC_OWM, data);
}

int MXC_OWM_WriteByte(uint8_t data)
{
    return MXC_OWM_RevA_WriteByte(data);
}

int MXC_OWM_ReadByte(void)
{
    return MXC_OWM_RevA_ReadByte();
}

int MXC_OWM_TouchBit(uint8_t bit)
{
    return MXC_OWM_RevA_TouchBit((mxc_owm_reva_regs_t *)MXC_OWM, bit);
}

int MXC_OWM_WriteBit(uint8_t bit)
{
    return MXC_OWM_RevA_WriteBit(bit);
}

int MXC_OWM_ReadBit(void)
{
    return MXC_OWM_RevA_ReadBit();
}

int MXC_OWM_Write(uint8_t *data, int len)
{
    return MXC_OWM_RevA_Write((mxc_owm_reva_regs_t *)MXC_OWM, data, len);
}

int MXC_OWM_Read(uint8_t *data, int len)
{
    return MXC_OWM_RevA_Read((mxc_owm_reva_regs_t *)MXC_OWM, data, len);
}

int MXC_OWM_ReadROM(uint8_t *ROMCode)
{
    return MXC_OWM_RevA_ReadROM(ROMCode);
}

int MXC_OWM_MatchROM(uint8_t *ROMCode)
{
    return MXC_OWM_RevA_MatchROM(ROMCode);
}

int MXC_OWM_ODMatchROM(uint8_t *ROMCode)
{
    return MXC_OWM_RevA_ODMatchROM((mxc_owm_reva_regs_t *)MXC_OWM, ROMCode);
}

int MXC_OWM_SkipROM(void)
{
    return MXC_OWM_RevA_SkipROM();
}

int MXC_OWM_ODSkipROM(void)
{
    return MXC_OWM_RevA_ODSkipROM((mxc_owm_reva_regs_t *)MXC_OWM);
}

int MXC_OWM_Resume(void)
{
    return MXC_OWM_RevA_Resume();
}

int MXC_OWM_SearchROM(int newSearch, uint8_t *ROMCode)
{
    return MXC_OWM_RevA_SearchROM((mxc_owm_reva_regs_t *)MXC_OWM, newSearch, ROMCode);
}

void MXC_OWM_ClearFlags(uint32_t mask)
{
    MXC_OWM_RevA_ClearFlags((mxc_owm_reva_regs_t *)MXC_OWM, mask);
}

unsigned MXC_OWM_GetFlags(void)
{
    return MXC_OWM_RevA_GetFlags((mxc_owm_reva_regs_t *)MXC_OWM);
}

void MXC_OWM_SetExtPullup(int enable)
{
    MXC_OWM_RevA_SetExtPullup((mxc_owm_reva_regs_t *)MXC_OWM, enable);
}

void MXC_OWM_SetOverdrive(int enable) {}

void MXC_OWM_EnableInt(int flags)
{
    MXC_OWM_RevA_EnableInt((mxc_owm_reva_regs_t *)MXC_OWM, flags);
}

void MXC_OWM_DisableInt(int flags)
{
    MXC_OWM_RevA_DisableInt((mxc_owm_reva_regs_t *)MXC_OWM, flags);
}

int MXC_OWM_SetForcePresenceDetect(int enable)
{
    return MXC_OWM_RevA_SetForcePresenceDetect((mxc_owm_reva_regs_t *)MXC_OWM, enable);
}

int MXC_OWM_SetInternalPullup(int enable)
{
    return MXC_OWM_RevA_SetInternalPullup((mxc_owm_reva_regs_t *)MXC_OWM, enable);
}

int MXC_OWM_SetExternalPullup(mxc_owm_ext_pu_t ext_pu_mode)
{
    return MXC_OWM_RevA_SetExternalPullup((mxc_owm_reva_regs_t *)MXC_OWM, ext_pu_mode);
}

int MXC_OWM_SystemClockUpdated(void)
{
    return MXC_OWM_RevA_SystemClockUpdated((mxc_owm_reva_regs_t *)MXC_OWM);
}

int MXC_OWM_SetSearchROMAccelerator(int enable)
{
    return MXC_OWM_RevA_SetSearchROMAccelerator((mxc_owm_reva_regs_t *)MXC_OWM, enable);
}

int MXC_OWM_BitBang_Init(int initialState)
{
    return MXC_OWM_RevA_BitBang_Init((mxc_owm_reva_regs_t *)MXC_OWM, initialState);
}

int MXC_OWM_BitBang_Read(void)
{
    return MXC_OWM_RevA_BitBang_Read((mxc_owm_reva_regs_t *)MXC_OWM);
}

int MXC_OWM_BitBang_Write(int state)
{
    return MXC_OWM_RevA_BitBang_Write((mxc_owm_reva_regs_t *)MXC_OWM, state);
}

int MXC_OWM_BitBang_Disable(void)
{
    return MXC_OWM_RevA_BitBang_Disable((mxc_owm_reva_regs_t *)MXC_OWM);
}
