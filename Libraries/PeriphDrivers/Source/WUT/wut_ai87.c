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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "wut.h"
#include "wut_reva.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Local Variables **** */

/* **** Functions **** */

/* ************************************************************************** */
void MXC_WUT_Init(mxc_wut_pres_t pres)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);
#endif
    MXC_WUT_RevA_Init((mxc_wut_reva_regs_t *)MXC_WUT, pres);
}

void MXC_WUT_Shutdown(void)
{
    MXC_WUT_RevA_Shutdown((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_Enable(void)
{
    MXC_WUT_RevA_Enable((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_Disable(void)
{
    MXC_WUT_RevA_Disable((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_Config(const mxc_wut_cfg_t *cfg)
{
    MXC_WUT_RevA_Config((mxc_wut_reva_regs_t *)MXC_WUT, (mxc_wut_reva_cfg_t *)cfg);
}

/* ************************************************************************** */
uint32_t MXC_WUT_GetCompare(void)
{
    return MXC_WUT_RevA_GetCompare((mxc_wut_reva_regs_t *)MXC_WUT);
}

///* ************************************************************************** */
//uint32_t MXC_WUT_GetCapture(void)
//{
//    return MXC_WUT_RevA_GetCapture((mxc_wut_reva_regs_t*) MXC_WUT);
//}

/* ************************************************************************* */
uint32_t MXC_WUT_GetCount(void)
{
    return MXC_WUT_RevA_GetCount((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************* */
void MXC_WUT_IntClear(void)
{
    MXC_WUT_RevA_IntClear((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************* */
void MXC_WUT_ClearFlags(void)
{
    MXC_WUT_RevA_IntClear((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************* */
uint32_t MXC_WUT_IntStatus(void)
{
    return MXC_WUT_RevA_IntStatus((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************* */
uint32_t MXC_WUT_GetFlags(void)
{
    return MXC_WUT_RevA_IntStatus((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************* */
void MXC_WUT_SetCompare(uint32_t cmp_cnt)
{
    MXC_WUT_RevA_SetCompare((mxc_wut_reva_regs_t *)MXC_WUT, cmp_cnt);
}

/* ************************************************************************* */
void MXC_WUT_SetCount(uint32_t cnt)
{
    MXC_WUT_RevA_SetCount((mxc_wut_reva_regs_t *)MXC_WUT, cnt);
}

/* ************************************************************************* */
int MXC_WUT_GetTicks(uint32_t time, mxc_wut_unit_t units, uint32_t *ticks)
{
    return MXC_WUT_RevA_GetTicks((mxc_wut_reva_regs_t *)MXC_WUT, ERTCO_FREQ, time, units, ticks);
}

/* ************************************************************************* */
int MXC_WUT_GetTime(uint32_t ticks, uint32_t *time, mxc_wut_unit_t *units)
{
    return MXC_WUT_RevA_GetTime((mxc_wut_reva_regs_t *)MXC_WUT, ERTCO_FREQ, ticks, time,
                                (mxc_wut_reva_unit_t *)units);
}

/* ************************************************************************** */
void MXC_WUT_Edge(void)
{
    MXC_WUT_RevA_Edge((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_WaitForEdge(void)
{
    MXC_WUT_RevA_Edge((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_Store(void)
{
    MXC_WUT_RevA_Store((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_StoreCount(void)
{
    MXC_WUT_RevA_Store((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_RestoreBBClock(uint32_t dbbFreq)
{
    MXC_WUT_RevA_RestoreBBClock((mxc_wut_reva_regs_t *)MXC_WUT, dbbFreq, ERTCO_FREQ);
}

/* ************************************************************************** */
uint32_t MXC_WUT_GetSleepTicks(void)
{
    return MXC_WUT_RevA_GetSleepTicks((mxc_wut_reva_regs_t *)MXC_WUT);
}

/* ************************************************************************** */
void MXC_WUT_Delay_MS(uint32_t waitMs)
{
    MXC_WUT_RevA_Delay_MS((mxc_wut_reva_regs_t *)MXC_WUT, waitMs, ERTCO_FREQ);
}
