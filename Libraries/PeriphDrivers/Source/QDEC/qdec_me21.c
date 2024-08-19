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

#include <stdio.h>
#include "qdec.h"
#include "qdec_regs.h"
#include "qdec_reva.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include "mxc_pins.h"

int MXC_QDEC_Init(mxc_qdec_req_t *req)
{
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_QDEC);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_QDEC);

    MXC_GPIO_Config(&gpio_cfg_qdec_in);
    MXC_GPIO_Config(&gpio_cfg_qdec_out);

    return MXC_QDEC_RevA_Init((mxc_qdec_reva_regs_t *)MXC_QDEC, req);
}

int MXC_QDEC_Shutdown(void)
{
    MXC_QDEC_RevA_Shutdown((mxc_qdec_reva_regs_t *)MXC_QDEC);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_QDEC);

    return E_NO_ERROR;
}

void MXC_QDEC_EnableInt(uint32_t flags)
{
    MXC_QDEC_RevA_EnableInt((mxc_qdec_reva_regs_t *)MXC_QDEC, flags);
}

void MXC_QDEC_DisableInt(uint32_t flags)
{
    MXC_QDEC_RevA_DisableInt((mxc_qdec_reva_regs_t *)MXC_QDEC, flags);
}

int MXC_QDEC_GetFlags(void)
{
    return MXC_QDEC_RevA_GetFlags((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

void MXC_QDEC_ClearFlags(uint32_t flags)
{
    MXC_QDEC_RevA_ClearFlags((mxc_qdec_reva_regs_t *)MXC_QDEC, flags);
}

void MXC_QDEC_SetMaxCount(uint32_t maxCount)
{
    MXC_QDEC_RevA_SetMaxCount((mxc_qdec_reva_regs_t *)MXC_QDEC, maxCount);
}

int MXC_QDEC_GetMaxCount(void)
{
    return MXC_QDEC_RevA_GetMaxCount((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

void MXC_QDEC_SetInitial(uint32_t initial)
{
    MXC_QDEC_RevA_SetInitial((mxc_qdec_reva_regs_t *)MXC_QDEC, initial);
}

int MXC_QDEC_GetInitial(void)
{
    return MXC_QDEC_RevA_GetInitial((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

void MXC_QDEC_SetCompare(uint32_t compare)
{
    MXC_QDEC_RevA_SetCompare((mxc_qdec_reva_regs_t *)MXC_QDEC, compare);
}

int MXC_QDEC_GetCompare(void)
{
    return MXC_QDEC_RevA_GetCompare((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

int MXC_QDEC_GetIndex(void)
{
    return MXC_QDEC_RevA_GetIndex((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

int MXC_QDEC_GetCapture(void)
{
    return MXC_QDEC_RevA_GetCapture((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

int MXC_QDEC_Handler(void)
{
    return MXC_QDEC_RevA_Handler((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

int MXC_QDEC_GetDirection(void)
{
    return MXC_QDEC_RevA_GetDirection((mxc_qdec_reva_regs_t *)MXC_QDEC);
}

int MXC_QDEC_GetPosition(void)
{
    return MXC_QDEC_RevA_GetPosition((mxc_qdec_reva_regs_t *)MXC_QDEC);
}
