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
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "htmr_regs.h"
#include "htmr.h"
#include "htmr_reva.h"

/* ****** Functions ****** */
int MXC_HTMR_Init(mxc_htmr_regs_t *htmr, uint32_t longInterval, uint8_t shortInterval)
{
    if (htmr == NULL) {
        return E_NULL_PTR;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    if (MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_HIRC8) != E_NO_ERROR) {
        return E_TIME_OUT;
    }

    if (htmr == MXC_HTMR0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HTMR0);
    } else if (htmr == MXC_HTMR1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HTMR1);
    } else {
        return E_BAD_PARAM;
    }
#endif

    return MXC_HTMR_RevA_Init((mxc_htmr_reva_regs_t *)htmr, longInterval, shortInterval);
}

int MXC_HTMR_Start(mxc_htmr_regs_t *htmr)
{
    return MXC_HTMR_RevA_Start((mxc_htmr_reva_regs_t *)htmr);
}

int MXC_HTMR_Stop(mxc_htmr_regs_t *htmr)
{
    return MXC_HTMR_RevA_Stop((mxc_htmr_reva_regs_t *)htmr);
}

int MXC_HTMR_GetShortCount(mxc_htmr_regs_t *htmr)
{
    return MXC_HTMR_RevA_GetShortCount((mxc_htmr_reva_regs_t *)htmr);
}

int MXC_HTMR_GetLongCount(mxc_htmr_regs_t *htmr)
{
    return MXC_HTMR_RevA_GetLongCount((mxc_htmr_reva_regs_t *)htmr);
}

int MXC_HTMR_SetLongAlarm(mxc_htmr_regs_t *htmr, uint32_t interval)
{
    return MXC_HTMR_RevA_SetLongAlarm((mxc_htmr_reva_regs_t *)htmr, interval);
}

int MXC_HTMR_SetShortAlarm(mxc_htmr_regs_t *htmr, uint32_t interval)
{
    return MXC_HTMR_RevA_SetShortAlarm((mxc_htmr_reva_regs_t *)htmr, interval);
}

int MXC_HTMR_CheckBusy(mxc_htmr_regs_t *htmr)
{
    return MXC_HTMR_RevA_CheckBusy((mxc_htmr_reva_regs_t *)htmr);
}

int MXC_HTMR_GetFlags(mxc_htmr_regs_t *htmr)
{
    return MXC_HTMR_RevA_GetFlags((mxc_htmr_reva_regs_t *)htmr);
}

int MXC_HTMR_ClearFlags(mxc_htmr_regs_t *htmr, int flags)
{
    return MXC_HTMR_RevA_ClearFlags((mxc_htmr_reva_regs_t *)htmr, flags);
}

int MXC_HTMR_EnableInt(mxc_htmr_regs_t *htmr, uint32_t mask)
{
    return MXC_HTMR_RevA_EnableInt((mxc_htmr_reva_regs_t *)htmr, mask);
}

int MXC_HTMR_DisableInt(mxc_htmr_regs_t *htmr, uint32_t mask)
{
    return MXC_HTMR_RevA_DisableInt((mxc_htmr_reva_regs_t *)htmr, mask);
}
