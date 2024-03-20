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
#include <stddef.h>
#include "mxc_assert.h"
#include "tmr.h"
#include "tmr_common.h"

/* **** Functions **** */

void MXC_TMR_Common_Delay(mxc_tmr_regs_t *tmr, uint32_t us)
{
    // Return immediately if delay is 0
    if (!us) {
        return;
    }

    MXC_TMR_TO_Start(tmr, us);

    while (MXC_TMR_TO_Check(tmr) != E_TIME_OUT) {}
}

int MXC_TMR_Common_TO_Check(mxc_tmr_regs_t *tmr)
{
    if (MXC_TMR_GetFlags(tmr)) {
        return E_TIME_OUT;
    }

    return E_NO_ERROR;
}

void MXC_TMR_Common_TO_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Stop(tmr);
    MXC_TMR_SetCount(tmr, 0x0);
}

void MXC_TMR_Common_TO_Clear(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_ClearFlags(tmr);
    MXC_TMR_SetCount(tmr, 0x0);
}

unsigned int MXC_TMR_Common_TO_Remaining(mxc_tmr_regs_t *tmr)
{
    uint32_t remaining_ticks, remaining_time;
    mxc_tmr_unit_t units;

    remaining_ticks = MXC_TMR_GetCompare(tmr) - MXC_TMR_GetCount(tmr);
    MXC_TMR_GetTime(tmr, remaining_ticks, &remaining_time, &units);

    switch (units) {
    case MXC_TMR_UNIT_NANOSEC:
    default:
        return (remaining_time / 1000);

    case MXC_TMR_UNIT_MICROSEC:
        return (remaining_time);

    case MXC_TMR_UNIT_MILLISEC:
        return (remaining_time * 1000);

    case MXC_TMR_UNIT_SEC:
        return (remaining_time * 1000000);
    }
}

void MXC_TMR_Common_SW_Start(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_TO_Start(tmr, 0xFFFFFFFF);
}

unsigned int MXC_TMR_Common_SW_Stop(mxc_tmr_regs_t *tmr)
{
    unsigned int elapsed = MXC_TMR_TO_Elapsed(tmr);
    MXC_TMR_TO_Stop(tmr);
    return elapsed;
}

unsigned int MXC_TMR_Common_TO_Elapsed(mxc_tmr_regs_t *tmr)
{
    uint32_t elapsed;
    mxc_tmr_unit_t units;
    MXC_TMR_GetTime(tmr, tmr->cnt, &elapsed, &units);

    switch (units) {
    case MXC_TMR_UNIT_NANOSEC:
    default:
        return (elapsed / 1000);

    case MXC_TMR_UNIT_MICROSEC:
        return (elapsed);

    case MXC_TMR_UNIT_MILLISEC:
        return (elapsed * 1000);

    case MXC_TMR_UNIT_SEC:
        return (elapsed * 1000000);
    }
}
