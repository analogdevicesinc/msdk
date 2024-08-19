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

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sema_reva.h"

/* ***** Functions ***** */

int MXC_SEMA_Init(void)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SMPHR);
    return E_NO_ERROR;
}

int MXC_SEMA_GetSema(unsigned sema)
{
    return MXC_SEMA_RevA_GetSema((mxc_sema_reva_regs_t *)MXC_SEMA, sema);
}

int MXC_SEMA_CheckSema(unsigned sema)
{
    return MXC_SEMA_RevA_CheckSema((mxc_sema_reva_regs_t *)MXC_SEMA, sema);
}

uint32_t MXC_SEMA_Status(void)
{
    return MXC_SEMA_RevA_Status((mxc_sema_reva_regs_t *)MXC_SEMA);
}

void MXC_SEMA_FreeSema(unsigned sema)
{
    MXC_SEMA_RevA_FreeSema((mxc_sema_reva_regs_t *)MXC_SEMA, sema);
}

int MXC_SEMA_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SMPHR);
    return E_NO_ERROR;
}
