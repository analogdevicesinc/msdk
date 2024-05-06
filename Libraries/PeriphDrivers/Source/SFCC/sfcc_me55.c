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
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sfcc.h"
#include "sfcc_reva.h"

int MXC_SFCC_ID(mxc_sfcc_info_t cid)
{
    return MXC_SFCC_RevA_ID((mxc_sfcc_reva_regs_t *)MXC_SFCC, cid);
}

void MXC_SFCC_Enable(void)
{
    MXC_SFCC_RevA_Enable((mxc_sfcc_reva_regs_t *)MXC_SFCC);
}

void MXC_SFCC_Disable(void)
{
    MXC_SFCC_RevA_Disable((mxc_sfcc_reva_regs_t *)MXC_SFCC);
}

void MXC_SFCC_Flush(void)
{
    /* NOTE: MEMPROT authentication bytes are not flushed with the SFCC invalidate bits,
     * the GCR SFCC flush is required.
     */
    /* Flush all instruction caches */
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_SFCC_FLUSH;

    /* Wait for flush to complete */
    while (MXC_GCR->sysctrl & MXC_F_GCR_SYSCTRL_SFCC_FLUSH) {}

    MXC_SFCC_Disable();
    MXC_SFCC_Enable();
}
