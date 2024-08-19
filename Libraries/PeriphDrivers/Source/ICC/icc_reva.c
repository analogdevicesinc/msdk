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
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "icc.h"
#include "icc_reva.h"
#include "icc_reva_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */
static int MXC_ICC_Ready(mxc_icc_reva_regs_t *icc)
{
    return (icc->ctrl & MXC_F_ICC_REVA_CTRL_RDY);
}

int MXC_ICC_RevA_ID(mxc_icc_reva_regs_t *icc, mxc_icc_info_t cid)
{
    if (icc == NULL) {
        return E_NULL_PTR;
    }

    switch (cid) {
    case ICC_INFO_RELNUM:
        return ((icc->info & MXC_F_ICC_REVA_INFO_RELNUM) >> MXC_F_ICC_REVA_INFO_RELNUM_POS);

    case ICC_INFO_PARTNUM:
        return ((icc->info & MXC_F_ICC_REVA_INFO_PARTNUM) >> MXC_F_ICC_REVA_INFO_PARTNUM_POS);

    case ICC_INFO_ID:
        return ((icc->info & MXC_F_ICC_REVA_INFO_ID) >> MXC_F_ICC_REVA_INFO_ID_POS);

    default:
        return E_BAD_PARAM;
    }
}

void MXC_ICC_RevA_Enable(mxc_icc_reva_regs_t *icc)
{
    // Invalidate cache and wait until ready
    icc->ctrl &= ~MXC_F_ICC_REVA_CTRL_EN;
    icc->invalidate = 1;

    while (!(MXC_ICC_Ready(icc))) {}

    // Enable Cache
    icc->ctrl |= MXC_F_ICC_REVA_CTRL_EN;
    while (!(MXC_ICC_Ready(icc))) {}
}

void MXC_ICC_RevA_Disable(mxc_icc_reva_regs_t *icc)
{
    // Disable Cache
    icc->ctrl &= ~MXC_F_ICC_REVA_CTRL_EN;
}
