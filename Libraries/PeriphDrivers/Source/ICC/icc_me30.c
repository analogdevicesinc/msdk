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
#include "icc.h"
#include "icc_reva.h"
#include "icc_common.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/* ******************************************************************************
Maxim Internal Use
 * ****************************************************************************** */

int MXC_ICC_ID(mxc_icc_info_t cid)
{
#if CONFIG_TRUSTED_EXECUTION_SECURE
    return MXC_ICC_RevA_ID((mxc_icc_reva_regs_t *)MXC_ICC, cid);
#else
    return E_NOT_SUPPORTED;
#endif
}

void MXC_ICC_Enable(void)
{
    /* Cache controller only accessible in secure world. */
#if CONFIG_TRUSTED_EXECUTION_SECURE
    MXC_ICC_RevA_Enable((mxc_icc_reva_regs_t *)MXC_ICC);
#endif
}

void MXC_ICC_Disable(void)
{
#if CONFIG_TRUSTED_EXECUTION_SECURE
    MXC_ICC_RevA_Disable((mxc_icc_reva_regs_t *)MXC_ICC);
#endif
}

void MXC_ICC_Flush(void)
{
#if CONFIG_TRUSTED_EXECUTION_SECURE
    MXC_ICC_Com_Flush();
#endif
}
