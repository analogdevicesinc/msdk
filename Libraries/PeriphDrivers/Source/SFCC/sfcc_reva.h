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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SFCC_SFCC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SFCC_SFCC_REVA_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sfcc.h"
#include "sfcc_reva_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

int MXC_SFCC_RevA_ID(mxc_sfcc_reva_regs_t *sfcc, mxc_sfcc_info_t cid);
void MXC_SFCC_RevA_Enable(mxc_sfcc_reva_regs_t *sfcc);
void MXC_SFCC_RevA_Disable(mxc_sfcc_reva_regs_t *sfcc);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SFCC_SFCC_REVA_H_
