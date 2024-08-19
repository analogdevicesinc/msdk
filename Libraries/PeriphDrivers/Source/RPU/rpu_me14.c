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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "rpu.h"
#include "rpu_reva.h"

/* **** Functions **** */
int MXC_RPU_Allow(mxc_rpu_device_t periph, uint32_t allow_mask)
{
    return MXC_RPU_RevA_Allow(periph, allow_mask);
}

int MXC_RPU_Disallow(mxc_rpu_device_t periph, uint32_t disallow_mask)
{
    return MXC_RPU_RevA_Disallow(periph, disallow_mask);
}

int MXC_RPU_IsAllowed(void)
{
    return MXC_RPU_RevA_IsAllowed();
}
