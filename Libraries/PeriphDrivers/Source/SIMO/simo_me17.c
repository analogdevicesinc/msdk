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
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "simo_reva.h"

/* **** Functions **** */
void MXC_SIMO_SetVregO_A(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_A((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_B(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_B((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_C(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_C((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

void MXC_SIMO_SetVregO_D(uint32_t voltage)
{
    MXC_SIMO_RevA_SetVregO_D((mxc_simo_reva_regs_t *)MXC_SIMO, voltage);
}

// void MXC_SIMO_setIpkA(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkA(peak_current);
// }

// void MXC_SIMO_setIpkB(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkB(peak_current);
// }

// void MXC_SIMO_setIpkC(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkC(peak_current);
// }

// void MXC_SIMO_setIpkD(uint32_t peak_current)
// {
//     MXC_SIMO_RevA_setIpkD(peak_current);
// }

// void MXC_SIMO_setMaxTon(uint32_t ontime)
// {
//     MXC_SIMO_RevA_setMaxTon(ontime);
// }

// void MXC_SIMO_setAlertThresholdA(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdA(threshold);
// }

// void MXC_SIMO_setAlertThresholdB(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdB(threshold);
// }

// void MXC_SIMO_setAlertThresholdC(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdC(threshold);
// }

// void MXC_SIMO_setAlertThresholdD(uint32_t threshold)
// {
//     MXC_SIMO_RevA_setAlertThresholdD(threshold);
// }

// void MXC_SIMO_setZeroCrossCalA(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalA(zerocross);
// }

// void MXC_SIMO_setZeroCrossCalB(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalB(zerocross);
// }

// void MXC_SIMO_setZeroCrossCalC(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalC(zerocross);
// }

// void MXC_SIMO_setZeroCrossCalD(uint32_t zerocross)
// {
//     MXC_SIMO_RevA_setZeroCrossCalD(zerocross);
// }

uint32_t MXC_SIMO_GetOutReadyA(void)
{
    return MXC_SIMO_RevA_GetOutReadyA((mxc_simo_reva_regs_t *)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyB(void)
{
    return MXC_SIMO_RevA_GetOutReadyB((mxc_simo_reva_regs_t *)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyC(void)
{
    return MXC_SIMO_RevA_GetOutReadyC((mxc_simo_reva_regs_t *)MXC_SIMO);
}

uint32_t MXC_SIMO_GetOutReadyD(void)
{
    return MXC_SIMO_RevA_GetOutReadyD((mxc_simo_reva_regs_t *)MXC_SIMO);
}
