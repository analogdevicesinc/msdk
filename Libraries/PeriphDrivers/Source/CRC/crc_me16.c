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

#include "crc.h"
#include "crc_reva.h"

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_CRC_Init(void)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CRC);
#endif

    MXC_CRC_RevA_Init();

    return E_NO_ERROR;
}

int MXC_CRC_Shutdown(void)
{
    int error = MXC_CRC_RevA_Shutdown();

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CRC);

    return error;
}

int MXC_CRC_Handler(int ch, int error)
{
    return MXC_CRC_RevA_Handler(ch, error);
}

void MXC_CRC_SetDirection(mxc_crc_bitorder_t bitOrder)
{
    MXC_CRC_RevA_SetDirection(bitOrder);
}

mxc_crc_bitorder_t MXC_CRC_GetDirection(void)
{
    return MXC_CRC_RevA_GetDirection();
}

void MXC_CRC_SwapDataIn(mxc_crc_bitorder_t bitOrder)
{
    MXC_CRC_RevA_SwapDataIn(bitOrder);
}

void MXC_CRC_SwapDataOut(mxc_crc_bitorder_t bitOrder)
{
    MXC_CRC_RevA_SwapDataOut(bitOrder);
}

void MXC_CRC_SetPoly(uint32_t poly)
{
    MXC_CRC_RevA_SetPoly(poly);
}

uint32_t MXC_CRC_GetPoly(void)
{
    return MXC_CRC_RevA_GetPoly();
}

uint32_t MXC_CRC_GetResult(void)
{
    return MXC_CRC_RevA_GetResult();
}

int MXC_CRC_Compute(mxc_crc_req_t *req)
{
    return MXC_CRC_RevA_Compute(req);
}

int MXC_CRC_ComputeAsync(mxc_crc_req_t *req)
{
    return MXC_CRC_RevA_ComputeAsync(req);
}
