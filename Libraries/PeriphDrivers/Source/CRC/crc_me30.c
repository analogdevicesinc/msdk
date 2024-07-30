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

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"

#include "crc.h"
#include "crc_reva.h"
#include "dma.h"

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_CRC_Init(mxc_dma_regs_t *dma)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CRC);
#endif

    MXC_CRC_RevA_Init((mxc_crc_reva_regs_t *)MXC_CRC, dma);

    return E_NO_ERROR;
}

int MXC_CRC_Shutdown(void)
{
    int error = MXC_CRC_RevA_Shutdown((mxc_crc_reva_regs_t *)MXC_CRC);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CRC);

    return error;
}

void MXC_CRC_Handler(int ch, int error)
{
    MXC_CRC_RevA_Handler(ch, error);
}

void MXC_CRC_SetDirection(mxc_crc_bitorder_t bitOrder)
{
    MXC_CRC_RevA_SetDirection((mxc_crc_reva_regs_t *)MXC_CRC, (mxc_crc_reva_bitorder_t)bitOrder);
}

mxc_crc_bitorder_t MXC_CRC_GetDirection(void)
{
    return MXC_CRC_RevA_GetDirection((mxc_crc_reva_regs_t *)MXC_CRC);
}

void MXC_CRC_SwapDataIn(mxc_crc_bitorder_t bitOrder)
{
    MXC_CRC_RevA_SwapDataIn((mxc_crc_reva_regs_t *)MXC_CRC, (mxc_crc_reva_bitorder_t)bitOrder);
}

void MXC_CRC_SwapDataOut(mxc_crc_bitorder_t bitOrder)
{
    MXC_CRC_RevA_SwapDataOut((mxc_crc_reva_regs_t *)MXC_CRC, (mxc_crc_reva_bitorder_t)bitOrder);
}

void MXC_CRC_SetPoly(uint32_t poly)
{
    MXC_CRC_RevA_SetPoly((mxc_crc_reva_regs_t *)MXC_CRC, poly);
}

uint32_t MXC_CRC_GetPoly(void)
{
    return MXC_CRC_RevA_GetPoly((mxc_crc_reva_regs_t *)MXC_CRC);
}

uint32_t MXC_CRC_GetResult(void)
{
    return MXC_CRC_RevA_GetResult((mxc_crc_reva_regs_t *)MXC_CRC);
}

int MXC_CRC_Compute(mxc_crc_req_t *req)
{
    return MXC_CRC_RevA_Compute((mxc_crc_reva_regs_t *)MXC_CRC, (mxc_crc_reva_req_t *)req);
}

int MXC_CRC_ComputeAsync(mxc_crc_req_t *req)
{
    return MXC_CRC_RevA_ComputeAsync((mxc_crc_reva_regs_t *)MXC_CRC, (mxc_crc_reva_req_t *)req);
}
