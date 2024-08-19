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

#include <stdlib.h>
#include <string.h>

#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"

#include "dma.h"
#include "crc_regs.h"
#include "crc_reva.h"

/***** Global Variables *****/
static mxc_crc_reva_req_t *CRCreq;
static mxc_dma_regs_t *CRCdma;

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_CRC_RevA_Init(mxc_crc_reva_regs_t *crc, mxc_dma_regs_t *dma)
{
    CRCdma = dma;

    crc->ctrl = 0x00;
    crc->val = 0xFFFFFFFF;
    return E_NO_ERROR;
}

int MXC_CRC_RevA_Shutdown(mxc_crc_reva_regs_t *crc)
{
    crc->ctrl &= ~MXC_F_CRC_REVA_CTRL_EN;
    return E_NO_ERROR;
}

int MXC_CRC_RevA_Handler(int ch, int error)
{
    if (error == E_NO_ERROR) {
        CRCreq->resultCRC = MXC_CRC_GetResult();
    }
    return error;
}

/* ************************************************************************* */
/* Cyclic Redundancy Check(CRC) functions                                   */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CRC_RevA_SetDirection(mxc_crc_reva_regs_t *crc, mxc_crc_reva_bitorder_t bitOrder)
{
    MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_MSB, bitOrder << MXC_F_CRC_REVA_CTRL_MSB_POS);
}

mxc_crc_bitorder_t MXC_CRC_RevA_GetDirection(mxc_crc_reva_regs_t *crc)
{
    return !!(crc->ctrl & MXC_F_CRC_REVA_CTRL_MSB);
}

void MXC_CRC_RevA_SwapDataIn(mxc_crc_reva_regs_t *crc, mxc_crc_reva_bitorder_t bitOrder)
{
    MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_BYTE_SWAP_IN,
                 bitOrder << MXC_F_CRC_REVA_CTRL_BYTE_SWAP_IN_POS);
}

void MXC_CRC_RevA_SwapDataOut(mxc_crc_reva_regs_t *crc, mxc_crc_reva_bitorder_t bitOrder)
{
    MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_BYTE_SWAP_OUT,
                 bitOrder << MXC_F_CRC_REVA_CTRL_BYTE_SWAP_OUT_POS);
}

void MXC_CRC_RevA_SetPoly(mxc_crc_reva_regs_t *crc, uint32_t poly)
{
    crc->poly = poly;
}

uint32_t MXC_CRC_RevA_GetPoly(mxc_crc_reva_regs_t *crc)
{
    return crc->poly;
}

uint32_t MXC_CRC_RevA_GetResult(mxc_crc_reva_regs_t *crc)
{
    return crc->val;
}

/*******************************/
/* High Level Functions        */
/*******************************/

int MXC_CRC_RevA_Compute(mxc_crc_reva_regs_t *crc, mxc_crc_reva_req_t *req)
{
    int i = 0;
    volatile int length;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataBuffer == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataLen == 0) {
        return E_INVALID;
    }

    crc->ctrl |= MXC_F_CRC_REVA_CTRL_EN;

    length = req->dataLen;

    while (length--) {
        crc->datain32 = req->dataBuffer[i++];
        while (crc->ctrl & MXC_F_CRC_REVA_CTRL_BUSY) {}
    }

    // Store the crc value
    req->resultCRC = MXC_CRC_GetResult();

    return E_NO_ERROR;
}

int MXC_CRC_RevA_ComputeAsync(mxc_crc_reva_regs_t *crc, mxc_crc_reva_req_t *req)
{
    uint8_t channel;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataBuffer == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataLen == 0) {
        return E_INVALID;
    }

    CRCreq = req;

#if (TARGET_NUM == 32657)
    MXC_DMA_Init(CRCdma);

    channel = MXC_DMA_AcquireChannel(CRCdma);
#else
    MXC_DMA_Init();

    channel = MXC_DMA_AcquireChannel();
#endif

    config.reqsel = MXC_DMA_REQUEST_CRCTX;

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = channel;
    srcdst.source = (uint8_t *)req->dataBuffer; //transfering bytes
    srcdst.len = req->dataLen * 4; //number of bytes

    MXC_CRC->ctrl |= MXC_F_CRC_CTRL_DMA_EN;
    MXC_CRC->ctrl |= MXC_F_CRC_CTRL_EN;

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_CRC_Handler);

#if (TARGET_NUM == 32657)
    MXC_DMA_EnableInt(CRCdma, channel);
#else
    MXC_DMA_EnableInt(channel);
#endif

    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);

    return E_NO_ERROR;
}
