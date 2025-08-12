/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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
static mxc_crc_reva_regs_t *CRCregs;
static mxc_dma_regs_t *CRCdma;

static mxc_crc_reva_req_t *CRCreq;
static mxc_crc_reva_full_req_t *CRCfullreq;

static uint8_t reflect_8bit[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

#define MIN_DMA_LENGTH 16

#define MAX_SIZE_REFLECTED_INPUT_BUFFER 256
static uint8_t reflected_input_buffer[MAX_SIZE_REFLECTED_INPUT_BUFFER];

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_CRC_RevA_Init(mxc_crc_reva_regs_t *crc, mxc_dma_regs_t *dma)
{
    CRCdma = dma;
    CRCregs = crc;

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

static uint32_t reflect_32bit(uint32_t value)
{
    uint32_t result = 0;

    for (int i = 0; i < 32; i++) {
        uint32_t bit = (value >> i) & 1;

        result |= bit << (31 - i);
    }

    return result;
}

static void prepare_temp_reflected_8bit_buffer(uint8_t *dst, uint8_t *src, uint32_t len)
{
    for (int i = 0; i < len; i++) {
        dst[i] = reflect_8bit[src[i]];
    }
}

static uint32_t reflect_8bit_4bytes(uint32_t value)
{
    uint32_t result = 0;
    uint8_t *in_ptr = (uint8_t *)&value;
    uint8_t *out_ptr = (uint8_t *)&result;

    out_ptr[0] = reflect_8bit[in_ptr[0]];
    out_ptr[1] = reflect_8bit[in_ptr[1]];
    out_ptr[2] = reflect_8bit[in_ptr[2]];
    out_ptr[3] = reflect_8bit[in_ptr[3]];

    return result;
}

static int setup_crc_hardware(mxc_crc_reva_regs_t *crc, mxc_crc_reva_full_req_t *req)
{
    uint32_t tmp_value = 0;

    // Setup the CRC detailed parameters:
    if ((req->polynominal & 0xFFFF0000) == 0) {
        MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_MSB, 0 << MXC_F_CRC_REVA_CTRL_MSB_POS);
    } else {
        MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_MSB,
                     req->inputReflected << MXC_F_CRC_REVA_CTRL_MSB_POS);
    }

    // Note:
    // Not found any protocol which requires calculating CRC
    // from the end of the byte sequence to the beginning
    // Setup Input byte order (By Hardware)
    MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_BYTE_SWAP_IN,
                 CRC_REVA_LSBYTE_FIRST << MXC_F_CRC_REVA_CTRL_BYTE_SWAP_IN_POS);

    // Setup Result byte order (By Hardware)
    MXC_SETFIELD(crc->ctrl, MXC_F_CRC_REVA_CTRL_BYTE_SWAP_OUT,
                 CRC_REVA_LSBYTE_FIRST << MXC_F_CRC_REVA_CTRL_BYTE_SWAP_OUT_POS);

    // Setup polynominal (Convert to MAXIM IP format)
    tmp_value = req->polynominal;
    if (((req->polynominal & 0xFFFF0000) == 0) || (req->inputReflected == CRC_REVA_REFLECTED)) {
        tmp_value = reflect_32bit(tmp_value);

        if ((req->polynominal & 0xFFFFFF00) == 0) {
            tmp_value = tmp_value >> 24; // CRC8 request
        } else if ((req->polynominal & 0xFFFF0000) == 0) {
            tmp_value = tmp_value >> 16; // CRC16 request
        } else {
            // CRC32 ==> Do nothing
        }
    }
    crc->poly = tmp_value;

    // Setup initial value (Convert to MAXIM IP format)
    tmp_value = req->initialValue;
    if (((req->polynominal & 0xFFFF0000) == 0) || (req->inputReflected == CRC_REVA_REFLECTED)) {
        tmp_value = reflect_32bit(tmp_value);

        if ((req->polynominal & 0xFFFFFF00) == 0) {
            tmp_value = tmp_value >> 24; // CRC8 request
        } else if ((req->polynominal & 0xFFFF0000) == 0) {
            tmp_value = tmp_value >> 16; // CRC16 request
        } else {
            // CRC32 ==> Do nothing
        }
    }
    crc->val = tmp_value;

    // For the case CRC_REVA_NOT_REFLECTED,
    // For CRC8 and CRC16 only (CRC32 hardware supported)
    // ==> Manual input reflected
    if ((req->inputReflected != CRC_REVA_REFLECTED) && ((req->polynominal & 0xFFFF0000) == 0)) {
        req->manual_reflected_input_required = CRC_REVA_REFLECTED;
    } else {
        req->manual_reflected_input_required = CRC_REVA_NOT_REFLECTED;
    }

    return E_NO_ERROR;
}

static int post_process_crc_result(mxc_crc_reva_regs_t *crc, mxc_crc_reva_full_req_t *req)
{
    uint32_t tmp_crc_result = 0;

    // Calculate CRC result reflected if needed
    tmp_crc_result = crc->val;
    if (((req->inputReflected == CRC_REVA_REFLECTED) &&
         (req->resultReflected != CRC_REVA_REFLECTED)) ||
        ((req->inputReflected != CRC_REVA_REFLECTED) &&
         (((req->resultReflected == CRC_REVA_REFLECTED) && (req->polynominal & 0xFFFF0000) != 0) ||
          ((req->resultReflected != CRC_REVA_REFLECTED) && (req->polynominal & 0xFFFF0000) == 0)))) {
        tmp_crc_result = reflect_32bit(tmp_crc_result);

        if ((req->polynominal & 0xFFFFFF00) == 0) {
            tmp_crc_result = tmp_crc_result >> 24; // CRC8 request
        } else if ((req->polynominal & 0xFFFF0000) == 0) {
            tmp_crc_result = tmp_crc_result >> 16; // CRC16 request
        } else {
            // CRC32 ==> Do nothing
        }
    }

    // Calculate Final Xor Value
    tmp_crc_result = tmp_crc_result ^ req->finalXorValue;
    if ((req->polynominal & 0xFFFFFF00) == 0) {
        tmp_crc_result = tmp_crc_result & 0x000000FF; // CRC8 request
    } else if ((req->polynominal & 0xFFFF0000) == 0) {
        tmp_crc_result = tmp_crc_result & 0x0000FFFF; // CRC16 request
    } else {
        // CRC32 ==> Do nothing
    }
    req->resultCRC = tmp_crc_result;

    // Shutdown the CRC module to save power
    CRCregs->ctrl &= ~MXC_F_CRC_REVA_CTRL_EN;

    return E_NO_ERROR;
}

int MXC_CRC_RevA_Full_Req_Handler(int ch, int error)
{
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;

    MXC_DMA_ReleaseChannel(ch);

    if (error == E_NO_ERROR) {
        if (CRCfullreq->nextPosDMA >= CRCfullreq->dataLen) {
            post_process_crc_result(CRCregs, CRCfullreq);

            CRCregs->ctrl &= ~MXC_F_CRC_CTRL_DMA_EN;

            CRCfullreq->req_state = CRC_REV_DONE;
        } else {
            // In this case Manual Reflect Input is required
            if ((CRCfullreq->dataLen - CRCfullreq->nextPosDMA) > MAX_SIZE_REFLECTED_INPUT_BUFFER) {
                prepare_temp_reflected_8bit_buffer(
                    reflected_input_buffer, &(CRCfullreq->dataBuffer[CRCfullreq->nextPosDMA]),
                    MAX_SIZE_REFLECTED_INPUT_BUFFER);

                srcdst.len = MAX_SIZE_REFLECTED_INPUT_BUFFER;
            } else {
                prepare_temp_reflected_8bit_buffer(
                    reflected_input_buffer, &(CRCfullreq->dataBuffer[CRCfullreq->nextPosDMA]),
                    CRCfullreq->dataLen - CRCfullreq->nextPosDMA);

                srcdst.len = CRCfullreq->dataLen - CRCfullreq->nextPosDMA;
            }
            CRCfullreq->nextPosDMA += srcdst.len;

            // For too short request, do it manually instead of DMA
            if (srcdst.len < MIN_DMA_LENGTH) {
                for (int i = 0; i < srcdst.len; i++) {
                    CRCregs->datain8[0] = reflected_input_buffer[i];
                    while (CRCregs->ctrl & MXC_F_CRC_CTRL_BUSY) {}
                }

                post_process_crc_result(CRCregs, CRCfullreq);

                CRCregs->ctrl &= ~MXC_F_CRC_CTRL_DMA_EN;

                CRCfullreq->req_state = CRC_REV_DONE;
            } else {
                // Continue to trigger another DMA transfer
#if (TARGET_NUM == 32657)
                MXC_DMA_Init(CRCdma);
                CRCfullreq->channel = MXC_DMA_AcquireChannel(CRCdma);
#else
                MXC_DMA_Init();
                CRCfullreq->dmaChannel = MXC_DMA_AcquireChannel();
#endif
                config.reqsel = MXC_DMA_REQUEST_CRCTX;
                config.ch = CRCfullreq->dmaChannel;
                config.srcwd = MXC_DMA_WIDTH_BYTE;
                config.dstwd = MXC_DMA_WIDTH_BYTE;
                config.srcinc_en = 1;
                config.dstinc_en = 0;

                srcdst.ch = CRCfullreq->dmaChannel;

                srcdst.source = reflected_input_buffer;

                MXC_DMA_ConfigChannel(config, srcdst);
                MXC_DMA_SetCallback(CRCfullreq->dmaChannel, MXC_CRC_Full_Req_Handler);

#if (TARGET_NUM == 32657)
                MXC_DMA_EnableInt(CRCdma, CRCfullreq->dmaChannel);
#else
                MXC_DMA_EnableInt(CRCfullreq->dmaChannel);
#endif

                MXC_DMA_SetChannelInterruptEn(CRCfullreq->dmaChannel, 0, 1);
                MXC_DMA_Start(CRCfullreq->dmaChannel);
            }
        }
    } else {
        // Shutdown the CRC module to save power
        CRCregs->ctrl &= ~MXC_F_CRC_REVA_CTRL_EN;
        CRCregs->ctrl &= ~MXC_F_CRC_CTRL_DMA_EN;

        CRCfullreq->resultCRC = 0;
        CRCfullreq->req_state = CRC_REV_DONE;
    }

    CRCfullreq->error = error;

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

int MXC_CRC_RevA_Calculate(mxc_crc_reva_regs_t *crc, mxc_crc_reva_full_req_t *req)
{
    int i = 0;
    uint8_t *ptr8_tail = 0;
    uint32_t *ptr32 = 0;
    volatile int head_bytes_counter = 0;
    volatile int tail_bytes_counter = 0;
    volatile int uint32_counter = 0;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataBuffer == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataLen == 0) {
        return E_INVALID;
    }

    setup_crc_hardware(crc, req);

    crc->ctrl |= MXC_F_CRC_CTRL_EN;

    head_bytes_counter = (4 - (((uintptr_t)req->dataBuffer) % 4)) % 4;

    if (req->dataLen > head_bytes_counter) {
        tail_bytes_counter = (((uintptr_t)req->dataBuffer) + req->dataLen) % 4;
        ptr8_tail = &(req->dataBuffer[req->dataLen - tail_bytes_counter]);
    } else {
        head_bytes_counter = req->dataLen;
        tail_bytes_counter = 0;
    }

    if (req->dataLen > (head_bytes_counter + tail_bytes_counter)) {
        uint32_counter = (req->dataLen - (head_bytes_counter + tail_bytes_counter)) / 4;
        ptr32 = (uint32_t *)(&(req->dataBuffer[head_bytes_counter]));
    } else {
        uint32_counter = 0;
    }

    i = 0;
    while (head_bytes_counter--) {
        crc->datain8[0] = (req->manual_reflected_input_required == CRC_REVA_REFLECTED) ?
                              reflect_8bit[req->dataBuffer[i++]] :
                              req->dataBuffer[i++];
        while (crc->ctrl & MXC_F_CRC_CTRL_BUSY) {}
    }

    i = 0;
    while (uint32_counter--) {
        crc->datain32 = (req->manual_reflected_input_required == CRC_REVA_REFLECTED) ?
                            reflect_8bit_4bytes(ptr32[i++]) :
                            ptr32[i++];
        while (crc->ctrl & MXC_F_CRC_CTRL_BUSY) {}
    }

    i = 0;
    while (tail_bytes_counter--) {
        crc->datain8[0] = (req->manual_reflected_input_required == CRC_REVA_REFLECTED) ?
                              reflect_8bit[ptr8_tail[i++]] :
                              ptr8_tail[i++];
        while (crc->ctrl & MXC_F_CRC_CTRL_BUSY) {}
    }

    post_process_crc_result(crc, req);

    return E_NO_ERROR;
}

int MXC_CRC_RevA_CalculateAsync(mxc_crc_reva_regs_t *crc, mxc_crc_reva_full_req_t *req)
{
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    uint8_t *ptr_input_source = 0;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataBuffer == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataLen == 0) {
        return E_INVALID;
    }

    CRCregs = crc;
    CRCfullreq = req;

    CRCfullreq->req_state = CRC_NOT_DONE;
    CRCfullreq->nextPosDMA = 0;

    setup_crc_hardware(CRCregs, CRCfullreq);

    if (req->manual_reflected_input_required == CRC_REVA_REFLECTED) {
        ptr_input_source = reflected_input_buffer;

        if (CRCfullreq->dataLen < MAX_SIZE_REFLECTED_INPUT_BUFFER) {
            prepare_temp_reflected_8bit_buffer(reflected_input_buffer, CRCfullreq->dataBuffer,
                                               CRCfullreq->dataLen);

            srcdst.len = CRCfullreq->dataLen;
        } else {
            prepare_temp_reflected_8bit_buffer(reflected_input_buffer, CRCfullreq->dataBuffer,
                                               MAX_SIZE_REFLECTED_INPUT_BUFFER);

            srcdst.len = MAX_SIZE_REFLECTED_INPUT_BUFFER;
        }
    } else {
        ptr_input_source = CRCfullreq->dataBuffer;
        srcdst.len = CRCfullreq->dataLen;
    }
    CRCfullreq->nextPosDMA += srcdst.len;

    // For too short request, do it manually instead of DMA
    if (srcdst.len < MIN_DMA_LENGTH) {
        CRCregs->ctrl |= MXC_F_CRC_CTRL_EN;

        for (int i = 0; i < srcdst.len; i++) {
            CRCregs->datain8[0] = ptr_input_source[i];
            while (CRCregs->ctrl & MXC_F_CRC_CTRL_BUSY) {}
        }

        post_process_crc_result(CRCregs, CRCfullreq);

        CRCfullreq->req_state = CRC_REV_DONE;
    } else {
#if (TARGET_NUM == 32657)
        MXC_DMA_Init(CRCdma);
        CRCfullreq->channel = MXC_DMA_AcquireChannel(CRCdma);
#else
        MXC_DMA_Init();
        CRCfullreq->dmaChannel = MXC_DMA_AcquireChannel();
#endif

        config.reqsel = MXC_DMA_REQUEST_CRCTX;
        config.ch = CRCfullreq->dmaChannel;
        config.srcwd = MXC_DMA_WIDTH_BYTE;
        config.dstwd = MXC_DMA_WIDTH_BYTE;
        config.srcinc_en = 1;
        config.dstinc_en = 0;

        srcdst.ch = CRCfullreq->dmaChannel;

        srcdst.source = ptr_input_source;

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_SetCallback(CRCfullreq->dmaChannel, MXC_CRC_Full_Req_Handler);

#if (TARGET_NUM == 32657)
        MXC_DMA_EnableInt(CRCdma, CRCfullreq->dmaChannel);
#else
        MXC_DMA_EnableInt(CRCfullreq->dmaChannel);
#endif

        MXC_DMA_SetChannelInterruptEn(CRCfullreq->dmaChannel, 0, 1);

        CRCregs->ctrl |= MXC_F_CRC_CTRL_DMA_EN;
        CRCregs->ctrl |= MXC_F_CRC_CTRL_EN;

        MXC_DMA_Start(CRCfullreq->dmaChannel);
    }

    return E_NO_ERROR;
}
