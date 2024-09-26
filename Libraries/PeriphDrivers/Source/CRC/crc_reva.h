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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CRC_CRC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CRC_CRC_REVA_H_

#include "crc.h"
#include "crc_reva_regs.h"
#include "dma.h"

/***** CRC Definitions *****/
/**
  * @brief  Structure used to set up CRC request
  *
  */
typedef struct _mxc_crc_reva_req_t {
    uint32_t *dataBuffer; ///< Pointer to the data
    uint32_t dataLen; ///< Length of the data
    uint32_t resultCRC; ///< Calculated CRC value
} mxc_crc_reva_req_t;

/** 
 * @brief CRC data bit order
 *  
 */
typedef enum { CRC_REVA_LSB_FIRST, CRC_REVA_MSB_FIRST } mxc_crc_reva_bitorder_t;

int MXC_CRC_RevA_Init(mxc_crc_reva_regs_t *crc, mxc_dma_regs_t *dma);
int MXC_CRC_RevA_Shutdown(mxc_crc_reva_regs_t *crc);
int MXC_CRC_RevA_Handler(int ch, int error);
void MXC_CRC_RevA_SetDirection(mxc_crc_reva_regs_t *crc, mxc_crc_reva_bitorder_t bitOrder);
mxc_crc_bitorder_t MXC_CRC_RevA_GetDirection(mxc_crc_reva_regs_t *crc);
void MXC_CRC_RevA_SwapDataIn(mxc_crc_reva_regs_t *crc, mxc_crc_reva_bitorder_t bitOrder);
void MXC_CRC_RevA_SwapDataOut(mxc_crc_reva_regs_t *crc, mxc_crc_reva_bitorder_t bitOrder);
void MXC_CRC_RevA_SetPoly(mxc_crc_reva_regs_t *crc, uint32_t poly);
uint32_t MXC_CRC_RevA_GetPoly(mxc_crc_reva_regs_t *crc);
uint32_t MXC_CRC_RevA_GetResult(mxc_crc_reva_regs_t *crc);
int MXC_CRC_RevA_Compute(mxc_crc_reva_regs_t *crc, mxc_crc_reva_req_t *req);
int MXC_CRC_RevA_ComputeAsync(mxc_crc_reva_regs_t *crc, mxc_crc_reva_req_t *req);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CRC_CRC_REVA_H_
