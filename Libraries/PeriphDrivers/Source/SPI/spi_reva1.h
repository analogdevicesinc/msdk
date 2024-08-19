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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVA1_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVA1_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi_regs.h"
#include "spi_reva_regs.h"
#include "spi.h"
#include "dma.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SPI_REVA_WIDTH_3WIRE,
    SPI_REVA_WIDTH_STANDARD,
    SPI_REVA_WIDTH_DUAL,
    SPI_REVA_WIDTH_QUAD,
} mxc_spi_reva_width_t;

typedef enum {
    SPI_REVA_MODE_0,
    SPI_REVA_MODE_1,
    SPI_REVA_MODE_2,
    SPI_REVA_MODE_3,
} mxc_spi_reva_mode_t;

typedef struct _mxc_spi_reva_req_t mxc_spi_reva_req_t;

struct _mxc_spi_reva_req_t {
    mxc_spi_reva_regs_t *spi;
    int ssIdx;
    int ssDeassert;
    uint8_t *txData;
    uint8_t *rxData;
    uint32_t txLen;
    uint32_t rxLen;
    uint32_t txCnt;
    uint32_t rxCnt;
    spi_complete_cb_t completeCB;
};

int MXC_SPI_RevA1_Init(mxc_spi_reva_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                       unsigned ssPolarity, unsigned int hz);
int MXC_SPI_RevA1_Shutdown(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_ReadyForSleep(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetFrequency(mxc_spi_reva_regs_t *spi, unsigned int hz);
unsigned int MXC_SPI_RevA1_GetFrequency(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetDataSize(mxc_spi_reva_regs_t *spi, int dataSize);
int MXC_SPI_RevA1_GetDataSize(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetMTMode(mxc_spi_reva_regs_t *spi, int mtMode);
int MXC_SPI_RevA1_GetMTMode(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetSlave(mxc_spi_reva_regs_t *spi, int ssIdx);
int MXC_SPI_RevA1_GetSlave(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetWidth(mxc_spi_reva_regs_t *spi, mxc_spi_reva_width_t spiWidth);
mxc_spi_reva_width_t MXC_SPI_RevA1_GetWidth(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetMode(mxc_spi_reva_regs_t *spi, mxc_spi_reva_mode_t spiMode);
mxc_spi_reva_mode_t MXC_SPI_RevA1_GetMode(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_StartTransmission(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_GetActive(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_AbortTransmission(mxc_spi_reva_regs_t *spi);
unsigned int MXC_SPI_RevA1_ReadRXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                      unsigned int len);
unsigned int MXC_SPI_RevA1_WriteTXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                       unsigned int len);
unsigned int MXC_SPI_RevA1_GetTXFIFOAvailable(mxc_spi_reva_regs_t *spi);
unsigned int MXC_SPI_RevA1_GetRXFIFOAvailable(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA1_ClearRXFIFO(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA1_ClearTXFIFO(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetRXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes);
unsigned int MXC_SPI_RevA1_GetRXThreshold(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA1_SetTXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes);
unsigned int MXC_SPI_RevA1_GetTXThreshold(mxc_spi_reva_regs_t *spi);
unsigned int MXC_SPI_RevA1_GetFlags(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA1_ClearFlags(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA1_EnableInt(mxc_spi_reva_regs_t *spi, unsigned int mask);
void MXC_SPI_RevA1_DisableInt(mxc_spi_reva_regs_t *spi, unsigned int mask);
int MXC_SPI_RevA1_MasterTransaction(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA1_MasterTransactionAsync(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA1_MasterTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                       mxc_dma_regs_t *dma);
int MXC_SPI_RevA1_SlaveTransaction(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA1_SlaveTransactionAsync(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA1_SlaveTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                      mxc_dma_regs_t *dma);
void MXC_SPI_RevA1_DMACallback(int ch, int error);
int MXC_SPI_RevA1_SetDefaultTXData(mxc_spi_reva_regs_t *spi, unsigned int defaultTXData);
void MXC_SPI_RevA1_AbortAsync(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA1_AsyncHandler(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA1_HWSSControl(mxc_spi_reva_regs_t *spi, int state);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVA1_H_
