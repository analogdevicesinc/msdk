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

/****** Includes *******/
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "spixf.h"
#include "spixfc_fifo_reva_regs.h"
#include "spixfc_reva_regs.h"
#include "spixfm_reva_regs.h"
#include "spixf_reva.h"

/***** Definitions *****/
#define MXC_SPIXF_HEADER_DEASS_SS 1
#define MAX_SCLK 0x10
#define MXC_SPIXF_HEADER_TX_DIR 1
#define NOT_HEADER_DATA 0xF000 // 0xF makes sure it is not a header
#define MXC_SPIXF_MAX_BYTE_LEN 32
#define MXC_SPIXF_MAX_PAGE_LEN 32
#define MXC_SPIXF_NUM_BYTES_PER_PAGE 32

/* Bit positions for the 16-BIT Header. */
#define MXC_SPIXF_HEADER_DIRECTION_POS 0
#define MXC_SPIXF_HEADER_UNITS_POS 2
#define MXC_SPIXF_HEADER_SIZE_POS 4
#define MXC_SPIXF_HEADER_WIDTH_POS 9
#define MXC_SPIXF_HEADER_DEASS_SS_POS 13

static int SPIXFC_ReadRXFIFO(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfc_fifo_reva_regs_t *fifo,
                             uint8_t *data, int len);
static void SPIXFC_TransHandler(mxc_spixfc_reva_regs_t *spixfc,
                                mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, mxc_spixf_req_t *req);

#if defined(SPIXF_RAM)
static int MXC_GetLock_SPIXF(uint32_t *lock, uint32_t value);
static void MXC_FreeLock_SPIXF(uint32_t *lock);
#endif

/******* Globals *******/
typedef struct {
    mxc_spixf_req_t *req;
    int head_rem;
} spixf_req_head_t;
static spixf_req_head_t states;

/****** Functions ******/

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_Init(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                        uint32_t cmdval, uint32_t frequency)
{
    states.req = NULL;
    states.head_rem = 0;
    spixfc->gen_ctrl = 0;
    MXC_SPIXF_RevA_RXFIFOEnable(spixfc);
    MXC_SPIXF_RevA_TXFIFOEnable(spixfc);
    MXC_SPIXF_RevA_SCKFeedbackEnable(spixfc, spixfm);
    MXC_SPIXF_RevA_SetSPIFrequency(spixfc, spixfm, frequency);
#if defined(SPIXF_RAM)
    MXC_SPIXF_RevA_SetSSActiveTime(spixfc, spixfm, MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_RevA_SetSSInactiveTime(spixfc, spixfm, MXC_SPIXF_SYS_CLOCKS_9);
#else
    MXC_SPIXF_RevA_SetSSActiveTime(spixfc, spixfm, MXC_SPIXF_SYS_CLOCKS_0);
    MXC_SPIXF_RevA_SetSSInactiveTime(spixfc, spixfm, MXC_SPIXF_SYS_CLOCKS_1);
#endif
    MXC_SPIXF_RevA_SetCmdValue(spixfm, cmdval);
    MXC_SPIXF_RevA_SetCmdWidth(spixfm, MXC_SPIXF_SINGLE_SDIO);
    MXC_SPIXF_RevA_SetAddrWidth(spixfm, MXC_SPIXF_SINGLE_SDIO);
    MXC_SPIXF_RevA_SetDataWidth(spixfm, MXC_SPIXF_SINGLE_SDIO);
    MXC_SPIXF_RevA_Set3ByteAddr(spixfm);
    MXC_SPIXF_RevA_SetMode(spixfc, spixfm, MXC_SPIXF_MODE_0);
    MXC_SPIXF_RevA_SetPageSize(spixfc, MXC_SPIXF_32B);
    MXC_SPIXF_RevA_SetSSPolActiveLow(spixfc, spixfm);
    return E_NO_ERROR;
}

void MXC_SPIXF_RevA_Shutdown(mxc_spixfc_reva_regs_t *spixfc)
{
    mxc_spixf_req_t *temp_req;
    MXC_SPIXF_RevA_DisableInt(spixfc, 0x3F);
    MXC_SPIXF_RevA_ClearFlags(spixfc, spixfc->int_fl);
    MXC_SPIXF_RevA_Disable(spixfc);
    MXC_SPIXF_RevA_TXFIFODisable(spixfc);
    MXC_SPIXF_RevA_RXFIFODisable(spixfc);

    // Call all of the pending callbacks for this SPIXFC
    if (states.req != NULL) {
        // Save the request
        temp_req = states.req;

        // Unlock this SPIXFC
        MXC_FreeLock((uint32_t *)&states.req);

        // Callback if not NULL
        if (temp_req->callback != NULL) {
            temp_req->callback(temp_req, E_SHUTDOWN);
        }
    }
}

void MXC_SPIXF_RevA_IOCtrl(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_ds_t sclk_ds,
                           mxc_spixf_ds_t ss_ds, mxc_spixf_ds_t sdio_ds,
                           mxc_spixf_padctrl_t padctrl)
{
    spixfm->io_ctrl = 0;

    if (sclk_ds) {
        MXC_SPIXF_RevA_SetIoctrlSCLKDriveHigh(spixfm);
    } else {
        MXC_SPIXF_RevA_SetIoctrlSCLKDriveLow(spixfm);
    }

    if (ss_ds) {
        MXC_SPIXF_RevA_SetIoctrlSSDriveHigh(spixfm);
    } else {
        MXC_SPIXF_RevA_SetIoctrlSSDriveLow(spixfm);
    }

    if (sdio_ds) {
        MXC_SPIXF_RevA_SetIoctrlSDIODriveHigh(spixfm);
    }

    MXC_SPIXF_RevA_SetPuPdCtrl(spixfm, padctrl);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_Clocks(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                          mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, uint32_t len, uint8_t deass)
{
    mxc_spixfc_fifo_reva_regs_t *fifo;
    uint16_t header = MXC_SPIXF_HEADER_TX_DIR;
    uint32_t num = len;

    // Make sure the SPIXFC has been initialized
    MXC_ASSERT(MXC_SPIXF_RevA_IsEnabled(spixfc));

    if (len == 0) {
        return E_NO_ERROR;
    }

#if defined(SPIXF_RAM)
    // Lock this SPIXFC
    if (MXC_GetLock_SPIXF((uint32_t *)&states.req, 1) != E_NO_ERROR) {
        return E_BUSY;
    }
#else
    // Lock this SPIXFC
    if (MXC_GetLock((uint32_t *)&states.req, 1) != E_NO_ERROR) {
        return E_BUSY;
    }
#endif

    // Wait for any previous data to transmit
    while (spixfc->fifo_ctrl & MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT) {}

    // Disable the feedback clock
    MXC_SPIXF_RevA_SCKFeedbackDisable(spixfc, spixfm);

    // Get the TX and RX FIFO for this SPIXFC
    fifo = spixfc_fifo;

    // Send the headers to transmit the clocks
    while (len > 32) {
        fifo->tx_16 = header;
        fifo->tx_16 = NOT_HEADER_DATA;
        fifo->tx_16 = NOT_HEADER_DATA;
        len -= 32;
    }

    if (len) {
        if (len < 32) {
            header |= (len << MXC_SPIXF_HEADER_SIZE_POS);
        }

        header |= (deass << MXC_SPIXF_HEADER_DEASS_SS_POS);

        fifo->tx_16 = header;

        if (len > 16) {
            fifo->tx_16 = NOT_HEADER_DATA;
        }

        fifo->tx_16 = NOT_HEADER_DATA;
    }

    // Wait for all of the data to transmit
    while (spixfc->fifo_ctrl & MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT) {}

    // Enable the feedback clock
    MXC_SPIXF_RevA_SCKFeedbackEnable(spixfc, spixfm);

#if defined(SPIXF_RAM)
    // Unlock this SPIXFC
    MXC_FreeLock_SPIXF((uint32_t *)&states.req);
#else
    // Unlock this SPIXFC
    MXC_FreeLock((uint32_t *)&states.req);
#endif

    return num;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_Transaction(mxc_spixfc_reva_regs_t *spixfc,
                               mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, mxc_spixf_req_t *req)
{
    int remain, temp_read;
    uint32_t pages, bytes;
    uint8_t read, write;
    mxc_spixfc_fifo_reva_regs_t *fifo;
    uint16_t header;

    // Check the input parameters
    MXC_ASSERT(req != NULL);
    MXC_ASSERT((req->rx_data != NULL) || (req->tx_data != NULL));

    // Make sure the SPIXFC has been initialized
    MXC_ASSERT(MXC_SPIXF_RevA_IsEnabled(spixfc));

    if (req->len == 0) {
        return E_NO_ERROR;
    }

#if defined(SPIXF_RAM)
    // Lock this SPIXFC
    if (MXC_GetLock_SPIXF((uint32_t *)&states.req, 1) != E_NO_ERROR) {
        return E_BUSY;
    }
#else
    // Lock this SPIXFC
    if (MXC_GetLock((uint32_t *)&states.req, 1) != E_NO_ERROR) {
        return E_BUSY;
    }
#endif

    // Clear the number of bytes counter
    req->read_num = 0;
    req->write_num = 0;
    states.head_rem = 0;

    // Figure out if we're reading and/or writing
    read = (req->rx_data != NULL);
    write = (req->tx_data != NULL);

    // Get the TX and RX FIFO for this SPIXFC
    fifo = spixfc_fifo;

    remain = req->len;

    while (remain) {
        // Set the transaction configuration in the header
        header = ((write << MXC_SPIXF_HEADER_DIRECTION_POS) |
                  (read << (MXC_SPIXF_HEADER_DIRECTION_POS + 1)) |
                  (req->width << MXC_SPIXF_HEADER_WIDTH_POS));

        if (remain >= MXC_SPIXF_MAX_BYTE_LEN) {
            // Send a 32 byte header
            if (remain == MXC_SPIXF_MAX_BYTE_LEN) {
                // 0 maps to 32 in the header, ...
                header |= ((MXC_SPIXF_HEADER_UNITS_BYTES << MXC_SPIXF_HEADER_UNITS_POS) |
                           (req->deass << MXC_SPIXF_HEADER_DEASS_SS_POS));

                bytes = MXC_SPIXF_MAX_BYTE_LEN;

            } else {
                // Send in increments of 32 byte pages
                header |= (MXC_SPIXF_HEADER_UNITS_PAGES << MXC_SPIXF_HEADER_UNITS_POS);
                pages = remain / MXC_SPIXF_NUM_BYTES_PER_PAGE;

                if (pages >= MXC_SPIXF_MAX_PAGE_LEN) {
                    // No need to set num pages field in header since 0 maps to MXC_SPIXF_MAX_PAGE_LEN in the header
                    // There are 32 bytes per page
                    bytes = MXC_SPIXF_NUM_BYTES_PER_PAGE * MXC_SPIXF_MAX_PAGE_LEN;
                } else {
                    header |= (pages << MXC_SPIXF_HEADER_SIZE_POS);
                    bytes = pages * MXC_SPIXF_NUM_BYTES_PER_PAGE;
                }

                // Check if this is the last header we will send
                if ((remain - bytes) == 0) {
                    header |= (req->deass << MXC_SPIXF_HEADER_DEASS_SS_POS);
                }
            }

            fifo->tx_16 = header;

            // Save the number of bytes we need to write to the FIFO
            states.head_rem = bytes;
            remain -= bytes;
        } else {
            // Send final header with the number of bytes remaining and if
            // we want to de-assert the SS at the end of the transaction
            header |= ((MXC_SPIXF_HEADER_UNITS_BYTES << MXC_SPIXF_HEADER_UNITS_POS) |
                       (remain << MXC_SPIXF_HEADER_SIZE_POS) |
                       (req->deass << MXC_SPIXF_HEADER_DEASS_SS_POS));

            fifo->tx_16 = header;

            states.head_rem = remain;
            remain = 0;
        }

        /* *** */
        while (states.head_rem) {
            if (write) {
                // Wait for there to be room in the TXFIFO
                while (((spixfc->fifo_ctrl & MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT) >>
                        MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT_POS) >
                       (MXC_CFG_SPIXFC_FIFO_DEPTH - 2)) {}

                if (states.head_rem > 1) {
                    fifo->tx_16 =
                        ((req->tx_data[req->write_num + 1] << 8) | (req->tx_data[req->write_num]));
                    req->write_num += 2;
                } else {
                    fifo->tx_16 = (NOT_HEADER_DATA | req->tx_data[req->write_num]);
                    req->write_num++;
                }
            }

            if (read) {
                // Read two bytes
                if (states.head_rem > 1) {
                    temp_read = 0;

                    do {
                        temp_read =
                            SPIXFC_ReadRXFIFO(spixfc, fifo, &req->rx_data[req->read_num], 1);
                    } while (temp_read != 1);

                    req->read_num += 1;
                }

                do {
                    temp_read = SPIXFC_ReadRXFIFO(spixfc, fifo, &req->rx_data[req->read_num], 1);
                } while (temp_read != 1);

                req->read_num += 1;
            }

            if (states.head_rem > 1) {
                states.head_rem -= 1;
            }

            states.head_rem -= 1;
        }
    } // end of while(remain)

#if defined(SPIXF_RAM)
    // Unlock this SPIXFC
    MXC_FreeLock_SPIXF((uint32_t *)&states.req);
#else
    // Unlock this SPIXFC
    MXC_FreeLock((uint32_t *)&states.req);
#endif

    if (write) {
        return req->write_num;
    }

    return req->read_num;
}

int MXC_SPIXF_RevA_TransactionAsync(mxc_spixfc_reva_regs_t *spixfc,
                                    mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, mxc_spixf_req_t *req)
{
    // Check the input parameters
    MXC_ASSERT(req != NULL);
    MXC_ASSERT((req->rx_data != NULL) || (req->tx_data != NULL));

    // Make sure the SPIXFC has been initialized
    MXC_ASSERT(MXC_SPIXF_RevA_IsEnabled(spixfc));

    if (req->len == 0) {
        return E_NO_ERROR;
    }

    // Attempt to register this write request
    if (MXC_GetLock((uint32_t *)&states.req, (uint32_t)req) != E_NO_ERROR) {
        return E_BUSY;
    }

    // Clear the number of bytes counter
    req->read_num = 0;
    req->write_num = 0;
    states.head_rem = 0;

    // Start the write
    SPIXFC_TransHandler(spixfc, spixfc_fifo, req);

    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_AbortAsync(mxc_spixfc_reva_regs_t *spixfc, mxc_spixf_req_t *req)
{
    mxc_spixf_req_t *temp_req;

    // Check the input parameters
    if (req == NULL) {
        return E_BAD_PARAM;
    }

    // Find the request, set to NULL
    if (req == states.req) {
        spixfc->int_en = 0;
        spixfc->int_fl = spixfc->int_fl;

        // Save the request
        temp_req = states.req;

        // Unlock this SPIXFC
        MXC_FreeLock((uint32_t *)&states.req);

        // Callback if not NULL
        if (temp_req->callback != NULL) {
            temp_req->callback(temp_req, E_ABORT);
        }

        return E_NO_ERROR;
    }

    return E_BAD_PARAM;
}

void MXC_SPIXF_RevA_Handler(mxc_spixfc_reva_regs_t *spixfc,
                            mxc_spixfc_fifo_reva_regs_t *spixfc_fifo)
{
    uint32_t flags;

    // Clear the interrupt flags
    spixfc->int_en = 0;
    flags = spixfc->int_fl;
    spixfc->int_fl = flags;

    // Figure out if this SPIXFC has an active request
    if ((states.req != NULL) && (flags)) {
        SPIXFC_TransHandler(spixfc, spixfc_fifo, states.req);
    }
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
__attribute__((noinline)) static int SPIXFC_ReadRXFIFO(mxc_spixfc_reva_regs_t *spixfc,
                                                       mxc_spixfc_fifo_reva_regs_t *fifo,
                                                       uint8_t *data, int len)
{
    int num = 0;

    // Get data from the RXFIFO
    while ((spixfc->fifo_ctrl & MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_CNT) && (len - num)) {
        // Save data from the RXFIFO
        data[num] = fifo->rx_8;
        num++;
    }

    return num;
}

static void SPIXFC_TransHandler(mxc_spixfc_reva_regs_t *spixfc,
                                mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, mxc_spixf_req_t *req)
{
    int remain;
    uint8_t read, write;
    uint16_t header;
    uint32_t pages, bytes, inten;
    unsigned int bytes_read;
    mxc_spixfc_fifo_reva_regs_t *fifo;
    mxc_spixf_req_t *temp_req;

    inten = 0;

    // Get the FIFOS for this spi
    fifo = spixfc_fifo;

    // Figure out if we're reading
    if (req->rx_data != NULL) {
        read = 1;
    } else {
        read = 0;
    }

    // Figure out if we're writing
    if (req->tx_data != NULL) {
        write = 1;
    } else {
        write = 0;
    }

    // Read byte from the FIFO if we are reading
    if (read) {
        // Read all of the data in the RXFIFO, or until we don't need anymore
        bytes_read = SPIXFC_ReadRXFIFO(spixfc, fifo, &req->rx_data[req->read_num],
                                       (req->len - req->read_num));

        req->read_num += bytes_read;

        // Adjust head_rem if we are only reading
        if (!write && (states.head_rem > 0)) {
            states.head_rem -= bytes_read;
        }

        // Figure out how many byte we have left to read
        if (states.head_rem > 0) {
            remain = states.head_rem;
        } else {
            remain = req->len - req->read_num;
        }

        if (remain) { // setting up int levels and RX interrupt flag...
            // Set the RX interrupts
            if (remain > MXC_CFG_SPIXFC_FIFO_DEPTH) { // FIFO Almost FULL level = 16-2 = 14;
                spixfc->fifo_ctrl =
                    ((spixfc->fifo_ctrl & ~MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL) |
                     ((MXC_CFG_SPIXFC_FIFO_DEPTH - 2)
                      << MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL_POS));

            } else { // FIFO Almost Full level = remain-1;
                spixfc->fifo_ctrl =
                    ((spixfc->fifo_ctrl & ~MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL) |
                     ((remain - 1) << MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL_POS));
            }

            inten |= MXC_F_SPIXFC_REVA_INT_EN_RX_FIFO_AF;
        }
    }

    // Figure out how many bytes we have left to send headers for
    if (write) {
        remain = req->len - req->write_num;
    } else {
        remain = req->len - req->read_num;
    }

    // See if we need to send a new header
    if (states.head_rem <= 0 && remain) {
        // Set the transaction configuration in the header
        header = ((write << MXC_SPIXF_HEADER_DIRECTION_POS) |
                  (read << (MXC_SPIXF_HEADER_DIRECTION_POS + 1)) |
                  (req->width << MXC_SPIXF_HEADER_WIDTH_POS));

        if (remain >= MXC_SPIXF_MAX_BYTE_LEN) {
            // Send a 32 byte header
            if (remain == MXC_SPIXF_MAX_BYTE_LEN) {
                header |= ((MXC_SPIXF_HEADER_UNITS_BYTES << MXC_SPIXF_HEADER_UNITS_POS) |
                           (req->deass << MXC_SPIXF_HEADER_DEASS_SS_POS));
                // Save the number of bytes we need to write to the FIFO
                bytes = MXC_SPIXF_MAX_BYTE_LEN;
            } else {
                // Send in increments of 32 byte pages
                header |= (MXC_SPIXF_HEADER_UNITS_PAGES << MXC_SPIXF_HEADER_UNITS_POS);
                pages = remain / MXC_SPIXF_NUM_BYTES_PER_PAGE;

                if (pages >= MXC_SPIXF_MAX_PAGE_LEN) {
                    // 0 maps to 32 in the header
                    bytes = MXC_SPIXF_NUM_BYTES_PER_PAGE * MXC_SPIXF_MAX_PAGE_LEN;
                } else {
                    header |= (pages << MXC_SPIXF_HEADER_SIZE_POS);
                    bytes = pages * MXC_SPIXF_NUM_BYTES_PER_PAGE;
                }

                // Check if this is the last header we will send
                if ((remain - bytes) == 0) {
                    header |= (req->deass << MXC_SPIXF_HEADER_DEASS_SS_POS);
                }
            }

            fifo->tx_16 = header;

            // Save the number of bytes we need to write to the FIFO
            states.head_rem = bytes;

        } else {
            // Send final header with the number of bytes remaining and if
            // we want to de-assert the SS at the end of the transaction
            header |= ((MXC_SPIXF_HEADER_UNITS_BYTES << MXC_SPIXF_HEADER_UNITS_POS) |
                       (remain << MXC_SPIXF_HEADER_SIZE_POS) |
                       (req->deass << MXC_SPIXF_HEADER_DEASS_SS_POS));
            fifo->tx_16 = header;
            states.head_rem = remain;
        }
    }

    // Put data into the FIFO if we are writing
    remain = req->len - req->write_num;

    if (write && states.head_rem) { // from above... the num of bytes...
        // Fill the FIFO
        while ((((spixfc->fifo_ctrl & MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT) >>
                 MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT_POS) < (MXC_CFG_SPIXFC_FIFO_DEPTH - 2)) &&
               (states.head_rem)) {
            if (states.head_rem > 1) {
                // Write 2 byte at a time
                fifo->tx_16 =
                    ((req->tx_data[req->write_num + 1] << 8) | (req->tx_data[req->write_num]));

                req->write_num += 2;
                states.head_rem -= 2;
            } else {
                // Write the last byte
                fifo->tx_16 = (NOT_HEADER_DATA | req->tx_data[req->write_num]);

                req->write_num += 1;
                states.head_rem -= 1;
            }
        }

        remain = req->len - req->write_num;

        // Set the TX interrupts
        if (remain) {
            // Set the TX FIFO almost empty interrupt if we have to refill
            spixfc->fifo_ctrl = ((spixfc->fifo_ctrl & ~MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_AE_LVL) |
                                 ((MXC_CFG_SPIXFC_FIFO_DEPTH - 2)
                                  << MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_AE_LVL_POS));

            inten |= MXC_F_SPIXFC_REVA_INT_EN_TX_FIFO_AE;
        } else if (req->wait_tx) {
            // Set the TX Ready interrupt if we're waiting for the FIFO to empty
            inten |= MXC_F_SPIXFC_REVA_INT_FL_TX_READY;
            req->wait_tx = 0;

            // Wait for transaction to completely finish by just returning and waiting for next interrupt.
            spixfc->int_en = inten;
            return;
        }
    }

    // Check to see if we've finished reading and writing
    if (((read && (req->read_num == req->len)) || !read) &&
        (((req->write_num == req->len) && !req->wait_tx) || !write)) {
        // clear interrupts
        spixfc->int_fl = spixfc->int_fl;

        // Save the request
        temp_req = states.req;

        // Unlock this SPIXFC
        MXC_FreeLock((uint32_t *)&states.req);

        // Callback if not NULL
        if (temp_req->callback != NULL) {
            temp_req->callback(temp_req, E_NO_ERROR);
        }
    }

    // Enable the SPIXFC interrupts
    spixfc->int_en = inten;
}

int MXC_SPIXF_RevA_ReadyForSleep(mxc_spixfc_reva_regs_t *spixfc)
{
    // Check to see if there are any ongoing transactions
    if ((states.req == NULL) && !(spixfc->fifo_ctrl & MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT)) {
        // Disable interrupts, clear flags
        spixfc->int_en = 0;
        spixfc->int_fl = spixfc->int_fl;

        return E_NO_ERROR;
    }

    return E_BUSY;
}

int MXC_SPIXF_RevA_EnableInt(mxc_spixfc_reva_regs_t *spixfc, uint32_t mask)
{
    mask &= (MXC_F_SPIXFC_REVA_INT_EN_TX_STALLED | MXC_F_SPIXFC_REVA_INT_EN_RX_STALLED |
             MXC_F_SPIXFC_REVA_INT_EN_TX_READY | MXC_F_SPIXFC_REVA_INT_EN_RX_DONE |
             MXC_F_SPIXFC_REVA_INT_EN_TX_FIFO_AE | MXC_F_SPIXFC_REVA_INT_EN_RX_FIFO_AF);

    if (!mask) {
        /* No bits set? Wasn't something we can enable. */
        return E_BAD_PARAM;
    }

    spixfc->int_en |= mask;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_DisableInt(mxc_spixfc_reva_regs_t *spixfc, uint32_t mask)
{
    mask &= (MXC_F_SPIXFC_REVA_INT_EN_TX_STALLED | MXC_F_SPIXFC_REVA_INT_EN_RX_STALLED |
             MXC_F_SPIXFC_REVA_INT_EN_TX_READY | MXC_F_SPIXFC_REVA_INT_EN_RX_DONE |
             MXC_F_SPIXFC_REVA_INT_EN_TX_FIFO_AE | MXC_F_SPIXFC_REVA_INT_EN_RX_FIFO_AF);

    if (!mask) {
        /* No bits set? Wasn't something we can disable. */
        return E_BAD_PARAM;
    }

    spixfc->int_en &= (~mask);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_ClearFlags(mxc_spixfc_reva_regs_t *spixfc, uint32_t flags)
{
    flags &= (MXC_F_SPIXFC_REVA_INT_FL_TX_STALLED | MXC_F_SPIXFC_REVA_INT_FL_RX_STALLED |
              MXC_F_SPIXFC_REVA_INT_FL_TX_READY | MXC_F_SPIXFC_REVA_INT_FL_RX_DONE |
              MXC_F_SPIXFC_REVA_INT_FL_TX_FIFO_AE | MXC_F_SPIXFC_REVA_INT_FL_RX_FIFO_AF);

    if (!flags) {
        /* No bits set? Wasn't a flag we can clear. */
        return E_BAD_PARAM;
    }

    spixfc->int_fl = flags;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_GetFlags(mxc_spixfc_reva_regs_t *spixfc)
{
    return spixfc->int_fl;
}

/* ************************************************ */
//Low level
/* ************************************************ */

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetMode(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                           mxc_spixf_mode_t mode)
{
    MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_MODE, mode << MXC_F_SPIXFM_REVA_CFG_MODE_POS);
    MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_MODE, mode << MXC_F_SPIXFC_REVA_CFG_MODE_POS);
    return E_NO_ERROR;
}

mxc_spixf_mode_t MXC_SPIXF_RevA_GetMode(mxc_spixfc_reva_regs_t *spixfc)
{
    return (mxc_spixf_mode_t)((spixfc->cfg & MXC_F_SPIXFC_REVA_CFG_MODE) >>
                              MXC_F_SPIXFC_REVA_CFG_MODE_POS);
}

int MXC_SPIXF_RevA_SetSSPolActiveHigh(mxc_spixfc_reva_regs_t *spixfc,
                                      mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->cfg &= ~MXC_F_SPIXFM_REVA_CFG_SSPOL;
    spixfc->ss_pol |= MXC_F_SPIXFC_REVA_SS_POL_SS_POLARITY;
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetSSPolActiveLow(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->cfg |= MXC_F_SPIXFM_REVA_CFG_SSPOL;
    spixfc->ss_pol &= (~MXC_F_SPIXFC_REVA_SS_POL_SS_POLARITY);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_GetSSPolarity(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->ss_pol & MXC_F_SPIXFC_REVA_SS_POL_SS_POLARITY);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetSPIFrequency(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                                   unsigned int hz)
{
    uint32_t clocks, hi_clk, lo_clk;

    // Check if frequency is too high
    if (hz > SystemCoreClock) {
        return E_BAD_PARAM;
    }

    // Make sure that we can generate this frequency
    clocks = SystemCoreClock / hz;
    hi_clk = clocks / 2;
    lo_clk = clocks / 2;

    if (clocks % 2) {
        hi_clk += 1;
    }

    if (spixfm) {
        if (hi_clk == 0) {
            MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_LO_CLK,
                         1 << MXC_F_SPIXFM_REVA_CFG_LO_CLK_POS);
            MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_HI_CLK,
                         1 << MXC_F_SPIXFM_REVA_CFG_HI_CLK_POS);
        } else if (hi_clk > 15) {
            spixfm->cfg |= MXC_F_SPIXFM_REVA_CFG_LO_CLK;
            spixfm->cfg |= MXC_F_SPIXFM_REVA_CFG_HI_CLK;
        } else {
            MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_LO_CLK,
                         lo_clk << MXC_F_SPIXFM_REVA_CFG_LO_CLK_POS);
            MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_HI_CLK,
                         hi_clk << MXC_F_SPIXFM_REVA_CFG_HI_CLK_POS);
        }
    }

    if (spixfc) {
        if (hi_clk == 0) {
            MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_LO_CLK,
                         1 << MXC_F_SPIXFC_REVA_CFG_LO_CLK_POS);
            MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_HI_CLK,
                         1 << MXC_F_SPIXFC_REVA_CFG_HI_CLK_POS);
        } else if (hi_clk > 15) {
            spixfc->cfg |= MXC_F_SPIXFC_REVA_CFG_LO_CLK;
            spixfc->cfg |= MXC_F_SPIXFC_REVA_CFG_HI_CLK;
        } else {
            MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_LO_CLK,
                         lo_clk << MXC_F_SPIXFC_REVA_CFG_LO_CLK_POS);
            MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_HI_CLK,
                         hi_clk << MXC_F_SPIXFC_REVA_CFG_HI_CLK_POS);
        }
    }

    if (spixfc) {
        return MXC_SPIXF_RevA_GetSPIFrequencyWrite(spixfc);
    } else if (spixfm) {
        return MXC_SPIXF_RevA_GetSPIFrequency(spixfm);
    } else {
        return E_BAD_PARAM;
    }
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
uint32_t MXC_SPIXF_RevA_GetSPIFrequency(mxc_spixfm_reva_regs_t *spixfm)
{
    uint32_t clocks;
    clocks = ((spixfm->cfg & MXC_F_SPIXFM_REVA_CFG_HI_CLK) >> MXC_F_SPIXFM_REVA_CFG_HI_CLK_POS);

    // Avoid divide by zero
    if (clocks == 0) {
        clocks = 0x10;
    }

    return SystemCoreClock / (2 * clocks);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
uint32_t MXC_SPIXF_RevA_GetSPIFrequencyWrite(mxc_spixfc_reva_regs_t *spixfc)
{
    uint32_t clocks;
    clocks = ((spixfc->cfg & MXC_F_SPIXFC_REVA_CFG_HI_CLK) >> MXC_F_SPIXFC_REVA_CFG_HI_CLK_POS);

    // Avoid divide by zero
    if (clocks == 0) {
        clocks = 0x10;
    }

    return PeripheralClock / (2 * clocks);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetSSActiveTime(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                                   mxc_spixf_ssact_t ssact)
{
    MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_SSACT,
                 ssact << MXC_F_SPIXFM_REVA_CFG_SSACT_POS);
    MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_SSACT,
                 ssact << MXC_F_SPIXFC_REVA_CFG_SSACT_POS);
    return E_NO_ERROR;
}

mxc_spixf_ssact_t MXC_SPIXF_RevA_GetSSActiveTime(mxc_spixfc_reva_regs_t *spixfc)
{
    return (mxc_spixf_ssact_t)((spixfc->cfg & MXC_F_SPIXFC_REVA_CFG_SSACT) >>
                               MXC_F_SPIXFC_REVA_CFG_SSACT_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetSSInactiveTime(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                                     mxc_spixf_ssiact_t ssiact)
{
    MXC_SETFIELD(spixfm->cfg, MXC_F_SPIXFM_REVA_CFG_SSIACT,
                 ssiact << MXC_F_SPIXFM_REVA_CFG_SSIACT_POS);
    MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_SSIACT,
                 ssiact << MXC_F_SPIXFC_REVA_CFG_SSIACT_POS);
    return E_NO_ERROR;
}

mxc_spixf_ssiact_t MXC_SPIXF_RevA_GetSSInactiveTime(mxc_spixfc_reva_regs_t *spixfc)
{
    return (mxc_spixf_ssiact_t)((spixfc->cfg & MXC_F_SPIXFC_REVA_CFG_SSIACT) >>
                                MXC_F_SPIXFC_REVA_CFG_SSIACT_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetCmdWidth(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_spiwidth_t width)
{
    MXC_ASSERT(width < MXC_SPIXF_INVALID);

    MXC_SETFIELD(spixfm->fetch_ctrl, MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH,
                 width << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS);
    return E_NO_ERROR;
}

mxc_spixf_spiwidth_t MXC_SPIXF_RevA_GetCmdWidth(mxc_spixfm_reva_regs_t *spixfm)
{
    return (mxc_spixf_spiwidth_t)((spixfm->fetch_ctrl & MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH) >>
                                  MXC_F_SPIXFM_REVA_FETCH_CTRL_CMD_WIDTH_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetAddrWidth(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_spiwidth_t width)
{
    MXC_ASSERT(width < MXC_SPIXF_INVALID);

    MXC_SETFIELD(spixfm->fetch_ctrl, MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH,
                 width << MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS);
    return E_NO_ERROR;
}

mxc_spixf_spiwidth_t MXC_SPIXF_RevA_GetAddrWidth(mxc_spixfm_reva_regs_t *spixfm)
{
    return (mxc_spixf_spiwidth_t)((spixfm->fetch_ctrl & MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH) >>
                                  MXC_F_SPIXFM_REVA_FETCH_CTRL_ADDR_WIDTH_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetDataWidth(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_spiwidth_t width)
{
    MXC_ASSERT(width < MXC_SPIXF_INVALID);

    MXC_SETFIELD(spixfm->fetch_ctrl, MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH,
                 width << MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS);
    return E_NO_ERROR;
}

mxc_spixf_spiwidth_t MXC_SPIXF_RevA_GetDataWidth(mxc_spixfm_reva_regs_t *spixfm)
{
    return (mxc_spixf_spiwidth_t)((spixfm->fetch_ctrl & MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH) >>
                                  MXC_F_SPIXFM_REVA_FETCH_CTRL_DATA_WIDTH_POS);
}

int MXC_SPIXF_RevA_Set4ByteAddr(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->fetch_ctrl |= MXC_F_SPIXFM_REVA_FETCH_CTRL_FOUR_BYTE_ADDR;
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_Set3ByteAddr(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->fetch_ctrl &= (~MXC_F_SPIXFM_REVA_FETCH_CTRL_FOUR_BYTE_ADDR);
    return E_NO_ERROR;
}

unsigned int MXC_SPIXF_RevA_GetBytesPerAddr(mxc_spixfm_reva_regs_t *spixfm)
{
    if (!!(spixfm->fetch_ctrl & MXC_F_SPIXFM_REVA_FETCH_CTRL_FOUR_BYTE_ADDR)) {
        return 4;
    } else {
        return 3;
    }
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetModeClk(mxc_spixfm_reva_regs_t *spixfm, uint8_t mdclk)
{
    MXC_SETFIELD(spixfm->mode_ctrl, MXC_F_SPIXFM_REVA_MODE_CTRL_MDCLK, mdclk);
    return E_NO_ERROR;
}

uint8_t MXC_SPIXF_RevA_GetModeClk(mxc_spixfm_reva_regs_t *spixfm)
{
    return ((spixfm->mode_ctrl & MXC_F_SPIXFM_REVA_MODE_CTRL_MDCLK) >>
            MXC_F_SPIXFM_REVA_MODE_CTRL_MDCLK_POS);
}

int MXC_SPIXF_RevA_SetCmdModeEveryTrans(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->mode_ctrl &= ~(MXC_F_SPIXFM_REVA_MODE_CTRL_NO_CMD);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_SetCmdModeFirstTrans(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->mode_ctrl |= MXC_F_SPIXFM_REVA_MODE_CTRL_NO_CMD;
    return E_NO_ERROR;
}

mxc_spixf_cmd_t MXC_SPIXF_RevA_GetCmdMode(mxc_spixfm_reva_regs_t *spixfm)
{
    if (!!(spixfm->mode_ctrl & MXC_F_SPIXFM_REVA_MODE_CTRL_NO_CMD)) {
        return MXC_SPIXF_CMD_FIRST_TRANS;
    }

    return MXC_SPIXF_CMD_EVERY_TRANS;
}

int MXC_SPIXF_RevA_BBDataOutputEnable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask)
{
    spixfc->gen_ctrl |= (mask << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_BBDataOutputDisable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask)
{
    spixfc->gen_ctrl &= ~(mask << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS);
    return E_NO_ERROR;
}

uint8_t MXC_SPIXF_RevA_BBDataOutputIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return ((spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN) >>
            MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS);
}

uint8_t MXC_SPIXF_RevA_GetBBDataOutputValue(mxc_spixfc_reva_regs_t *spixfc)
{
    return ((spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA) >>
            MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS);
}

uint8_t MXC_SPIXF_RevA_GetBBDataInputValue(mxc_spixfc_reva_regs_t *spixfc)
{
    return ((spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN) >>
            MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetModeData(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                               uint16_t data)
{
    MXC_SETFIELD(spixfm->mode_data, MXC_F_SPIXFM_REVA_MODE_DATA_DATA,
                 data << MXC_F_SPIXFM_REVA_MODE_DATA_DATA_POS);
    return E_NO_ERROR;
}

uint16_t MXC_SPIXF_RevA_GetModeData(mxc_spixfm_reva_regs_t *spixfm)
{
    return ((spixfm->mode_data & MXC_F_SPIXFM_REVA_MODE_DATA_DATA) >>
            MXC_F_SPIXFM_REVA_MODE_DATA_DATA_POS);
}

int MXC_SPIXF_RevA_SetSCKInverted(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->fb_ctrl |= MXC_F_SPIXFM_REVA_FB_CTRL_INVERT_EN;
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_INVERT;
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetSCKNonInverted(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->fb_ctrl &= (~MXC_F_SPIXFM_REVA_FB_CTRL_INVERT_EN);
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_INVERT);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_GetSCKInverted(mxc_spixfm_reva_regs_t *spixfm)
{
    return !!(spixfm->fb_ctrl & MXC_F_SPIXFM_REVA_FB_CTRL_INVERT_EN);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SCKFeedbackEnable(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->fb_ctrl |= MXC_F_SPIXFM_REVA_FB_CTRL_FB_EN;
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB;
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SCKFeedbackDisable(mxc_spixfc_reva_regs_t *spixfc,
                                      mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->fb_ctrl &= (~MXC_F_SPIXFM_REVA_FB_CTRL_FB_EN);
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB);
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SCKFeedbackIsEnabled(mxc_spixfm_reva_regs_t *spixfm)
{
    return !!(spixfm->fb_ctrl & MXC_F_SPIXFM_REVA_FB_CTRL_FB_EN);
}

int MXC_SPIXF_RevA_SetSCKSampleDelay(mxc_spixfc_reva_regs_t *spixfc, uint8_t delay)
{
    MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_IOSMPL,
                 delay << MXC_F_SPIXFC_REVA_CFG_IOSMPL_POS);
    return E_NO_ERROR;
}

uint8_t MXC_SPIXF_RevA_GetSCKSampleDelay(mxc_spixfc_reva_regs_t *spixfc)
{
    return ((spixfc->cfg & MXC_F_SPIXFC_REVA_CFG_IOSMPL) >> MXC_F_SPIXFC_REVA_CFG_IOSMPL_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetCmdValue(mxc_spixfm_reva_regs_t *spixfm, uint8_t cmdval)
{
    MXC_SETFIELD(spixfm->fetch_ctrl, MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL,
                 cmdval << MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL_POS);
    return E_NO_ERROR;
}

uint8_t MXC_SPIXF_RevA_GetCmdValue(mxc_spixfm_reva_regs_t *spixfm)
{
    return ((spixfm->fetch_ctrl & MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL) >>
            MXC_F_SPIXFM_REVA_FETCH_CTRL_CMDVAL_POS);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
void MXC_SPIXF_RevA_SetPageSize(mxc_spixfc_reva_regs_t *spixfc, mxc_spixf_page_size_t size)
{
    MXC_SETFIELD(spixfc->cfg, MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE,
                 size << MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS);
}

mxc_spixf_page_size_t MXC_SPIXF_RevA_GetPageSize(mxc_spixfc_reva_regs_t *spixfc)
{
    return (mxc_spixf_page_size_t)((spixfc->cfg & MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE) >>
                                   MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS);
}

int MXC_SPIXF_RevA_SimpleRXEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_RX;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_SimpleRXDisable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_RX);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_SimpleRXIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_RX);
}

int MXC_SPIXF_RevA_SimpleModeEnable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_SimpleModeDisable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_SimpleModeIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE);
}

int MXC_SPIXF_RevA_SampleOutputEnable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask)
{
    spixfc->sp_ctrl |= (mask << MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN_POS);
    return E_NO_ERROR;
    {
    }
}

int MXC_SPIXF_RevA_SampleOutputDisable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask)
{
    spixfc->sp_ctrl &= ~(mask << MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN_POS);
    return E_NO_ERROR;
}

uint8_t MXC_SPIXF_RevA_SampleOutputIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return ((spixfc->sp_ctrl & MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN) >>
            MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN_POS);
}

uint8_t MXC_SPIXF_RevA_GetSampleOutputValue(mxc_spixfc_reva_regs_t *spixfc)
{
    return ((spixfc->sp_ctrl & MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT) >>
            MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_POS);
}

void MXC_SPIXF_RevA_SetIoctrlSDIODriveHigh(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->io_ctrl |= MXC_F_SPIXFM_REVA_IO_CTRL_SDIO_DS;
}

void MXC_SPIXF_RevA_SetIoctrlSDIODriveLow(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->io_ctrl &= (~MXC_F_SPIXFM_REVA_IO_CTRL_SDIO_DS);
}

uint8_t MXC_SPIXF_RevA_GetIoctrlSDIODrive(mxc_spixfm_reva_regs_t *spixfm)
{
    return !!(spixfm->io_ctrl & MXC_F_SPIXFM_REVA_IO_CTRL_SDIO_DS);
}

void MXC_SPIXF_RevA_SetIoctrlSCLKDriveHigh(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->io_ctrl |= MXC_F_SPIXFM_REVA_IO_CTRL_SCLK_DS;
}

void MXC_SPIXF_RevA_SetIoctrlSCLKDriveLow(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->io_ctrl &= (~MXC_F_SPIXFM_REVA_IO_CTRL_SCLK_DS);
}

uint8_t MXC_SPIXF_RevA_GetIoctrlSCLKDrive(mxc_spixfm_reva_regs_t *spixfm)
{
    return !!(spixfm->io_ctrl & MXC_F_SPIXFM_REVA_IO_CTRL_SCLK_DS);
}

void MXC_SPIXF_RevA_SetIoctrlSSDriveHigh(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->io_ctrl |= MXC_F_SPIXFM_REVA_IO_CTRL_SS_DS;
}

void MXC_SPIXF_RevA_SetIoctrlSSDriveLow(mxc_spixfm_reva_regs_t *spixfm)
{
    spixfm->io_ctrl &= (~MXC_F_SPIXFM_REVA_IO_CTRL_SS_DS);
}

uint8_t MXC_SPIXF_RevA_GetIoctrlSSDrive(mxc_spixfm_reva_regs_t *spixfm)
{
    return !!(spixfm->io_ctrl & MXC_F_SPIXFM_REVA_IO_CTRL_SS_DS);
}

void MXC_SPIXF_RevA_SetPuPdCtrl(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_padctrl_t pad)
{
    MXC_SPIXF_RevA_SetPadCtrl(spixfm, pad);
}

uint8_t MXC_SPIXF_RevA_GetPuPdCtrl(mxc_spixfm_reva_regs_t *spixfm)
{
    return MXC_SPIXF_RevA_GetPadCtrl(spixfm);
}

void MXC_SPIXF_RevA_SetPadCtrl(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_padctrl_t pad)
{
    spixfm->io_ctrl &= ~(MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL);
    spixfm->io_ctrl |= (pad & MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL);
}

uint8_t MXC_SPIXF_RevA_GetPadCtrl(mxc_spixfm_reva_regs_t *spixfm)
{
    return (spixfm->io_ctrl & MXC_F_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL);
}

void MXC_SPIXF_RevA_SetSCKDriveHigh(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_DR;
}

void MXC_SPIXF_RevA_SetSCKDriveLow(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_DR);
}

uint8_t MXC_SPIXF_RevA_GetSCKDrive(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_DR);
}

void MXC_SPIXF_RevA_SetSSDriveOutputHigh(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_SSDR;
}

void MXC_SPIXF_RevA_SetSSDriveOutputLow(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_SSDR);
}

uint8_t MXC_SPIXF_RevA_GetSSDriveOutput(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_SSDR);
}

int MXC_SPIXF_RevA_BitBangModeEnable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_BBMODE;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_BitBangModeDisable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_BBMODE);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_BitBangModeIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_BBMODE);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_RXFIFOEnable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_RX_FIFO_EN;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_RXFIFODisable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_RX_FIFO_EN);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_RXFIFOIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_RX_FIFO_EN);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_TXFIFOEnable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_TX_FIFO_EN;
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_TXFIFODisable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_TX_FIFO_EN);
    return E_NO_ERROR;
}

int MXC_SPIXF_RevA_TXFIFOIsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_TX_FIFO_EN);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_Enable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl |= MXC_F_SPIXFC_REVA_GEN_CTRL_ENABLE;
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_Disable(mxc_spixfc_reva_regs_t *spixfc)
{
    spixfc->gen_ctrl &= (~MXC_F_SPIXFC_REVA_GEN_CTRL_ENABLE);
    return E_NO_ERROR;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_IsEnabled(mxc_spixfc_reva_regs_t *spixfc)
{
    return !!(spixfc->gen_ctrl & MXC_F_SPIXFC_REVA_GEN_CTRL_ENABLE);
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_SPIXF_RevA_SetBusIdle(mxc_spixfm_reva_regs_t *spixfm, unsigned int busidle)
{
    spixfm->bus_idle = busidle;
    return E_NO_ERROR;
}

unsigned int MXC_SPIXF_RevA_GetBusIdle(mxc_spixfm_reva_regs_t *spixfm)
{
    return spixfm->bus_idle;
}

/* ************************************************************************** */

// MXC_GetLock
#if defined(SPIXF_RAM)
#if IAR_PRAGMAS
#pragma section = ".spix_config"
#else
__attribute__((section(".spix_config")))
#endif
int MXC_GetLock_SPIXF(uint32_t *lock, uint32_t value)
{
#ifndef __riscv
    do {
        // Return if the lock is taken by a different thread
        if (__LDREXW((volatile uint32_t *)lock) != 0) {
            return E_BUSY;
        }

        // Attempt to take the lock
    } while (__STREXW(value, (volatile uint32_t *)lock) != 0);

    // Do not start any other memory access until memory barrier is complete
    __DMB();
#endif // __riscv

    return E_NO_ERROR;
}
#endif

// MXC_FreeLock
#if defined(SPIXF_RAM)
#if IAR_PRAGMAS
#pragma section = ".spix_config"
#else
__attribute__((section(".spix_config")))
#endif
void MXC_FreeLock_SPIXF(uint32_t *lock)
{
#ifndef __riscv
    // Ensure memory operations complete before releasing lock
    __DMB();
#endif // __riscv
    *lock = 0;
}
#endif
