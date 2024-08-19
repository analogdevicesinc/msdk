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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPIMSS_SPIMSS_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPIMSS_SPIMSS_REVA_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_errors.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spimss_regs.h"
#include "spimss_reva_regs.h"
#include "spimss.h"

/** 
 * @brief Enumeration type for setting the number data lines to use for communication.
 */
typedef enum { // ONLY FOR COMPATIBILITY FOR CONSOLIDATION WITH SPY17, NOT USED OR NEEDED
    DUMMY_1_RevA, /**< NOT USED                */
    DUMMY_2_RevA, /**< NOT USED                */
    DUMMY_3_RevA, /**< NOT USED                */
} spimss_reva_width_t;

/**
 * @brief Structure type representing a SPI Master Transaction request.
 */
typedef struct spimss_reva_req spimss_reva_req_t;

/**
 * @brief Callback function type used in asynchronous SPI Master communication requests.
 * @details The function declaration for the SPI Master callback is:
 * @code 
 * void callback(spi_req_t * req, int error_code);
 * @endcode
 * |        |                                            |
 * | -----: | :----------------------------------------- |
 * | \p req |  Pointer to a #spi_req object representing the active SPI Master active transaction. |
 * | \p error_code | An error code if the active transaction had a failure or #E_NO_ERROR if successful. |
 * @note Callback will execute in interrupt context
 * @addtogroup spi_async
 */
typedef void (*spimss_reva_callback_fn)(spimss_reva_req_t *req, int error_code);

/**
 * @brief      Structure definition for an SPI Master Transaction request.
 * @note       When using this structure for an asynchronous operation, the
 *             structure must remain allocated until the callback is completed.
 * @addtogroup spi_async
 */
struct spimss_reva_req {
    uint8_t ssel; /**< Not Used*/
    uint8_t deass; /**< Not Used*/
    void *tx_data; /**< Pointer to a buffer to transmit data from. NULL if undesired. */
    void *rx_data; /**< Pointer to a buffer to store data received. NULL if undesired.*/
    spimss_reva_width_t width; /**< Not Used */
    unsigned len; /**< Number of transfer units to send from the \p tx_data buffer. */
    unsigned bits; /**< Number of bits in transfer unit (e.g. 8 for byte, 16 for short) */
    unsigned rx_num; /**< Number of bytes actually read into the \p rx_data buffer. */
    unsigned tx_num; /**< Number of bytes actually sent from the \p tx_data buffer */
    spimss_reva_callback_fn callback; /**< Callback function if desired, NULL otherwise */
};

int MXC_SPIMSS_RevA_Init(mxc_spimss_reva_regs_t *spi, unsigned mode, unsigned freq);
int MXC_SPIMSS_RevA_Shutdown(mxc_spimss_reva_regs_t *spi);
void MXC_SPIMSS_RevA_Handler(mxc_spimss_reva_regs_t *spi);
int MXC_SPIMSS_RevA_MasterTrans(mxc_spimss_reva_regs_t *spi, spimss_reva_req_t *req);
int MXC_SPIMSS_RevA_MasterTransDMA(mxc_spimss_reva_regs_t *spi, spimss_reva_req_t *req);
int MXC_SPIMSS_RevA_SlaveTrans(mxc_spimss_reva_regs_t *spi, spimss_reva_req_t *req);
int MXC_SPIMSS_RevA_MasterTransAsync(mxc_spimss_reva_regs_t *spi, spimss_reva_req_t *req);
int MXC_SPIMSS_RevA_SlaveTransAsync(mxc_spimss_reva_regs_t *spi, spimss_reva_req_t *req);
int MXC_SPIMSS_RevA_AbortAsync(spimss_reva_req_t *req);

int MXC_SPIMSS_RevA_SetAutoDMAHandlers(mxc_spimss_reva_regs_t *spi, bool enable);
int MXC_SPIMSS_RevA_SetTXDMAChannel(mxc_spimss_reva_regs_t *spi, unsigned int channel);
int MXC_SPIMSS_RevA_GetTXDMAChannel(mxc_spimss_reva_regs_t *spi);
int MXC_SPIMSS_RevA_SetRXDMAChannel(mxc_spimss_reva_regs_t *spi, unsigned int channel);
int MXC_SPIMSS_RevA_GetRXDMAChannel(mxc_spimss_reva_regs_t *spi);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPIMSS_SPIMSS_REVA_H_
