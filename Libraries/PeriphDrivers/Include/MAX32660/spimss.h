/**
 * @file    spimss.h
 * @brief   Serial Peripheral Interface (SPIMSS) function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32660_SPIMSS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32660_SPIMSS_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_pins.h"
#include "gpio.h"
#include "spimss_regs.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spimss SPIMSS
 * @ingroup spimss
 * @{
 */

/* **** Definitions **** */

/** 
 * @brief Enumeration type for setting the number data lines to use for communication.
 */
typedef enum { // ONLY FOR COMPATIBILITY FOR CONSOLIDATION WITH SPY17, NOT USED OR NEEDED
    DUMMY_1, /**< NOT USED                */
    DUMMY_2, /**< NOT USED                */
    DUMMY_3, /**< NOT USED                */
} mxc_spimss_width_t;

/**
 * @brief Structure type representing a SPI Master Transaction request.
 */
typedef struct mxc_spimss_req mxc_spimss_req_t;

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
 */
typedef void (*mxc_spimss_callback_fn)(mxc_spimss_req_t *req, int error_code);

/**
 * @brief      Structure definition for an SPI Master Transaction request.
 * @note       When using this structure for an asynchronous operation, the
 *             structure must remain allocated until the callback is completed.
 */
struct mxc_spimss_req {
    uint8_t ssel; /**< Not Used*/
    uint8_t deass; /**< Not Used*/
    void *tx_data; /**< Pointer to a buffer to transmit data from. NULL if undesired. */
    void *rx_data; /**< Pointer to a buffer to store data received. NULL if undesired.*/
    mxc_spimss_width_t width; /**< Not Used */
    unsigned len; /**< Number of transfer units to send from the \p tx_data buffer. */
    unsigned bits; /**< Number of bits in transfer unit (e.g. 8 for byte, 16 for short) */
    unsigned rx_num; /**< Number of bytes actually read into the \p rx_data buffer. */
    unsigned tx_num; /**< Number of bytes actually sent from the \p tx_data buffer */
    mxc_spimss_callback_fn callback; /**< Callback function if desired, NULL otherwise */
};

/* **** Function Prototypes **** */

/**
 * @brief     Initialize the spi.
 * @param     spi     Pointer to spi module to initialize.
 * @param     mode    SPI mode for clock phase and polarity.
 * @param     freq    Desired clock frequency.
 * @param     sys_cfg System configuration object
 *
 * @return \c #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_SPIMSS_Init(mxc_spimss_regs_t *spi, unsigned mode, unsigned freq, const sys_map_t sys_cfg);

/**
 * @brief      Shutdown SPI module.
 * @param      spi  Pointer to SPI regs.
 * 
 * @return  \c #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_SPIMSS_Shutdown(mxc_spimss_regs_t *spi);

/**
 * @brief     Execute a master transaction.
 * @param     spi   Pointer to spi module.
 * @param     req   Pointer to spi request
 * 
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIMSS_MasterTrans(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req);

/**
 * @brief     Execute a master transaction over DMA.
 * @param     spi   Pointer to spi module.
 * @param     req   Pointer to spi request.
 *
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIMSS_MasterTransDMA(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req);

/**
 * @brief      Execute SPI transaction based on interrupt handler
 * @param      spi   The spi
 *
 */
void MXC_SPIMSS_Handler(mxc_spimss_regs_t *spi);

/**
 * @brief     Execute a slave transaction.
 * @param     spi   Pointer to spi module.
 * @param     req   Pointer to spi request
 * 
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIMSS_SlaveTrans(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req);

/**
 * @brief      Asynchronously read/write SPI Master data
 *
 * @param      spi   Pointer to spi module
 * @param      req   Pointer to spi request
 *
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIMSS_MasterTransAsync(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req);

/**
 * @brief      Asynchronously read/write SPI Slave data
 *
 * @param      spi   Pointer to spi module
 * @param      req   Pointer to spi request
 *
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIMSS_SlaveTransAsync(mxc_spimss_regs_t *spi, mxc_spimss_req_t *req);

/**
 * @brief      Aborts an Asynchronous request
 *
 * @param      req   Pointer to spi request
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIMSS_AbortAsync(mxc_spimss_req_t *req);

/**
 * @brief      Enable Disable auto dma handling. If set to true, dma channel for transaction
 *             is acquired in the MXC_SPIMSS_MasterTransDMA function. Otherwise, user has to set
 *             tx and rx channel for SPIMSS DMA transaction with MXC_SPIMSS_SetTXDMAChannel and 
 *             MXC_SPIMSS_SetRXDMAChannel functions.
 * 
 * @param      spi   Pointer to spi module
 * @param      enable    Enable Disable auto handler
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful. 
*/
int MXC_SPIMSS_SetAutoDMAHandlers(mxc_spimss_regs_t *spi, bool enable);

/**
 * @brief      Set the TX channel id for DMA to be used in SPIMSS DMA transaction.
 * 
 * @param      spi   Pointer to spi module
 * @param      channel    Id of the channel for TXDma Channel.
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful. 
*/
int MXC_SPIMSS_SetTXDMAChannel(mxc_spimss_regs_t *spi, unsigned int channel);

/**
 * @brief      Returns the current TX channel id set for SPIMSS DMA transaction.
 * 
 * @param      spi   Pointer to spi module
 * @return  \c #TXDMA_ChannelId of the spi module. 
*/
int MXC_SPIMSS_GetTXDMAChannel(mxc_spimss_regs_t *spi);

/**
 * @brief      Set the RX channel id for DMA to be used in SPIMSS DMA transaction.
 * 
 * @param      spi   Pointer to spi module
 * @param      channel    Id of the channel for RXDma Channel.
 * @return  \c #E_NO_ERROR if successful, @ref
 *             MXC_Error_Codes "error" if unsuccessful. 
*/
int MXC_SPIMSS_SetRXDMAChannel(mxc_spimss_regs_t *spi, unsigned int channel);

/**
 * @brief      Returns the current RX channel id set for SPIMSS DMA transaction.
 * 
 * @param      spi   Pointer to spi module
 * @return  \c #RXDMA_ChannelId of the spi module. 
*/
int MXC_SPIMSS_GetRXDMAChannel(mxc_spimss_regs_t *spi);

/**@} end of group spimss */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32660_SPIMSS_H_
