/**
 * @file    emac.h
 * @brief   EMAC driver function prototypes and data types.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_EMAC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_EMAC_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "emac_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup emac Ethernet Media Access Controller (EMAC)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/** @brief   Enumeration for the EMAC interrupt events */
typedef enum {
    MXC_EMAC_EVENT_MPS =
        MXC_F_EMAC_INT_EN_MPS, /**! Management Packet Sent Interrupt                   */
    MXC_EMAC_EVENT_RXCMPL =
        MXC_F_EMAC_INT_EN_RXCMPL, /**! Receive Complete Interrupt                         */
    MXC_EMAC_EVENT_RXUBR =
        MXC_F_EMAC_INT_EN_RXUBR, /**! RX Used Bit Read Interrupt                         */
    MXC_EMAC_EVENT_TXUBR =
        MXC_F_EMAC_INT_EN_TXUBR, /**! TX Used Bit Read Interrupt                         */
    MXC_EMAC_EVENT_TXUR =
        MXC_F_EMAC_INT_EN_TXUR, /**! Ethernet Transmit Underrun Interrupt               */
    MXC_EMAC_EVENT_RLE =
        MXC_F_EMAC_INT_EN_RLE, /**! Retry Limit Exceeded Interrupt                     */
    MXC_EMAC_EVENT_TXERR =
        MXC_F_EMAC_INT_EN_TXERR, /**! Transmit Buffers Exhausted In Mid-Frame Interrupt  */
    MXC_EMAC_EVENT_TXCMPL =
        MXC_F_EMAC_INT_EN_TXCMPL, /**! Transmit Complete Interrupt                        */
    MXC_EMAC_EVENT_LC =
        MXC_F_EMAC_INT_EN_LC, /**! Link Change Interrupt                              */
    MXC_EMAC_EVENT_RXOR =
        MXC_F_EMAC_INT_EN_RXOR, /**! Receive Overrun Interrupt                          */
    MXC_EMAC_EVENT_HRESPNO =
        MXC_F_EMAC_INT_EN_HRESPNO, /**! HRESP Not OK Interrupt                             */
    MXC_EMAC_EVENT_PPR =
        MXC_F_EMAC_INT_EN_PPR, /**! Pause Packet Received Interrupt                    */
    MXC_EMAC_EVENT_PTZ =
        MXC_F_EMAC_INT_EN_PTZ /**! Pause Time Zero Interrupt                          */
} mxc_emac_events_t;

/* **** Structures **** */
/**
 * @brief   The callback called on EMAC interrupt event
 *
 */
typedef void (*mxc_emac_cb_func_t)(void);

/**
 * @brief   The microsecond delay function used by the driver
 *
 */
typedef int (*mxc_emac_delay_func_t)(uint32_t);

/**
 * @brief   The table of callback functions for EMAC interrupt events
 *
 */
typedef struct {
    mxc_emac_cb_func_t mps_handler;
    mxc_emac_cb_func_t rxcmpl_handler;
    mxc_emac_cb_func_t rxubr_handler;
    mxc_emac_cb_func_t txubr_handler;
    mxc_emac_cb_func_t txur_handler;
    mxc_emac_cb_func_t rle_handler;
    mxc_emac_cb_func_t txerr_handler;
    mxc_emac_cb_func_t txcmpl_handler;
    mxc_emac_cb_func_t lc_handler;
    mxc_emac_cb_func_t rxor_handler;
    mxc_emac_cb_func_t hrespno_handler;
    mxc_emac_cb_func_t ppr_handler;
    mxc_emac_cb_func_t ptz_handler;
} mxc_emac_cb_funcs_tbl_t;

/**
 * @brief   The information needed for an EMAC buffer descriptor
 *
 */
typedef struct {
    unsigned int addr;
    unsigned int ctrl;
} mxc_emac_dma_desc_t;

/**
 * @brief   The information needed by the EMAC driver to operate
 *
 */
typedef struct {
    mxc_emac_regs_t *regs;
    unsigned int rx_tail;
    unsigned int tx_head;
    unsigned int tx_tail;
    void *rx_buffer;
    void *tx_buffer;
    mxc_emac_dma_desc_t *rx_ring;
    mxc_emac_dma_desc_t *tx_ring;
    unsigned int rx_buffer_dma;
    unsigned int rx_ring_dma;
    unsigned int tx_ring_dma;
    uint16_t phy_addr;

    unsigned int first_init;
    unsigned int rx_buffer_size;
    unsigned int rx_ring_size;
    unsigned int tx_ring_size;
    mxc_emac_delay_func_t delay_us;
    mxc_emac_cb_funcs_tbl_t cb_funcs;
} mxc_emac_device_t;

/**
 * @brief   The basic configuration information to set up EMAC module
 *
 */
typedef struct {
    unsigned char *rx_buff;
    unsigned char *rx_ring_buff;
    unsigned char *tx_ring_buff;
    unsigned int rx_buff_size;
    unsigned int rx_ring_buff_size;
    unsigned int tx_ring_buff_size;
    uint16_t phy_addr;
    unsigned int interrupt_mode;
    unsigned int interrupt_events;
    mxc_emac_delay_func_t delay_us;
    mxc_emac_cb_funcs_tbl_t conf_cb_funcs;
} mxc_emac_config_t;

/* **** Function Prototypes **** */
/* ************************************************************************* */
/* Control/Configuration Functions                                           */
/* ************************************************************************* */
/**
 * @brief      Initialize EMAC device structure
 *
 * @param      config             EMAC configuration parameters
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 * @return     #E_INVALID         if parameter is invalid
 */
int MXC_EMAC_Init(mxc_emac_config_t *config);

/**
 * @brief      Set configuration for EMAC device
 *
 * @param      config             EMAC configuration parameters
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 * @return     #E_INVALID         if parameter is invalid
 * @return     #E_UNINITIALIZED   if device is uninitialized
 */
int MXC_EMAC_SetConfiguration(mxc_emac_config_t *config);

/**
 * @brief      Set EMAC hardware address
 *
 * @param      enetaddr           MAC address
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 */
int MXC_EMAC_SetHwAddr(unsigned char *enetaddr);

/**
 * @brief      Enable interrupt events
 *
 * @param      events             interrupt events to be enabled
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_UNINITIALIZED   if device is uninitialized
 */
int MXC_EMAC_EnableInterruptEvents(unsigned int events);

/**
 * @brief      Disable interrupt events
 *
 * @param      events             interrupt events to be disabled
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_UNINITIALIZED   if device is uninitialized
 */
int MXC_EMAC_DisableInterruptEvents(unsigned int events);

/* ************************************************************************* */
/* Low-Level Functions                                                       */
/* ************************************************************************* */
/**
 * @brief      Start EMAC device
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_UNINITIALIZED   if device is uninitialized
 * @return     #E_NO_DEVICE       if no phy device
 * @return     #E_NO_RESPONSE     if link down
 */
int MXC_EMAC_Start(void);

/**
 * @brief      Stop EMAC device
 *
 * @return     #E_NO_ERROR        if successful
 */
int MXC_EMAC_Stop(void);

/**
 * @brief      Read link status
 *
 * @return     #E_NO_ERROR        link up
 * @return     #E_NO_DEVICE       link down
 */
int MXC_EMAC_ReadLinkStatus(void);

/* ************************************************************************* */
/* Transaction-Level Functions                                               */
/* ************************************************************************* */
/**
 * @brief      Send Ethernet packet in sync mode
 *
 * @param      packet             pointer to the transmission buffer
 * @param      length             length of the packet to be transmitted
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 * @return     #E_UNINITIALIZED   if device is uninitialized
 * @return     #E_UNDERFLOW       if transmission is underrun
 * @return     #E_OVERFLOW        if transmission is exhausted
 * @return     #E_TIME_OUT        if transmission timeout occurs
 */
int MXC_EMAC_SendSync(const void *packet, unsigned int length);

/**
 * @brief      Send Ethernet packet in async mode
 *
 * @param      packet             pointer to the transmission buffer
 * @param      length             length of the packet to be transmitted
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 */
int MXC_EMAC_SendAsync(const void *packet, unsigned int length);

/**
 * @brief      Receive Ethernet packet
 *
 * @param      rx_buff            pointer to the receive buffer
 * @param      max_len            max length to be stored into the receive buffer
 *
 * @return     0 - 1514           length of the received packet
 * @return     #E_UNINITIALIZED   if device is uninitialized
 * @return     #E_NULL_PTR        if pointer is null
 * @return     #E_NONE_AVAIL      if received packet does not fit into the receive buffer
 */
int MXC_EMAC_Recv(void *rx_buff, unsigned int max_len);

/**
 * @brief      Used for interrupt handling
 * @details    In order to handle EMAC interrupt events, the application must call this
 *             function periodically. This can be done from within the EMAC interrupt
 *             handler or periodically by the application if EMAC interrupts are disabled.
 *
 */
void MXC_EMAC_IrqHandler(void);

/**@} end of group emac */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_EMAC_H_
