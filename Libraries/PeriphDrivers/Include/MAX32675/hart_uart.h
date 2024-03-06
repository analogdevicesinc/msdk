/**
 * @file    afe.h
 * @brief   Analog Front End (AFE) communications r
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_HART_UART_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_HART_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/***** Includes *******/
#include "stdint.h"
#include "afe.h"
#include "afe_adc_zero_regs.h"
#include "afe_adc_one_regs.h"
#include "afe_dac_regs.h"
#include "afe_hart_regs.h"
#include "mxc_sys.h"
#include "mxc_assert.h"

/***** Definitions *****/
#define NORMAL_HART_TRANSCEIVE_MODE 0
#define HART_TEST_MODE_TX_1200 1
#define HART_TEST_MODE_TX_2200 2
#define HART_TEST_MODE_EXTERNAL 3

/** TPDLL Communications Errors, based on Command Summary Specification
    HCF-Spec099 Section 7.3.1 Table 10 Communication Status. */
#define TPDLL_COMM_ERROR_INDICATOR 0x80
#define TPDLL_VERTICAL_PARITY_ERROR 0x40
#define TPDLL_OVERRUN_ERROR 0x20
#define TPDLL_FRAMING_ERROR 0x10
#define TPDLL_BUFFER_OVERFLOW_ERROR 0x02

/** SAP definitions */
#define HART_STATE_IDLE 0
#define HART_STATE_TRANSMIT 1
#define HART_STATE_RECEIVE_ACTIVE 2

// Note: no flag in UARTn_INT_FL register map for buffer overflow since
//      since its a SW error
#define UART_FLAG_BUFFER_OVERFLOW_ERROR 0x80

/** HART UART SAP Callback for RESET.confirm() */
typedef void (*reset_confirm_callback_t)(void);

/** HART UART SAP callback for ENABLE.confirm(state) */
typedef void (*enable_confirm_callback_t)(uint32_t);

/** HART UART SAP callback for ENABLE.Indicate(state) */
typedef void (*enable_indicate_callback_t)(uint32_t);

/** HART UART SAP Callback for DATA.confirm(data) */
typedef void (*data_confirm_callback_t)(uint8_t);

/** HART UART SAP Callback for DATA.Indicate(data) */
typedef void (*data_indicate_callback_t)(uint8_t);

/** HART UART SAP Callback for Error.Indicate(status, data) */
typedef void (*error_indicate_callback_t)(uint8_t, uint8_t);

/** HART UART Callbacks */
typedef struct {
    reset_confirm_callback_t reset_confirm_cb; /**< Callback function for RESET.confirm() */
    enable_confirm_callback_t enable_confirm_cb; /**< Callback function for ENABLE.confirm() */
    enable_indicate_callback_t enable_indicate_cb; /**< Callback function for ENABLE.Indicate() */
    data_confirm_callback_t data_confirm_cb; /**< Callback function for DATA.confirm() */
    data_indicate_callback_t data_indicate_cb; /**< Callback function for DATA.Indicate(data) */
    error_indicate_callback_t
        error_indicate_cb; /**< Callback function for ERROR.Indicate(status, data) */
} hart_uart_callbacks_t;

/***** Function Prototypes *****/
/**
 * @brief   Setup UART connection to HART.
 * @note    Test mode is required for physical layer test to output constant bit
 *          frequencies.
 *
 * @param   test_mode   Select the test mode to run the HART. (Check Definitions)
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_setup(uint32_t test_mode);

/**
 * @brief   Set callbacks for HART Physical Layer SAPs.
 *
 * @param   callbacks   \ref hart_uart_callbacks_t Struct of callbacks for HART Physical Layer SAPs.
 */
void hart_uart_setup_saps(hart_uart_callbacks_t callbacks);

/**
 * @brief   Enable HART.
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_enable(void);

/**
 * @brief   Disable HART.
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_disable(void);

/**
 * @brief   Select HART transmission test at 1200 Hz.
 */
void hart_uart_test_transmit_1200(void);

/**
 * @brief   Select HART transmission test at 2200 Hz.
 */
void hart_uart_test_transmit_2200(void);

/**
 * @brief   Send data to HART, copies data into internal buffer and uses
 *          interrupts to handle transmission. Toggles RTS before and after.
 *
 * @param   data        The buffer of data to write
 * @param   length      The number of bytes to write
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_send(uint8_t *data, uint32_t length);

/**
 * @brief   Check if current transmit packet is complete. This state is reset
 *          to 0 (false) on every call to \ref hart_uart_send, or \ref hart_uart_send_nonblocking
 *
 * @return  0 - transmission still active
 *          1 - transmission completed
 */
int hart_uart_check_transmit_complete(void);

/**
 * @brief   Unload the data received from the HART.
 *
 * @param   buffer          The buffer to store data in
 * @param   *packet_length  The number of bytes received
 * @param   *comm_errors    Any communications error during reception
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_get_received_packet(uint8_t *buffer, uint32_t *packet_length, uint32_t *comm_errors);

/**
 * @brief   The processing function for HART UART interrupts.
 *
 * @note    When using the HART UART, the application must call this
 *          function periodically. This can be done from within the UART interrupt
 *          handler or periodically by the application if UART interrupts are disabled.
 */
void hart_uart_handler(void);

/**
 * @brief   Set HART RTS pin to receive mode, usually for testing purposes.
 *
 */
void hart_rts_receive_mode(void);

/**
 * @brief   Set HART RTS pin to transmit mode, usually for testing purposes.
 *
 */
void hart_rts_transmit_mode(void);

/**
 * @brief   Enables the 4Mhz clock that drives HART state machine
 *
 */
int hart_clock_enable(void);

/**
 * @brief   Disables the 4Mhz clock that drives HART state machine
 *
 */
void hart_clock_disable(void);

/**
 * @brief   Disables then enables HART UART Physical Layer interface
 *
 */
void hart_sap_reset_request(void);

/**
 * @brief  Adds data to transmit FIFO
 * @param  data   Data to transmit
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_sap_data_request(uint8_t data);

/**
 * @brief Sets RTS state
 * @param state State for RTS pin /ref HART_STATE_TRANSMIT /ref HART_STATE_RECEIVE
 *
 */
void hart_sap_enable_request(uint32_t state);

/**@} end of group hart_uart */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_HART_UART_H_
