/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#include <rmw_microros/rmw_microros.h>
#include "uart.h"

typedef struct {
    mxc_uart_regs_t *uart_instance;
    unsigned int baudrate;
} transport_config_t;

#ifdef __GNUC__
__attribute__((unused))
/* GCC thinks this variable is unused...  but it's not. */
#endif
static transport_config_t transport_config;

/**
 * @brief Function signature callback for opening a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful opened transport, and false in other case.
 */
bool vMXC_Serial_Open(struct uxrCustomTransport* transport);

/**
 * @brief Function signature callback for closing a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful closed transport, and false in other case.
 */
bool vMXC_Serial_Close(struct uxrCustomTransport* transport);

/**
 * @brief Function signature callback for writing to a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @param buffer        Buffer to be sent.
 * @param length        Length of buffer.
 * @param error_code    Error code that should be set in case the write process experiences some error.
 * @return      Number of successfully sent bytes.
 */
size_t vMXC_Serial_Write (
        struct uxrCustomTransport* transport,
        const uint8_t* buffer,
        size_t length,
        uint8_t* error_code);

/**
 * @brief Function signature callback for reading from a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @param buffer        Buffer to write.
 * @param length        Maximum length of buffer.
 * @param timeout       Maximum timeout of the read operation.
 * @param error_code    Error code that should be set in case the write process experiences some error.
 * @return      Number of successfully received bytes.
 */
size_t vMXC_Serial_Read (
        struct uxrCustomTransport* transport,
        uint8_t* buffer,
        size_t length,
        int timeout,
        uint8_t* error_code);