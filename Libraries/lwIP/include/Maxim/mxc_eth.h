/******************************************************************************
 *
 * Copyright (C) 2019-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
#ifndef LIBRARIES_LWIP_INCLUDE_MAXIM_MXC_ETH_H_
#define LIBRARIES_LWIP_INCLUDE_MAXIM_MXC_ETH_H_

#include "lwip/netif.h"
#include "lwip/tcpip.h"

#define MXC_ETH_INTERNAL_BUFF_SIZE 2048
#define MXC_NETIF_MTU_SIZE 1500
#define MXC_ETH_MAX_DATA_SIZE (MXC_NETIF_MTU_SIZE + 14)

typedef struct {
    char name[2];
    netif_status_callback_fn    link_callback;
#if !NO_SYS
    tcpip_init_done_fn          init_done_callback;
    void                        *init_done_arg;
#endif
    unsigned int                (*sys_get_ms)(void);
} mxc_eth_config_t;

/**
 * @brief      Initialize lwIP stack
 *
 * @param      config             configuration parameters
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 */
int MXC_ETH_Init(mxc_eth_config_t *config);

/**
 * @brief      Application must call this function when an Ethernet packet is received
 *
 */
void MXC_ETH_RecvIrq(void);

/**
 * @brief      Application must call this function periodically in order to run lwIP stack
 *
 * @return     #E_NO_ERROR        no issue
 * @return     #MXC_ERROR_CODES   phy issue
 */
int MXC_ETH_Tick(void);

#endif // LIBRARIES_LWIP_INCLUDE_MAXIM_MXC_ETH_H_
