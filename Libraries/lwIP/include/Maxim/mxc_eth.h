/*
 * @file mxc_eth.h
 *
 ******************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 ******************************************************************************
 */
#ifndef _MXC_ETH_H_
#define _MXC_ETH_H_

#include "lwip/netif.h"

#define MXC_ETH_INTERNAL_BUFF_SIZE			2048
#define MXC_NETIF_MTU_SIZE					1500
#define MXC_ETH_MAX_DATA_SIZE				(MXC_NETIF_MTU_SIZE + 14)

typedef struct {
	char						name[2];
	netif_status_callback_fn	link_callback;
	unsigned int				(*sys_get_ms)(void);
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

#endif /* _MXC_ETH_H_ */

