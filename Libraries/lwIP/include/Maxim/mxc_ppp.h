/*
 * @file mxc_ppp.h
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
#ifndef _MXC_PPP_H_
#define _MXC_PPP_H_

#include "lwip/netif.h"
#include "lwip/sio.h"

#if NO_SYS
#include "netif/ppp/pppos.h"
#else
#include "netif/ppp/pppapi.h"
#endif

#define MXC_PPP_DEBUG

#ifdef MXC_PPP_DEBUG
//extern void uart_printf(const char *msg, ...);
#define mxc_printf		printf
#else
#define mxc_printf
#endif

typedef sio_fd_t (*sio_open_t)(uint8_t);
typedef uint32_t (*sio_write_t)(sio_fd_t, uint8_t *, uint32_t);
typedef uint32_t (*sio_read_t)(sio_fd_t, uint8_t *, uint32_t);
typedef void (*sio_read_abort_t)(sio_fd_t);
typedef void (*ppp_cb)(ppp_pcb *);

/**
 * @brief      Initialize lwIP PPP
 *
 * @param      uart_port          uart port to be used running pppos
 * @param      mxc_open           uart open function pointer
 * @param      mxc_write          uart write function pointer
 * @param      mxc_read           uart read function pointer
 * @param      mxc_read_abort     uart read abort function pointer
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 */
void mxc_ppp_init(u8_t uart_port, sio_open_t mxc_open, sio_write_t mxc_write,
		sio_read_t mxc_read, sio_read_abort_t mxc_read_abort, void *ctx);

/**
 * @brief      lwIP PPP loop
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 */
void mxc_ppp_loop(void);

/**
 * @brief      force to close lwIP PPP connection
 *
 * @return     #E_NO_ERROR        if successful
 * @return     #E_NULL_PTR        if pointer is null
 */
void mxc_ppp_force_close(void);
#endif /* _MXC_PPP_H_ */