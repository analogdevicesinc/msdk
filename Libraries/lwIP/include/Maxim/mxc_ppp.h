/*
 * @file mxc_ppp.h
 *
 ******************************************************************************
 *
 * Copyright (C) 2019-2023 Maxim Integrated Products, Inc. (now owned by 
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
 ******************************************************************************
 */
#ifndef LIBRARIES_LWIP_INCLUDE_MAXIM_ARCH_SYS_ARCH_H_
#define LIBRARIES_LWIP_INCLUDE_MAXIM_ARCH_SYS_ARCH_H_

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
#endif /* LIBRARIES_LWIP_INCLUDE_MAXIM_ARCH_SYS_ARCH_H_ */