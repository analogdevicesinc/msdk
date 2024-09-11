/*
 * @file mxc_ppp.c
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

#include <stdio.h>
#include <string.h>

#include "lwip/init.h"
#include "lwip/opt.h"
#include "arch/cc.h"

#include "lwip/sio.h"

#if PPP_SUPPORT && PPPOS_SUPPORT /* don't build if not configured for use in lwipopts.h */
#include "mxc_ppp.h"

/***** Globals *****/
static sio_fd_t ppp_sio = NULL;
static ppp_pcb *ppp = NULL;
static struct netif ppp_netif;
u8_t sio_idx = 0;
volatile int callClosePpp = 0;

static sio_open_t mxc_sio_open;
static sio_write_t mxc_sio_write;
static sio_read_t mxc_sio_read;
static sio_read_abort_t mxc_sio_read_abort;

/***** Functions *****/
static void ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
	struct netif *pppif = ppp_netif(pcb);

	switch(err_code)
	{
		case PPPERR_NONE:               /* No error. */
		{
			mxc_printf("ppp_cb: connected\n\r");
#if LWIP_IPV4
			mxc_printf(" ip = %s\n\r", ip4addr_ntoa(netif_ip4_addr(pppif)));
			mxc_printf(" peer = %s\n\r", ip4addr_ntoa(netif_ip4_gw(pppif)));
			mxc_printf(" netmask = %s\n\r", ip4addr_ntoa(netif_ip4_netmask(pppif)));
#endif /* LWIP_IPV4 */
			if(ctx)
				((ppp_cb)ctx)(pcb);
		}
		break;

		case PPPERR_PARAM:             /* Invalid parameter. */
			mxc_printf("ppp_cb: PPPERR_PARAM\n");
			break;

		case PPPERR_OPEN:              /* Unable to open PPP session. */
			mxc_printf("ppp_cb: Unable to open PPP session\n");
			break;

		case PPPERR_DEVICE:            /* Invalid I/O device for PPP. */
			mxc_printf("ppp_cb: Invalid I/O device for PPP\n");
			break;

		case PPPERR_ALLOC:             /* Unable to allocate resources. */
			mxc_printf("ppp_cb: Unable to allocate resources\n");
			break;

		case PPPERR_USER:              /* User interrupt. */
			mxc_printf("ppp_cb: User interrupt\n");
			break;

		case PPPERR_CONNECT:           /* Connection lost. */
			mxc_printf("ppp_cb: Connection lost\n");
			if(ctx)
				((ppp_cb)ctx)(pcb);
			break;

		case PPPERR_AUTHFAIL:          /* Failed authentication challenge. */
			mxc_printf("ppp_cb: Failed authentication challenge\n");
			break;

		case PPPERR_PROTOCOL:          /* Failed to meet protocol. */
			mxc_printf("ppp_cb: Failed to meet protocol\n");
			break;

		case PPPERR_PEERDEAD:          /* Connection timeout. */
			mxc_printf("ppp_cb: Connection timeout\n");
			break;

		case PPPERR_IDLETIMEOUT:       /* Idle Timeout. */
			mxc_printf("ppp_cb: Idle Timeout\n");
			break;

		case PPPERR_CONNECTTIME:       /* PPPERR_CONNECTTIME. */
			mxc_printf("ppp_cb: PPPERR_CONNECTTIME\n");
			break;

		case PPPERR_LOOPBACK:          /* Connection timeout. */
			mxc_printf("ppp_cb: Connection timeout\n");
			break;

		default:
			mxc_printf("ppp_cb: unknown errCode %d\n", err_code);
			break;
	}
}

static u32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
	u32_t size = 0;

	LWIP_UNUSED_ARG(pcb);
	LWIP_UNUSED_ARG(ctx);

	if (mxc_sio_write)
		size = mxc_sio_write(ppp_sio, data, len);

	return size;
}

/* This function initializes all network interfaces */
#if NO_SYS
static void mxc_ppp_netif_init(void *ctx)
{
	if (mxc_sio_open)
		ppp_sio = mxc_sio_open(sio_idx);

	if (ppp_sio)
	{
		ppp = pppos_create(&ppp_netif, ppp_output_cb, ppp_link_status_cb, ctx);

		if (ppp)
		{
			ppp_set_default(ppp);
			//ppp_set_auth(ppp, PPPAUTHTYPE_ANY, "login", "password");

			ppp_connect(ppp, 0);
		}
	}
}
#else
static void mxc_ppp_netif_init(void *ctx)
{
	if (mxc_sio_open)
		ppp_sio = mxc_sio_open(sio_idx);

	if (ppp_sio)
	{
		ppp = pppapi_pppos_create(&ppp_netif, ppp_output_cb, ppp_link_status_cb, ctx);

		if (ppp)
		{
			pppapi_set_default(ppp);
			//pppapi_set_auth(ppp, PPPAUTHTYPE_ANY, "login", "password");

			pppapi_connect(ppp, 0);
		}
	}
}
#endif
/* This function initializes this lwIP ppp. */
void mxc_ppp_init(u8_t uart_port, sio_open_t mxc_open, sio_write_t mxc_write,
					sio_read_t mxc_read, sio_read_abort_t mxc_read_abort, void *ctx)
{
	sio_idx = uart_port;

	srand((unsigned int)time(0));	/* init randomizer again (seed per thread) */

	if(mxc_open && mxc_write && mxc_read && mxc_read_abort)
	{
		mxc_sio_open = mxc_open;
		mxc_sio_write = mxc_write;
		mxc_sio_read = mxc_read;
		mxc_sio_read_abort = mxc_read_abort;
	}

	/* initialize lwIP stack, network interfaces and applications */
#if NO_SYS //No need to run this again if !NO_SYS
	lwip_init();
#endif
	mxc_ppp_netif_init(ctx);	/* init network interfaces */
}

void mxc_ppp_loop(void)
{
	int count = 0;
	u8_t rxbuf[PPP_RX_BUFFER_SIZE];

	/* handle timers (already done in tcpip.c when NO_SYS=0) */
	sys_check_timeouts();

	/* try to read characters from serial line and pass them to PPPoS */
	if (mxc_sio_read)
		count = mxc_sio_read(ppp_sio, (u8_t*)rxbuf, PPP_RX_BUFFER_SIZE);

	if(count)
#if NO_SYS
		pppos_input(ppp, rxbuf, count);
#else
		pppos_input_tcpip(ppp, rxbuf, count);
#endif

	if(callClosePpp && ppp)
	{
		callClosePpp = 0;
#if NO_SYS
		ppp_close(ppp, 0);
#else
		pppapi_close(ppp, 0);
#endif
		ppp = NULL;
	}
}

void mxc_ppp_force_close(void)
{
	if(ppp)
	{
		u32_t started;

		/* make sure to disconnect PPP before stopping the program... */
#if NO_SYS
		ppp_close(ppp, 0);
#else
		pppapi_close(ppp, 0);
#endif
		ppp = NULL;

		/* Wait for some time to let PPP finish... */
		started = sys_now();
		do
		{
		/* @todo: need a better check here: only wait until PPP is down */
		} while(sys_now() - started < 5000);
	}
}

#endif /* PPP_SUPPORT && PPPOS_SUPPORT */
