/**
 * @file        main.c
 * @brief       lwIP Ping Example
 * @details     This example shows how to ping with lwIP library
 * @note
 */

/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <string.h>

#include <MAX32xxx.h>

#include "mxc_eth.h"
#include "lwipcfg.h"
#include "ping.h"

/***** Definitions *****/
#define ETH_RX_BUFFER_SIZE              (4096)
#define ETH_RX_RING_BUFFER_SIZE         (256)
#define ETH_TX_RING_BUFFER_SIZE         (128)
#define ETH_TOTAL_BUFFER_SIZE           (ETH_RX_BUFFER_SIZE + ETH_RX_RING_BUFFER_SIZE + ETH_TX_RING_BUFFER_SIZE)

/***** Globals *****/
unsigned char eth_buffer[ETH_TOTAL_BUFFER_SIZE];

/***** Functions *****/
static void rxcmpl_handler_func(void)
{
    MXC_ETH_RecvIrq();
}

static int config_emac(void)
{
    int                 result = E_UNKNOWN;
    mxc_emac_config_t   emac_config;
    unsigned char       hwaddr[MAC_LEN] = {MAC_BYTE1, MAC_BYTE2, MAC_BYTE3,
                                           MAC_BYTE4, MAC_BYTE5, MAC_BYTE6
                                          };
                                          
    memset(&emac_config, 0, sizeof(emac_config));
    
    emac_config.rx_ring_buff      = &eth_buffer[0];
    emac_config.rx_ring_buff_size = ETH_RX_RING_BUFFER_SIZE;
    
    emac_config.tx_ring_buff      = &eth_buffer[ETH_RX_RING_BUFFER_SIZE];
    emac_config.tx_ring_buff_size = ETH_TX_RING_BUFFER_SIZE;
    
    emac_config.rx_buff           = &eth_buffer[ETH_RX_RING_BUFFER_SIZE + ETH_TX_RING_BUFFER_SIZE];
    emac_config.rx_buff_size      = ETH_RX_BUFFER_SIZE;
    
    emac_config.phy_addr          = 0;
    emac_config.delay_us          = MXC_Delay;
    
    emac_config.interrupt_mode    = 1;
    emac_config.interrupt_events  = MXC_EMAC_EVENT_RXCMPL;
    
    emac_config.conf_cb_funcs.rxcmpl_handler = rxcmpl_handler_func;
    
    result = MXC_EMAC_Init(&emac_config);
    
    if (result) {
        return result;
    }
    
    result = MXC_EMAC_SetHwAddr(hwaddr);
    
    if (result) {
        return result;
    }
    
    NVIC_SetVector(EMAC_IRQn, MXC_EMAC_IrqHandler);
    NVIC_EnableIRQ(EMAC_IRQn);
    __enable_irq();
    
    return E_NO_ERROR;
}

static void link_callback_func(struct netif* netif)
{
    if (netif->flags & NETIF_FLAG_LINK_UP) {
        printf("Link Status: Up\n");
    }
    else {
        printf("Link Status: Down\n");
    }
}

static unsigned int sys_get_ms(void)
{
    int sec;
    double subsec;
    unsigned int ms;
    
    subsec = MXC_RTC_GetSubSecond() / 4096.0;
    sec = MXC_RTC_GetSecond();
    
    ms = (sec * 1000) + (int)(subsec * 1000);
    
    return ms;
}

int main(void)
{
    int                 result;
    mxc_eth_config_t    lwip_config;
    ip_addr_t           ping_target_ip;
    
    printf("*** Ping Example ***\n");
    
    result = MXC_RTC_Init(0, 0);
    
    if (result) {
        printf("RTC Initialization Failed ( 0x%X )\n", result);
        return result;
    }
    
    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("RTC Start Failed ( 0x%X )\n", result);
        return result;
    }
    
    result = config_emac();
    
    if (result) {
        printf("EMAC Driver Initialization Failed ( 0x%X )\n", result);
        return result;
    }
    
    lwip_config.name[0] = 'e';
    lwip_config.name[1] = '0';
    lwip_config.link_callback = link_callback_func;
    lwip_config.sys_get_ms    = sys_get_ms;
    
    result = MXC_ETH_Init(&lwip_config);
    
    if (result) {
        printf("LWIP Initialization Failed ( 0x%X )\n", result);
        return result;
    }
    
    /**
     * Ping Example Configuration:
     * To use DHCP instead of static ip, set "USE_DHCP" definition
     * in the "lwipcfg.h" and update the "ping_target_ip" if necessary
     */
    LWIP_PORT_INIT_GW(&ping_target_ip);
    ping_init(&ping_target_ip);
    
    while (1) {
        result = MXC_ETH_Tick();
        
        if (result) {
            printf("Error Occurred ( 0x%X )\n", result);
            break;
        }
    }
    
    return result;
}
