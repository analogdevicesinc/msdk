/* ****************************************************************************
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
 *************************************************************************** */

/*******   Includes      *******/
#include "emac.h"
#include "emac_reva.h"
#include "mxc_sys.h"

/*******   Definitions   *******/
/* **** Definitions **** */
#define CONFIG_EMAC_SEARCH_PHY                              /**! Enable EMAC PHY ID Search */


/*******   Functions     *******/
/* ************************************************************************* */
/* Control/Configuration Functions                                           */
/* ************************************************************************* */

int MXC_EMAC_Init(mxc_emac_config_t* config)
{
    if (!config) {
        return E_NULL_PTR;
    }
    
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_EMAC);
    MXC_GPIO_Config(&gpio_cfg_emac_P2a);
    MXC_GPIO_Config(&gpio_cfg_emac_P2b);
    
    return MXC_EMAC_RevA_Init ((mxc_emac_reva_config_t*) config);
}

int MXC_EMAC_SetConfiguration(mxc_emac_config_t* config)
{
    return MXC_EMAC_RevA_SetConfiguration ((mxc_emac_reva_config_t*) config);
}

int MXC_EMAC_SetHwAddr(unsigned char* enetaddr)
{
    return MXC_EMAC_RevA_SetHwAddr(enetaddr);
}

int MXC_EMAC_EnableInterruptEvents(unsigned int events)
{
    return MXC_EMAC_RevA_EnableInterruptEvents(events);
}

int MXC_EMAC_DisableInterruptEvents(unsigned int events)
{
    return MXC_EMAC_RevA_DisableInterruptEvents(events);
}

/* ************************************************************************* */
/* Low-Level Functions                                                       */
/* ************************************************************************* */
int MXC_EMAC_Start(void)
{
    return MXC_EMAC_RevA_Start();
}

int MXC_EMAC_Stop(void)
{
    return MXC_EMAC_RevA_Stop();
}

int MXC_EMAC_ReadLinkStatus(void)
{
    return MXC_EMAC_RevA_ReadLinkStatus();
}

/* ************************************************************************* */
/* Transaction-Level Functions                                               */
/* ************************************************************************* */
int MXC_EMAC_SendSync(const void* packet, unsigned int length)
{
    return MXC_EMAC_RevA_SendSync(packet, length);
}

int MXC_EMAC_SendAsync(const void* packet, unsigned int length)
{
    return MXC_EMAC_RevA_SendAsync(packet, length);
}

int MXC_EMAC_Recv(void* rx_buff, unsigned int max_len)
{
    return MXC_EMAC_RevA_Recv(rx_buff, max_len);
}

void MXC_EMAC_IrqHandler(void)
{
    MXC_EMAC_RevA_IrqHandler();
}
