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

/*******   Includes      *******/
#include "emac.h"
#include "emac_reva.h"
#include "mxc_sys.h"

/*******   Definitions   *******/
/* **** Definitions **** */
#define CONFIG_EMAC_SEARCH_PHY /**! Enable EMAC PHY ID Search */

/*******   Functions     *******/
/* ************************************************************************* */
/* Control/Configuration Functions                                           */
/* ************************************************************************* */

int MXC_EMAC_Init(mxc_emac_config_t *config)
{
    if (!config) {
        return E_NULL_PTR;
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_EMAC);
    MXC_GPIO_Config(&gpio_cfg_emac_P2a);
    MXC_GPIO_Config(&gpio_cfg_emac_P2b);

    return MXC_EMAC_RevA_Init((mxc_emac_reva_config_t *)config);
}

int MXC_EMAC_SetConfiguration(mxc_emac_config_t *config)
{
    return MXC_EMAC_RevA_SetConfiguration((mxc_emac_reva_config_t *)config);
}

int MXC_EMAC_SetHwAddr(unsigned char *enetaddr)
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
int MXC_EMAC_SendSync(const void *packet, unsigned int length)
{
    return MXC_EMAC_RevA_SendSync(packet, length);
}

int MXC_EMAC_SendAsync(const void *packet, unsigned int length)
{
    return MXC_EMAC_RevA_SendAsync(packet, length);
}

int MXC_EMAC_Recv(void *rx_buff, unsigned int max_len)
{
    return MXC_EMAC_RevA_Recv(rx_buff, max_len);
}

void MXC_EMAC_IrqHandler(void)
{
    MXC_EMAC_RevA_IrqHandler();
}
