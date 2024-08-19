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

//#include "smoh.h"
#include "skbd.h"
#include "skbd_reva.h"
#include "gpio.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "mxc_lock.h"

int MXC_SKBD_PreInit(void)
{
    return MXC_SKBD_RevA_PreInit();
}

int MXC_SKBD_Init(mxc_skbd_config_t config)
{
    /* Enable the SKBD clock i.e. just in case clock is disabled */
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CTB);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SKBD);

    /* Set the SKBD GPIO configs */
    MXC_GPIO_Config(&gpio_cfg_skbd_P2);

    return MXC_SKBD_RevA_Init((mxc_skbd_reva_regs_t *)MXC_SKBD, config);
}

int MXC_SKBD_EnableInterruptEvents(unsigned int events)
{
    return MXC_SKBD_RevA_EnableInterruptEvents((mxc_skbd_reva_regs_t *)MXC_SKBD, events);
}

int MXC_SKBD_DisableInterruptEvents(unsigned int events)
{
    return MXC_SKBD_RevA_DisableInterruptEvents((mxc_skbd_reva_regs_t *)MXC_SKBD, events);
}

int MXC_SKBD_ClearInterruptStatus(unsigned int status)
{
    return MXC_SKBD_RevA_ClearInterruptStatus((mxc_skbd_reva_regs_t *)MXC_SKBD, status);
}

int MXC_SKBD_InterruptStatus(unsigned int *status)
{
    return MXC_SKBD_RevA_InterruptStatus((mxc_skbd_reva_regs_t *)MXC_SKBD, status);
}

int MXC_SKBD_ReadKeys(mxc_skbd_keys_t *keys)
{
    return MXC_SKBD_RevA_ReadKeys((mxc_skbd_reva_regs_t *)MXC_SKBD, (mxc_skbd_reva_keys_t *)keys);
}

int MXC_SKBD_Close(void)
{
    /* Reset the SKBD controller */
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SKBD);

    /* Wait until SKBD reset completes */
    while (MXC_F_GCR_RST1_SKBD & MXC_GCR->rst1) {}

    return MXC_SKBD_RevA_Close();
}

const char *MXC_SKBD_GetVersion(void)
{
    return MXC_SKBD_REVA_VERSION_STRING;
}
