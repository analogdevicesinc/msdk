/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

/**
 * @file       nspc_me30.c
 * @brief      Non-Secure Privilege Controller (NSPC) Driver for the MAX32657 (ME30).
 * @details    This driver is used to control the privilege policy of data 
 *              flowing to/from peripherals though the APB Peripheral Proection
 *              Controllers (PPC).
 *             The NSPC is only readable via Non-Secure Privileged access.
 */

/**
 * NSPC can only be accessed from the non-secure world.
 */
#if CONFIG_TRUSTED_EXECUTION_SECURE == 0

/**** Includes ****/
#include <stdbool.h>
#include <stdint.h>
#include "mxc_device.h"
#include "nspc.h"
#include "nspc_regs.h"

/**** Definitions ****/

/**** Globals ****/

void MXC_NSPC_SetPrivAccess(mxc_nspc_periph_t periph, mxc_nspc_priv_t priv)
{
    if (priv == MXC_NSPC_UNPRIVILEGED) {
        MXC_NSPC->apbpriv |= periph;
    } else {
        MXC_NSPC->apbpriv &= ~periph;
    }
}

// TODO(SW): Check function name.
void MXC_NSPC_DMA_SetPrivAccess(mxc_nspc_priv_t priv)
{
    if (priv == MXC_NSPC_UNPRIVILEGED) {
        MXC_NSPC->ahbmpriv |= MXC_F_NSPC_AHBMPRIV_DMA;
    } else {
        MXC_NSPC->ahbmpriv &= ~MXC_F_NSPC_AHBMPRIV_DMA;
    }
}

#endif
