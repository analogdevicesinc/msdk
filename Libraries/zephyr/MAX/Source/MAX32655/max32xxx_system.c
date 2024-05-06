/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc.
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

#include "max32655.h"
#include "gcr_regs.h"
#include "icc.h"
#include "simo.h"
#include "mcr_regs.h"

static int pre_init(void)
{
    uint32_t psc = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_DIV;

    /* Divide down system clock until SIMO is ready */
    MXC_GCR->clkctrl = (MXC_GCR->clkctrl & ~(MXC_F_GCR_CLKCTRL_SYSCLK_DIV)) |
                       (MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128);

    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYA)) {}
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC)) {}

    /* Restore system clock divider */
    MXC_GCR->clkctrl = (MXC_GCR->clkctrl & ~(MXC_F_GCR_CLKCTRL_SYSCLK_DIV)) | (psc);

    return 0;
}

/* 
 * This function is called during boot up.
 */
void max32xx_system_init(void)
{
    pre_init();

    /* Disable SRAM ECC until it is handled on zephyr side */
    MXC_MCR->eccen &= ~MXC_F_MCR_ECCEN_RAM0;

    /* Enable instruction cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Setup the SIMO voltages */
    MXC_SIMO_SetVregO_A(1750);
    while (MXC_SIMO_GetOutReadyA() != E_NO_ERROR) {}
    MXC_SIMO_SetVregO_B(1100);
    while (MXC_SIMO_GetOutReadyB() != E_NO_ERROR) {}
    MXC_SIMO_SetVregO_C(1100);
    while (MXC_SIMO_GetOutReadyC() != E_NO_ERROR) {}
}
