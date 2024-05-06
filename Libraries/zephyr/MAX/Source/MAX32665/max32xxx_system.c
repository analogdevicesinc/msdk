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

#include "max32665.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "icc_regs.h"
#include "pwrseq_regs.h"
#include "simo_regs.h"
#include "mcr_regs.h"

static int pre_init(void)
{
    uint32_t psc = MXC_GCR->clkcn & MXC_F_GCR_CLKCN_PSC;

    /* Disable USB switch to minimize current consumption */
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_USBSWEN_N;

    /* Divide down system clock until SIMO is ready */
    MXC_GCR->clkcn = (MXC_GCR->clkcn & ~(MXC_F_GCR_CLKCN_PSC)) | (MXC_S_GCR_CLKCN_PSC_DIV128);

    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYA)) {}
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC)) {}

    /* Restore system clock divider */
    MXC_GCR->clkcn = (MXC_GCR->clkcn & ~(MXC_F_GCR_CLKCN_PSC)) | (psc);

    /* Set the proper OVR setting */
    MXC_GCR->scon = (MXC_GCR->scon & ~(MXC_F_GCR_SCON_OVR)) | (MXC_S_GCR_SCON_OVR_1_1V);

    return 0;
}

/* 
 * This function is called during boot up.
 */
void max32xx_system_init(void)
{
    pre_init();

    /* Disable SRAM ECC until it is handled on zephyr side */
    MXC_MCR->eccen &= ~(MXC_F_MCR_ECCEN_SYSRAM0ECCEN | MXC_F_MCR_ECCEN_SYSRAM1ECCEN |
                        MXC_F_MCR_ECCEN_SYSRAM2ECCEN | MXC_F_MCR_ECCEN_SYSRAM3ECCEN |
                        MXC_F_MCR_ECCEN_SYSRAM4ECCEN | MXC_F_MCR_ECCEN_SYSRAM5ECCEN);

    /* We'd like to switch to the fast clock, but can only do so if the 
     * core's operating voltage (VregO_B) is high enough to support it
     * Otherwise, we need to remain on the slow clock
     */
    if ((MXC_SIMO->vrego_b > 48) && (MXC_SIMO->buck_out_ready & 0x2)) {
        // Switch to fast clock on startup
        MXC_GCR->clkcn &= ~(MXC_S_GCR_CLKCN_PSC_DIV128);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC96);
    }

    // FIXME Pre-production parts: Enable TME, disable ICache Read Buffer, disable TME
    *(uint32_t *)0x40000c00 = 1;
    *(uint32_t *)0x4000040c = (1 << 6);
    *(uint32_t *)0x40000c00 = 0;

    // Flush and enable instruction cache
    MXC_ICC0->invalidate = 1;
    while (!(MXC_ICC0->cache_ctrl & MXC_F_ICC_CACHE_CTRL_RDY)) {}
    MXC_ICC0->cache_ctrl |= MXC_F_ICC_CACHE_CTRL_EN;
    while (!(MXC_ICC0->cache_ctrl & MXC_F_ICC_CACHE_CTRL_RDY)) {}

    // Set all GPIO to 25K pullup mode by default
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_GPIO0->vssel |= 0xFFFFFFFF;
    MXC_GPIO0->ps |= 0xFFFFFFFF;
    MXC_GPIO0->pad_cfg1 |= 0xFFFFFFFF;
    MXC_GPIO0->pad_cfg2 &= ~(0xFFFFFFFF);
    MXC_GPIO1->vssel |= 0xFFFFFFFF;
    MXC_GPIO1->ps |= 0xFFFFFFFF;
    MXC_GPIO1->pad_cfg1 |= 0xFFFFFFFF;
    MXC_GPIO1->pad_cfg2 &= ~(0xFFFFFFFF);

    /* Disable fast wakeup due to issues with SIMO in wakeup */
    MXC_PWRSEQ->lpcn &= ~MXC_F_PWRSEQ_LPCN_FWKM;
}
