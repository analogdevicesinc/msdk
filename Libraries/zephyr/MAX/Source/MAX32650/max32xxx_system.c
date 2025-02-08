/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

#include "max32650.h"
#include "mxc_sys.h"
#include "icc.h"

/* 
 * This function is called during boot up.
 */
void max32xx_system_init(void)
{
    /* Workaround: Write to SCON register on power up to fix trim issue for SRAM */
    MXC_GCR->scon = (MXC_GCR->scon & ~(MXC_F_GCR_SCON_OVR)) | (MXC_S_GCR_SCON_OVR_1V1);

    /* Erratum #?: Adjust register timing for VCORE == 1.1v, prevents USB failure. 2017-10-04 ZNM/HTN */
    MXC_GCR->scon |= MXC_S_GCR_SCON_OVR_1V1;

    // Flush and enable instruction cache
    MXC_ICC->invalidate = 1;
    while (!(MXC_ICC->cache_ctrl & MXC_F_ICC_CACHE_CTRL_READY)) {}
    MXC_ICC->cache_ctrl |= MXC_F_ICC_CACHE_CTRL_ENABLE;
    while (!(MXC_ICC->cache_ctrl & MXC_F_ICC_CACHE_CTRL_READY)) {}

    /* Shutdown all peripheral clocks initially.  They will be re-enabled by each periph's init function. */
    /* GPIO Clocks are left enabled */
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_USB);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TFT);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_DMA);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI2);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TPU);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TIMER0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TIMER1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TIMER2);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TIMER3);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TIMER4);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TIMER5);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ADC);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_PT);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIPF);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIPM);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART2);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TRNG);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_FLC);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_HBC);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SDMA);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SEMA);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SDHC);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ICACHE);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ICACHEXIP);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI3);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2S);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
}
