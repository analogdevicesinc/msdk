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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "max32650.h"
#include "gcr_regs.h"
#include "mxc_sys.h"
#include "usbhs_regs.h"
#include "flc_regs.h"
#include "icc_regs.h"
#include "mxc_errors.h"

extern void (*const __isr_vector[])(void);
uint32_t SystemCoreClock = 0;
uint8_t ChipRevision = 0;

/*
The libc implementation from GCC 11+ depends on _getpid and _kill in some places.
There is no concept of processes/PIDs in the baremetal PeriphDrivers, therefore
we implement stub functions that return an error code to resolve linker warnings.
*/
__weak int _getpid(void)
{
    return E_NOT_SUPPORTED;
}

__weak int _kill(void)
{
    return E_NOT_SUPPORTED;
}

__weak void SystemCoreClockUpdate(void)
{
    uint32_t base_freq, div, clk_src;

    // Get the clock source and frequency
    clk_src = (MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_SYSOSC_SEL);
    if (clk_src == MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HFXIN) {
        base_freq = HFX_FREQ;
    } else if (clk_src == MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_NANORING) {
        base_freq = NANORING_FREQ;
    } else if (clk_src == MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96) {
        base_freq = HIRC96_FREQ;
    } else if (clk_src == MXC_S_GCR_CLK_CTRL_SYSOSC_SEL_HIRC8) {
        base_freq = HIRC8_FREQ;
    } else {
#ifndef CRYPTO_FREQ
        if (ChipRevision > 0xA1) {
            base_freq = CRYPTO_FREQ_A3;
        } else {
            base_freq = CRYPTO_FREQ_A1;
        }
#else
        base_freq = CRYPTO_FREQ;
#endif
    }

    // Get the clock divider
    div = (MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE) >>
          MXC_F_GCR_CLK_CTRL_SYSCLK_PRESCALE_POS;

    SystemCoreClock = base_freq >> div;
}

/* This function is called before C runtime initialization and can be
 * implemented by the application for early initializations. If a value other
 * than '0' is returned, the C runtime initialization will be skipped.
 *
 * You may over-ride this function in your program by defining a custom
 *  PreInit(), but care should be taken to reproduce the initialization steps
 *  or a non-functional system may result.
 */
__weak int PreInit(void)
{
    /* Workaround: Write to SCON register on power up to fix trim issue for SRAM */
    MXC_GCR->scon = (MXC_GCR->scon & ~(MXC_F_GCR_SCON_OVR)) | (MXC_S_GCR_SCON_OVR_1V1);
    return 0;
}

/* This function is called before the Board_Init function.  This weak 
 * implementation does nothing, but you may over-ride this function in your 
 * program if you want to configure the state of all pins prior to the 
 * application running.  This is useful when using external tools (like a
 * Pin Mux configuration tool) that generate code to initialize the pins.
 */
__weak void PinInit(void)
{
    /* Do nothing */
}

__weak int PeripheralInit(void)
{
    /* Do nothing */
    return E_NO_ERROR;
}

/* This function can be implemented by the application to initialize the board */
__weak int Board_Init(void)
{
    /* Do nothing */
    return 0;
}

/* This function is called just before control is transferred to main().
 *
 * You may over-ride this function in your program by defining a custom
 *  SystemInit(), but care should be taken to reproduce the initialization
 *  steps or a non-functional system may result.
 */
__weak void SystemInit(void)
{
    ChipRevision = MXC_SYS_GetRev();
    /* Configure the interrupt controller to use the application vector table in */
    /* the application space */
    SCB->VTOR = (uint32_t)__isr_vector;

    /* MAX3265x ROM turns off interrupts, which is not the same as the reset state. */
    __enable_irq();

    /* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11 */
    /* Grant full access, per "Table B3-24 CPACR bit assignments". */
    /* DDI0403D "ARMv7-M Architecture Reference Manual" */
    SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
    __DSB();
    __ISB();

    /* Change system clock source to the main high-speed clock */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC96);
    SystemCoreClockUpdate();

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

    PinInit();
    Board_Init();

    /* Call peripheral init after board init to ensure the user's configuration
     * is not overwritten    
     */
    PeripheralInit();
}

#if defined(__CC_ARM)
/* Function called post memory initialization (post scatter load) in the Keil Toolchain, which
 * we are using to call the system core clock update and board initialization
 * to prevent data corruption if they are called from SystemInit. */
extern void $Super$$__main_after_scatterload(void);
void $Sub$$__main_after_scatterload(void)
{
    SystemInit();
    $Super$$__main_after_scatterload();
}
#endif /* __CC_ARM */
