/**
 * @file       system_max32660.c
 * @brief      System-level initialization implementation file
 */

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
#include "max32660.h"
#include "gcr_regs.h"
#include "pwrseq_regs.h"
#include "tmr_regs.h"
#include "wdt_regs.h"
#include "mxc_sys.h"

extern void (*const __isr_vector[])(void);
uint32_t SystemCoreClock = HIRC96_FREQ;

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
    uint32_t base_freq, div, clk_src, ovr;

    // Get the clock source and frequency
    clk_src = (MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CLKSEL);

    if (clk_src == MXC_S_GCR_CLK_CTRL_CLKSEL_HFXIN) {
        base_freq = HFX_FREQ;
    } else {
        if (clk_src == MXC_S_GCR_CLK_CTRL_CLKSEL_NANORING) {
            base_freq = NANORING_FREQ;
        } else {
            ovr = (MXC_PWRSEQ->lp_ctrl & MXC_F_PWRSEQ_LP_CTRL_OVR);
            if (ovr == MXC_S_PWRSEQ_LP_CTRL_OVR_0_9V) {
                base_freq = HIRC96_FREQ / 4;
            } else {
                if (ovr == MXC_S_PWRSEQ_LP_CTRL_OVR_1_0V) {
                    base_freq = HIRC96_FREQ / 2;
                } else {
                    base_freq = HIRC96_FREQ;
                }
            }
        }
    }

    // Get the clock divider
    div = (MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_PSC) >> MXC_F_GCR_CLK_CTRL_PSC_POS;

    SystemCoreClock = base_freq >> div;
}

/* This function is called before C runtime initialization and can be
 * implemented by the application for early initializations. If a value other
 * than '0' is returned, the C runtime initialization will be skipped.
 *
 * You may over-ride this function in your program by defining a custom
 *  PreInit(), but care should be taken to reproduce the initilization steps
 *  or a non-functional system may result.
 */
__weak int PreInit(void)
{
    /* Do nothing */
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
    /* Configure the interrupt controller to use the application vector table in */
    /* the application space */
    /* IAR & Keil must set vector table after all memory initialization. */
    SCB->VTOR = (uint32_t)__isr_vector;

    MXC_WDT0->ctrl &=
        ~MXC_F_WDT_CTRL_WDT_EN; /* Turn off watchdog. Application can re-enable as needed. */

    /* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11 */
    /* Grant full access, per "Table B3-24 CPACR bit assignments". */
    /* DDI0403D "ARMv7-M Architecture Reference Manual" */
    SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
    __DSB();
    __ISB();

    /* Switch system clock to HIRC */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC);

    /* Disable clocks to peripherals by default to reduce power */
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_DMA);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR0);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR1);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR2);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C1);

    PinInit();
    Board_Init();
}

#if defined(__CC_ARM)
/* Global variable initialization does not occur until post scatterload in Keil tools.*/

/* External function called after our post scatterload function implementation. */
extern void $Super$$__main_after_scatterload(void);

/**
 * @brief   Initialization function for SystemCoreClock and Board_Init.
 * @details $Sub$$__main_after_scatterload is called during system startup in the Keil
 *          toolset. Global variable and static variable space must be set up by the compiler
 *          prior to using these memory spaces. Setting up the SystemCoreClock and Board_Init
 *          require global memory for variable storage and are called from this function in
 *          the Keil tool chain.
 */
void $Sub$$__main_after_scatterload(void)
{
    SystemInit();
    $Super$$__main_after_scatterload();
}
#endif /* __CC_ARM */
