
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "max78000.h"
#include "gcr_regs.h"

volatile uint32_t mailbox __attribute__((section(".mailbox")));
uint32_t SystemCoreClock __attribute__((section(".shared")));

void __enable_irq(void)
{
    // Set the MIE bit if we're outside the interrupt context
    __asm volatile("csrw mstatus, 0x8");
}

__weak void SystemCoreClockUpdate(void)
{
    // uint32_t base_freq, div, clk_src;

    // ARM core does this stuff for us
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
    // ARM core does this stuff for us
    // but riscv needs variables initialized
    SystemCoreClockUpdate();

    Board_Init();
}
