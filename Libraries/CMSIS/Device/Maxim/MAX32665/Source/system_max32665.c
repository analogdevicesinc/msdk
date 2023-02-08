
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "max32665.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "icc_regs.h"
#include "pwrseq_regs.h"
#include "simo_regs.h"
#include "mcr_regs.h"
#include "lp.h"

// Backup mode entry point
extern void Reset_Handler(void);

extern void (*const __isr_vector[])(void);

// Part defaults to HIRC/2 out of reset
uint32_t SystemCoreClock = HIRC_FREQ >> 1;

__weak void SystemCoreClockUpdate(void)
{
    uint32_t base_freq, div, clk_src;

    // Determine the clock source and frequency
    clk_src = (MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CLKSEL);
    switch (clk_src) {
    case MXC_S_GCR_CLKCN_CLKSEL_HIRC:
        base_freq = HIRC_FREQ;
        break;
    case MXC_S_GCR_CLKCN_CLKSEL_XTAL32M:
        base_freq = XTAL32M_FREQ;
        break;
    case MXC_S_GCR_CLKCN_CLKSEL_LIRC8:
        base_freq = LIRC8_FREQ;
        break;
    case MXC_S_GCR_CLKCN_CLKSEL_HIRC96:
        base_freq = HIRC96_FREQ;
        break;
    case MXC_S_GCR_CLKCN_CLKSEL_HIRC8:
        base_freq = HIRC8_FREQ;
        break;
    case MXC_S_GCR_CLKCN_CLKSEL_XTAL32K:
        base_freq = XTAL32K_FREQ;
        break;
    default:
        // Values 001 and 111 are reserved, and should never be encountered.
        base_freq = HIRC_FREQ;
        break;
    }
    // Clock divider is retrieved to compute system clock
    div = (MXC_GCR->clkcn & MXC_F_GCR_CLKCN_PSC) >> MXC_F_GCR_CLKCN_PSC_POS;

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

// This function can be implemented by the application to initialize the board
__weak int Board_Init(void)
{
    // Do nothing
    return 0;
}

__weak void PalSysInit(void) {}

/* This function is called just before control is transferred to main().
 *
 * You may over-ride this function in your program by defining a custom 
 *  SystemInit(), but care should be taken to reproduce the initialization
 *  steps or a non-functional system may result.
 */
__weak void SystemInit(void)
{
    /* Configure the interrupt controller to use the application vector 
     * table in flash. Initially, VTOR points to the ROM's table.
     */
    SCB->VTOR = (uint32_t)&__isr_vector;

    /* We'd like to switch to the fast clock, but can only do so if the 
     * core's operating voltage (VregO_B) is high enough to support it
     * Otherwise, we need to remain on the slow clock
     */
    if ((MXC_SIMO->vrego_b > 48) && (MXC_SIMO->buck_out_ready & 0x2)) {
        // Switch to fast clock on startup
        MXC_GCR->clkcn &= ~(MXC_S_GCR_CLKCN_PSC_DIV128);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC96);
    }

    /* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11
     * Grant full access, per "Table B3-24 CPACR bit assignments".
     * DDI0403D "ARMv7-M Architecture Reference Manual"
     */
    SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
    __DSB();
    __ISB();

    // Initialize backup mode entry point to safe default value.
    MXC_PWRSEQ->buretvec = (uint32_t)(Reset_Handler) | 1;

    // FIXME Pre-production parts: Enable TME, disable ICache Read Buffer, disable TME
    *(uint32_t *)0x40000c00 = 1;
    *(uint32_t *)0x4000040c = (1 << 6);
    *(uint32_t *)0x40000c00 = 0;

    // Flush and enable instruction cache
    MXC_ICC0->invalidate = 1;
    while (!(MXC_ICC0->cache_ctrl & MXC_F_ICC_CACHE_CTRL_RDY)) {}
    MXC_ICC0->cache_ctrl |= MXC_F_ICC_CACHE_CTRL_EN;
    while (!(MXC_ICC0->cache_ctrl & MXC_F_ICC_CACHE_CTRL_RDY)) {}

    SystemCoreClockUpdate();

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

    Board_Init();

    PalSysInit();
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
    while (1) {}
}
#endif /* __CC_ARM */
