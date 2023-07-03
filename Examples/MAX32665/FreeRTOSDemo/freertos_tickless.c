/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* Maxim CMSIS */
#include "mxc_device.h"
#include "board.h"
#include "mxc_assert.h"
#include "lp.h"
#include "pwrseq_regs.h"
#include "wut.h"
#include "mcr_regs.h"
#include "simo.h"
#include "icc.h"
#include "pb.h"
#include "led.h"
#include "uart.h"

#define WUT_RATIO (configRTC_TICK_RATE_HZ / configTICK_RATE_HZ)
#define MAX_WUT_SNOOZE (5 * configRTC_TICK_RATE_HZ)
#define MIN_SYSTICK 2
#define MIN_WUT_TICKS 50

static uint32_t wutSnooze = 0;
static int wutSnoozeValid = 0;

extern mxc_gpio_cfg_t uart_cts;
extern mxc_gpio_cfg_t uart_rts;

/* Enables/disables tick-less mode */
extern unsigned int disable_tickless;

extern void vApplicationIdleHook(void);

/*
 * Sleep-check function
 *
 * Your code should over-ride this weak function and return E_NO_ERROR if
 * tickless sleep is permissible (ie. no UART/SPI/I2C activity). Any other
 * return code will prevent FreeRTOS from entering tickless idle.
 */
__attribute__((weak)) int freertos_permit_tickless(void)
{
    if (disable_tickless == 1) {
        return E_BUSY;
    }

    /* Prevent characters from being corrupted if still transmitting,
      UART will shutdown in deep sleep  */
    if (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

/*
 *  Switch the system clock to the HIRC / 4
 *
 *  Enable the HIRC, set the divide ration to /4, and disable the HIRC96 oscillator.
 */
void switchToHIRCD4(void)
{
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_PSC, MXC_S_GCR_CLKCN_PSC_DIV4);
    MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_HIRC);
    /* Disable unused clocks */
    while (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CKRDY)) {}
    /* Wait for the switch to occur */
    MXC_GCR->clkcn &= ~(MXC_F_GCR_CLKCN_HIRC96M_EN);
    SystemCoreClockUpdate();
}

/*
 *  Switch the system clock to the HIRC96
 *
 *  Enable the HIRC96, set the divide ration to /1, and disable the HIRC oscillator.
 */
void switchToHIRC(void)
{
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_PSC, MXC_S_GCR_CLKCN_PSC_DIV1);
    MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC96M_EN;
    MXC_SETFIELD(MXC_GCR->clkcn, MXC_F_GCR_CLKCN_CLKSEL, MXC_S_GCR_CLKCN_CLKSEL_HIRC96);
    /* Disable unused clocks */
    while (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CKRDY)) {}
    /* Wait for the switch to occur */
    MXC_GCR->clkcn &= ~(MXC_F_GCR_CLKCN_HIRC_EN);
    SystemCoreClockUpdate();
}

/*
 *  Enter deep sleep mode
 *
 *  Adjust system clocks and voltages for deep sleep.
 */
static void deepSleep(void)
{
    MXC_ICC_Disable();
    MXC_LP_ICache0Shutdown();

    /* Shutdown unused power domains */
    MXC_PWRSEQ->lpcn |= MXC_F_PWRSEQ_LPCN_BGOFF;

    /* Prevent SIMO soft start on wakeup */
    MXC_LP_FastWakeupDisable();

    /* Enable VDDCSWEN=1 prior to enter backup/deepsleep mode */
    MXC_MCR->ctrl |= MXC_F_MCR_CTRL_VDDCSWEN;

    /* Switch the system clock to a lower frequency to conserve power in deep sleep
       and reduce current inrush on wakeup */
    switchToHIRCD4();

    /* Reduce VCOREB to 0.81v */
    MXC_SIMO_SetVregO_B(810);

    MXC_LP_EnterDeepSleepMode();

    /*  If VCOREA not ready and VCOREB ready, switch VCORE=VCOREB 
    (set VDDCSW=2’b01). Configure VCOREB=1.1V wait for VCOREB ready. */

    /* Check to see if VCOREA is ready on  */
    if (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYC)) {
        /* Wait for VCOREB to be ready */
        while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}

        /* Move VCORE switch back to VCOREB */
        MXC_MCR->ctrl = (MXC_MCR->ctrl & ~(MXC_F_MCR_CTRL_VDDCSW)) |
                        (0x1 << MXC_F_MCR_CTRL_VDDCSW_POS);

        /* Raise the VCORE_B voltage */
        while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
        MXC_SIMO_SetVregO_B(1000);
        while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYB)) {}
    }

    MXC_LP_ICache0PowerUp();
    MXC_ICC_Enable();

    /* Restore the system clock */
    switchToHIRC();
}

/*
 *  Snooze the wake up timer
 *
 *  Prevent the system from entering deep sleep for MAX_WUT_SNOOZE WUT ticks.
 */
void wutHitSnooze(void)
{
    wutSnooze = MXC_WUT_GetCount() + MAX_WUT_SNOOZE;
    wutSnoozeValid = 1;
}

/*
 * This function overrides vPortSuppressTicksAndSleep in portable/.../ARM_CM4F/port.c
 *
 * DEEPSLEEP mode will stop SysTick from counting, so that can't be
 * used to wake up. Instead, calculate a wake-up period for the WUT to
 * interrupt the WFI and continue execution.
 *
 */
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t wut_ticks;
    uint32_t actual_ticks;
    uint32_t pre_capture, post_capture;

    /* We do not currently handle to case where the WUT is slower than the RTOS tick */
    MXC_ASSERT(configRTC_TICK_RATE_HZ >= configTICK_RATE_HZ);

    if (SysTick->VAL < MIN_SYSTICK) {
        /* Avoid sleeping too close to a systick interrupt */
        vApplicationIdleHook();
        return;
    }

    /* Calculate the number of WUT ticks, but we need one to synchronize */
    wut_ticks = (xExpectedIdleTime - 1) * WUT_RATIO;

    if (wut_ticks > MAX_WUT_SNOOZE) {
        wut_ticks = MAX_WUT_SNOOZE;
    }

    /* Check to see if we meet the minimum requirements for deep sleep */
    if (wut_ticks < MIN_WUT_TICKS) {
        vApplicationIdleHook();
        return;
    }

    /* Check the WUT snooze */
    if (wutSnoozeValid && (MXC_WUT_GetCount() < wutSnooze)) {
        vApplicationIdleHook();
        return;
    }
    wutSnoozeValid = 0;

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
     method as that will mask interrupts that should exit sleep mode. */
    __asm volatile("cpsid i");

    /* If a context switch is pending or a task is waiting for the scheduler
     to be unsuspended then abandon the low power entry. */
    /* Also check the MXC drivers for any in-progress activity */
    if ((eTaskConfirmSleepModeStatus() == eAbortSleep) ||
        (freertos_permit_tickless() != E_NO_ERROR)) {
        /* Re-enable interrupts - see comments above the cpsid instruction()
       above. */
        __asm volatile("cpsie i");
        return;
    }

    /* Set RTS to prevent the console UART from transmitting */
    MXC_GPIO_OutSet(uart_rts.port, uart_rts.mask);

    /* Snapshot the current WUT value */
    MXC_WUT_Edge();
    pre_capture = MXC_WUT_GetCount();
    MXC_WUT_SetCompare(pre_capture + wut_ticks);
    MXC_WUT_Edge();

    LED_Off(SLEEP_LED);

    deepSleep();

    LED_On(SLEEP_LED);

    post_capture = MXC_WUT_GetCount();
    actual_ticks = post_capture - pre_capture;

    LED_On(1);

    /*  Snooze the deep sleep if we woke up on the UART CTS GPIO */
    if ((uart_cts.port == MXC_GPIO0) && (MXC_PWRSEQ->lpwkst0 & uart_cts.mask)) {
        wutHitSnooze();
    } else if ((uart_cts.port == MXC_GPIO1) && (MXC_PWRSEQ->lpwkst1 & uart_cts.mask)) {
        wutHitSnooze();
    }

    /* Clear RTS */
    MXC_GPIO_OutClr(uart_rts.port, uart_rts.mask);

    /* Re-enable interrupts - see comments above the cpsid instruction()
     above. */
    __asm volatile("cpsie i");

    /*
   * Advance ticks by # actually elapsed
   */
    portENTER_CRITICAL();
    vTaskStepTick((actual_ticks / WUT_RATIO));
    portEXIT_CRITICAL();
}
