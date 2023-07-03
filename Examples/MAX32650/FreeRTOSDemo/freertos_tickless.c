/**
 * @file
 * @brief   Freertos tickless 
 * @details How to use RTC for freeRTOS
 *
 */

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

/* MXC */
#include "mxc_errors.h"
#include "board.h"
#include "mxc_assert.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* Maxim CMSIS */
#include "lp.h"
#include "rtc.h"
#include "pwrseq_regs.h"

#define RTC_RATIO (configRTC_TICK_RATE_HZ / configTICK_RATE_HZ)
#define MAX_SNOOZE 0xFF
#define MIN_SYSTICK 2
#define MIN_RTC_TICKS 5

static uint32_t residual = 0;

/*
 * Sleep-check function
 *
 * Your code should over-ride this weak function and return E_NO_ERROR if
 *  tickless sleep is permissible (ie. no UART/SPI/I2C activity). Any other
 *  return code will prevent FreeRTOS from entering tickless idle.
 */
__attribute__((weak)) int freertos_permit_tickless(void)
{
    return E_NO_ERROR;
}

/*
 * This function overrides vPortSuppressTicksAndSleep in portable/.../ARM_CM4F/port.c
 *
 * DEEPSLEEP mode will stop SysTick from counting, so that can't be
 *  used to wake up. Instead, calculate a wake-up period for the RTC to
 *  interrupt the WFI and continue execution.
 *
 */
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    volatile uint32_t rtc_ticks, rtc_ss_val;
    volatile uint32_t actual_ticks;
    volatile uint32_t pre_capture, post_capture;
    mxc_gpio_cfg_t uart_rx_pin = { MXC_GPIO2, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_IN,
                                   MXC_GPIO_PAD_NONE };

    /* Example:
   *
   *  configTICK_RATE_HZ      512
   *  configRTC_TICK_RATE_HZ 4096
   *
   *  RTC is 8x more accurate than the normal tick in this case. We can accumulate an error term and
   *   fix up when called again as the error term equals 1 task tick
   */

    /* We do not currently handle to case where the RTC is slower than the RTOS tick */
    MXC_ASSERT(configRTC_TICK_RATE_HZ >= configTICK_RATE_HZ);

    if (SysTick->VAL < MIN_SYSTICK) {
        /* Avoid sleeping too close to a systick interrupt */
        return;
    }

    /* Deep sleep time is limited */
    if (xExpectedIdleTime > (MAX_SNOOZE / RTC_RATIO)) {
        xExpectedIdleTime = (MAX_SNOOZE / RTC_RATIO);
    }

    /* Calculate the number of RTC ticks, but we need a few ticks to synchronize */
    rtc_ticks = (xExpectedIdleTime - 3UL) * RTC_RATIO;

    if ((xExpectedIdleTime < 4) || (rtc_ticks < MIN_RTC_TICKS)) {
        /* Finish out the rest of this tick with normal sleep */
        MXC_LP_EnterSleepMode();
        return;
    }

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

    /* Use sub-second roll-over to wake up */
    rtc_ss_val = 0xFFFFFFFF - (rtc_ticks - 1);
    MXC_RTC_SetSubsecondAlarm(rtc_ss_val);

    MXC_RTC->ctrl &= ~(MXC_F_RTC_CTRL_SSEC_ALARM_FL);
    MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT);

    /* Pin used for UART RX must have GPIO interrupt enabled */
    MXC_GPIO_EnableInt(uart_rx_pin.port, uart_rx_pin.mask);

    /* Snapshot the current RTC value */
    pre_capture = MXC_RTC->ssec;

    /* Sleep */
    MXC_LP_EnterDeepSleepMode();

    /* -- WAKE HERE -- */

    /* We'll need to wait for the RTC to synchronize */
    MXC_RTC->ctrl &= ~MXC_F_RTC_CTRL_READY;

    /* Remove interrupt flag and configuration for GPIO on UART RX pin */
    uart_rx_pin.port->int_clr = uart_rx_pin.mask;
    MXC_GPIO_DisableInt(uart_rx_pin.port, uart_rx_pin.mask);

    /* Snapshot the current RTC value */
    while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_READY)) {}
    post_capture = MXC_RTC->ssec;

    /* Dermine wake cause */
    if (MXC_RTC->ctrl & MXC_F_RTC_CTRL_SSEC_ALARM_FL) {
        /* RTC woke the processor */
        actual_ticks = rtc_ticks;
    } else {
        /* Determine the actual duration of sleep */
        if (post_capture < pre_capture) {
            /* Rollover */
            post_capture += 0x100;
        }
        actual_ticks = post_capture - pre_capture;
        if (actual_ticks > rtc_ticks) {
            /* Assert or just fix up */
            MXC_ASSERT_FAIL();
            actual_ticks = rtc_ticks;
        }

        if (RTC_RATIO > 1) {
            /* Add residual from any previous early wake */
            actual_ticks += residual;
            /* Find new residual */
            residual = actual_ticks % RTC_RATIO;
        }
    }

    /* RTC sub-second interrupt no longer desired */
    MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT);
    MXC_RTC->ctrl &= ~(MXC_F_RTC_CTRL_SSEC_ALARM_FL);

    /* Re-enable interrupts - see comments above the cpsid instruction()
     above. */
    __asm volatile("cpsie i");

    /*
   * Advance ticks by # actually elapsed
   */
    portENTER_CRITICAL();
    /* Future enhancement: Compare time in seconds to RTC and slew to correct */
    vTaskStepTick(actual_ticks / RTC_RATIO);
    portEXIT_CRITICAL();
}
