/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include "mxc_device.h"
#include "board.h"
#include "mxc_assert.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* Maxim CMSIS */
#include "lp.h"
#include "pwrseq_regs.h"
#include "wut.h"
#include "mcr_regs.h"
#include "icc.h"
#include "pb.h"
#include "led.h"

#define WUT_RATIO           (configRTC_TICK_RATE_HZ / configTICK_RATE_HZ)
#define MAX_WUT_SNOOZE      (5*configRTC_TICK_RATE_HZ)
#define MIN_SYSTICK         2
#define MIN_WUT_TICKS       50

static uint32_t wutSnooze = 0;
static int wutSnoozeValid = 0;

extern mxc_gpio_cfg_t uart_cts;
extern mxc_gpio_cfg_t uart_rts;

/*
 * Sleep-check function
 *
 * Your code should over-ride this weak function and return E_NO_ERROR if
 * tickless sleep is permissible (ie. no UART/SPI/I2C activity). Any other
 * return code will prevent FreeRTOS from entering tickless idle.
 */
__attribute__((weak)) int freertos_permit_tickless(void)
{
  return E_NO_ERROR;
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
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
  uint32_t wut_ticks;
  uint32_t actual_ticks;
  uint32_t pre_capture, post_capture;

  /* We do not currently handle to case where the WUT is slower than the RTOS tick */
  MXC_ASSERT(configRTC_TICK_RATE_HZ >= configTICK_RATE_HZ);

  if (SysTick->VAL < MIN_SYSTICK) {
    /* Avoid sleeping too close to a systick interrupt */
    return;
  }

  /* Calculate the number of WUT ticks, but we need one to synchronize */
  wut_ticks = (xExpectedIdleTime - 1) * WUT_RATIO;

  if(wut_ticks > MAX_WUT_SNOOZE) {
    wut_ticks = MAX_WUT_SNOOZE;
  }

  /* Check to see if we meet the minimum requirements for deep sleep */
  if (wut_ticks < MIN_WUT_TICKS) {
    /* Finish out the rest of this tick with normal sleep */
    MXC_LP_EnterSleepMode();
    return;
  }

  /* Check the WUT snooze */
  if(wutSnoozeValid && (MXC_WUT_GetCount() < wutSnooze)) {
    /* Finish out the rest of this tick with normal sleep */
    MXC_LP_EnterSleepMode();
    return;
  }
  wutSnoozeValid = 0;

  /* Enter a critical section but don't use the taskENTER_CRITICAL()
     method as that will mask interrupts that should exit sleep mode. */
  __asm volatile( "cpsid i" );

  /* If a context switch is pending or a task is waiting for the scheduler
     to be unsuspended then abandon the low power entry. */
  /* Also check the MXC drivers for any in-progress activity */
  if ((eTaskConfirmSleepModeStatus() == eAbortSleep) ||
      (freertos_permit_tickless() != E_NO_ERROR)) {
    /* Re-enable interrupts - see comments above the cpsid instruction()
       above. */
    __asm volatile( "cpsie i" );
    return;
  }

  /* Set RTS to prevent the console UART from transmitting */
  MXC_GPIO_OutSet(uart_rts.port, uart_rts.mask);

  /* Snapshot the current WUT value */
  MXC_WUT_Edge();
  pre_capture = MXC_WUT_GetCount();
  MXC_WUT_SetCompare(pre_capture+wut_ticks);
  MXC_WUT_Edge();

  LED_Off(1);

  MXC_LP_EnterStandbyMode();

  post_capture = MXC_WUT_GetCount();
  actual_ticks = post_capture - pre_capture;

  LED_On(1);

  /*  Snooze the deep sleep if we woke up on the UART CTS GPIO */
  if((uart_cts.port == MXC_GPIO0) && (MXC_PWRSEQ->lpwkst0 & uart_cts.mask)) {
    wutHitSnooze();
  } else if ((uart_cts.port == MXC_GPIO1) && (MXC_PWRSEQ->lpwkst1 & uart_cts.mask)) {
    wutHitSnooze();
  }

  /* Clear RTS */
  MXC_GPIO_OutClr(uart_rts.port, uart_rts.mask);

  /* Re-enable interrupts - see comments above the cpsid instruction()
     above. */
  __asm volatile( "cpsie i" );

  /*
   * Advance ticks by # actually elapsed
   */
  portENTER_CRITICAL();
  vTaskStepTick( (actual_ticks / WUT_RATIO) );
  portEXIT_CRITICAL();
}
