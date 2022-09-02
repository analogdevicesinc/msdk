/*
 * @file
 * @brief   Demonstrates the various low power modes.
 *
 * @details Iterates through the various low power modes, using either the RTC
 *          alarm or a GPIO to wake from each.  #defines determine which wakeup
 *          source to use.  Once the code is running, you can measure the
 *          current used on the VCORE rail.
 *
 *          The power states shown are:
 *            1. Active mode power with all clocks on
 *            2. Active mode power with peripheral clocks disabled
 *            3. Active mode power with unused RAMs in light sleep mode
 *            4. Active mode power with unused RAMS shut down
 *            5. SLEEP mode
 *            6. BACKGROUND mode
 *            7. DEEPSLEEP mode
 *            8. BACKUP mode
 */

/* *****************************************************************************
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

#include <stdio.h>
#include <stdint.h>
#include "mxc_errors.h"
#include "pb.h"
#include "led.h"
#include "lp.h"
#include "icc.h"
#include "emcc.h"
#include "rtc.h"
#include "uart.h"
#include "nvic_table.h"

#define DELAY_IN_SEC 2
#define USE_CONSOLE  1

#define USE_BUTTON 1
#define USE_ALARM  0

#define DO_SLEEP      1
#define DO_BACKGROUND 0
#define DO_DEEPSLEEP  1
#define DO_BACKUP     0

#if (!(USE_BUTTON || USE_ALARM))
#error "You must set either USE_BUTTON or USE_ALARM to 1."
#endif
#if (USE_BUTTON && USE_ALARM)
#error "You must select either USE_BUTTON or USE_ALARM, not both."
#endif

// *****************************************************************************
#if USE_ALARM
volatile int alarmed;
void alarmHandler(void)
{
    int flags = MXC_RTC_GetFlags();
    alarmed   = 1;

    if (flags & MXC_F_RTC_CTRL_SSEC_ALARM_FL) {
        MXC_RTC_ClearFlags(MXC_F_RTC_CTRL_SSEC_ALARM_FL);
    }

    if (flags & MXC_F_RTC_CTRL_TOD_ALARM_FL) {
        MXC_RTC_ClearFlags(MXC_F_RTC_CTRL_TOD_ALARM_FL);
    }
}

// *****************************************************************************
void setTrigger(int waitForTrigger)
{
    alarmed = 0;
    while (MXC_RTC_Init(0, 0) == E_BUSY)
        ;
    while (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_TOD_ALARM_EN) == E_BUSY)
        ;
    while (MXC_RTC_SetTimeofdayAlarm(DELAY_IN_SEC) == E_BUSY)
        ;
    while (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_EN) == E_BUSY)
        ;
    while (MXC_RTC_Start() == E_BUSY)
        ;

    if (waitForTrigger) {
        while (!alarmed)
            ;
    }

// Wait for serial transactions to complete.
#if USE_CONSOLE
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR)
        ;
#endif // USE_CONSOLE
}
#endif // USE_ALARM

// *****************************************************************************
#if USE_BUTTON
void GPIOWAKE_IRQHandler(void)
{
    // Clear interrupt flag.
    MXC_PWRSEQ->gpio2_wk_fl = 0xFFFFFFFF;
}

// *****************************************************************************
void setTrigger(int waitForTrigger)
{
    int tmp;

    if (waitForTrigger) {
        while (PB_Get(0) == 0)
            ;

        // Debounce the button press.
        for (tmp = 0; tmp < 0x800000; tmp++) {
            __NOP();
        }
    }

    // Wait for button to be "up" (unpressed).
    while (PB_Get(0) == 1)
        ;

// Wait for serial transactions to complete.
#if USE_CONSOLE
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR)
        ;
#endif // USE_CONSOLE
}
#endif // USE_BUTTON

// *****************************************************************************
int main(void)
{
#if USE_CONSOLE
    printf("****Low Power Mode Example****\n\n");
#endif // USE_CONSOLE

#if USE_ALARM
#if USE_CONSOLE
    printf("This code cycles through the MAX32650 power modes, using the RTC alarm to exit from ");
    printf("each mode.  The modes will change every %d seconds.\n\n", DELAY_IN_SEC);
#endif // USE_CONSOLE
    MXC_NVIC_SetVector(RTC_IRQn, alarmHandler);
#endif // USE_ALARM

#if USE_BUTTON
#if USE_CONSOLE
    printf("This code cycles through the MAX32650 power modes, using a push button (SW2) to exit ");
    printf("from each mode and enter the next.\n\n");
#endif // USE_CONSOLE
    NVIC_EnableIRQ(GPIOWAKE_IRQn);
#endif // USE_BUTTON

#if USE_CONSOLE
    printf("Running in ACTIVE mode.\n");
#else
    SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
#endif // USE_CONSOLE
    setTrigger(1);

    MXC_LP_EnableROMLightSleep();
    MXC_LP_EnableUSBFIFOLightSleep();
    MXC_LP_EnableCryptoRAMLightSleep();
    MXC_LP_EnableSCacheLightSleep();
    MXC_LP_EnableICacheXIPLightSleep();
    MXC_ICC_Disable();
    MXC_LP_EnableICacheLightSleep();

    MXC_LP_DisableSysRAM0LightSleep(); // Global variables are in RAM0

#if USE_CONSOLE
    printf("All unused RAMs placed in LIGHT SLEEP mode.\n");
#endif // USE_CONSOLE
    setTrigger(1);

    MXC_LP_DisableROM();
    MXC_LP_DisableUSBFIFO();
    MXC_LP_DisableCryptoRAM();
    MXC_LP_DisableSCache();
    MXC_LP_DisableICacheXIP();
    MXC_LP_DisableICache();
    MXC_LP_DisableSRAM5();
    MXC_LP_DisableSRAM4();
    MXC_LP_DisableSRAM3();
    MXC_LP_DisableSRAM2();
    MXC_LP_DisableSRAM1();
    MXC_LP_DisableSRAM6();

    MXC_LP_EnableSRAM0(); // Global variables are in RAM0

#if USE_CONSOLE
    printf("All unused RAMs shutdown.\n");
#endif // USE_CONSOLE
    setTrigger(1);

#if USE_BUTTON
    MXC_LP_EnableGPIOWakeup((mxc_gpio_cfg_t*)&pb_pin[0]);
#endif // USE_BUTTON
#if USE_ALARM
    MXC_LP_EnableRTCAlarmWakeup();
#endif // USE_ALARM

    while (1) {
#if DO_SLEEP
#if USE_CONSOLE
        printf("Entering SLEEP mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterSleepMode();
#endif // DO_SLEEP

#if DO_BACKGROUND
#if USE_CONSOLE
        printf("Entering BACKGROUND mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterBackgroundMode();
#endif // DO_BACKGROUND

#if DO_DEEPSLEEP
#if USE_CONSOLE
        printf("Entering DEEPSLEEP mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterDeepSleepMode();
#endif // DO_DEEPSLEEP

#if DO_BACKUP
#if USE_CONSOLE
        printf("Entering BACKUP mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterBackupMode();
#endif // DO_BACKUP \
       //
    }
}
