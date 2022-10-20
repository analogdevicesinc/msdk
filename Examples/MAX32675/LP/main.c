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

/*
 * @file    main.c
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

#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "pb.h"
#include "led.h"
#include "lp.h"
#include "icc.h"
#include "rtc.h"
#include "uart.h"
#include "nvic_table.h"

#define DELAY_IN_SEC 2
#define USE_CONSOLE 1

#define DO_SLEEP 1
#define DO_DEEPSLEEP 1
#define DO_BACKUP 0
#define DO_SHUTDOWN 0

#if (DO_BACKUP && DO_STORAGE)
#error "You must select either DO_BACKUP or DO_STORAGE or neither, not both."
#endif

// *****************************************************************************
volatile int buttonPressed;
void buttonHandler(void *pb)
{
    buttonPressed = 1;
}

void setTrigger(int waitForTrigger)
{
    int tmp;

    buttonPressed = 0;

    if (waitForTrigger) {
        while (!buttonPressed) {}
    }

    // Debounce the button press.
    for (tmp = 0; tmp < 0x80000; tmp++) {
        __NOP();
    }

    // Wait for serial transactions to complete.
#if USE_CONSOLE

    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}

#endif // USE_CONSOLE
}

void configure_gpio(void)
{
    //Set GPIOs to output mode except PB0 and UART0 pins
    MXC_GPIO0->en0 |= 0xFFFFFCFFUL;
    MXC_GPIO0->outen |= 0xFFFFFCFFUL;

    MXC_GPIO1->en0 |= 0xFFFFF7FFUL;
    MXC_GPIO1->outen |= 0xFFFFF7FFUL;

    // Pull down all the GPIO pins except PB0 and UART0
    MXC_GPIO0->padctrl0 &= ~0xFFFFFFFFUL;
    MXC_GPIO0->padctrl1 |= 0xFFFFFFFFUL;
    MXC_GPIO0->ps &= ~0xFFFFFFFFUL;
    MXC_GPIO1->padctrl0 &= ~0xFFFFF7FFUL;
    MXC_GPIO1->padctrl1 |= 0xFFFFF7FFUL;
    MXC_GPIO1->ps &= ~0xFFFFF7FFUL;

    //Set output low
    // MXC_GPIO0->out      &= ~0xFFDFFCFFUL;
    // MXC_GPIO1->out      &= ~0xFFFFFFFFUL;
}

int main(void)
{
#if USE_CONSOLE
    printf("****Low Power Mode Example****\n\n");
#endif // USE_CONSOLE

#if USE_CONSOLE
    printf("This code cycles through the MAX32675 power modes, using a push button 0 (SW1) to exit "
           "from each mode and enter the next.\n\n");
#endif // USE_CONSOLE
    PB_RegisterCallback(0, buttonHandler);

    //Pull down all GPIOs except PB0 and UART0 pins
    configure_gpio();

#if USE_CONSOLE
    printf("Running in ACTIVE mode.\n");
#else
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
#endif // USE_CONSOLE
    setTrigger(1);

    MXC_LP_ROMLightSleepEnable();

    // MXC_LP_SysRam3LightSleepDisable();
    MXC_LP_SysRam2LightSleepEnable();
    MXC_LP_SysRam1LightSleepDisable();
    MXC_LP_SysRam0LightSleepDisable(); // Global variables are in RAM0 and RAM1

#if USE_CONSOLE
    printf("All unused RAMs placed in LIGHT SLEEP mode.\n");
#endif // USE_CONSOLE
    setTrigger(1);

    // MXC_LP_SysRam3Shutdown();
    MXC_LP_SysRam2Shutdown();

    MXC_LP_SysRam1PowerUp();
    MXC_LP_SysRam0PowerUp(); // Global variables are in RAM0 and RAM1

#if USE_CONSOLE
    printf("All unused RAMs shutdown.\n");
#endif // USE_CONSOLE
    setTrigger(1);

    MXC_LP_EnableGPIOWakeup((mxc_gpio_cfg_t *)&pb_pin[0]);

    while (1) {
#if DO_SLEEP
#if USE_CONSOLE
        printf("Entering SLEEP mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterSleepMode();
        printf("Waking up from SLEEP mode.\n");

#endif // DO_SLEEP
#if DO_DEEPSLEEP
#if USE_CONSOLE
        printf("Entering DEEPSLEEP mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterDeepSleepMode();
        printf("Waking up from DEEPSLEEP mode.\n");
#endif // DO_DEEPSLEEP

#if DO_BACKUP
#if USE_CONSOLE
        printf("Entering BACKUP mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterBackupMode();
#endif // DO_BACKUP

#if DO_SHUTDOWN
#if USE_CONSOLE
        printf("Entering Shutdown mode.\n");
#endif // USE_CONSOLE
        setTrigger(0);
        MXC_LP_EnterShutDownMode();
#endif // DO_STORAGE
    }
}
