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

/*
 * @file    main.c
 * @brief   Demonstrates the various low power modes.
 *
 * @details Iterates through the various low power modes, using
 * 			GPIO to wake from each.  #defines determine which wakeup
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
#include "led.h"
#include "lp.h"
#include "icc.h"
#include "uart.h"
#include "nvic_table.h"
#include "mxc_delay.h"

#ifndef BOARD_MAX32520FTHR
#include "pb.h"
#endif //BOARD_MAX32520FTHR

#define USE_CONSOLE 1

#define DO_SLEEP 1
#define DO_DEEPSLEEP 1
#define DO_BACKUP 0

#if USE_CONSOLE
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...)
#endif

// *****************************************************************************
volatile int buttonPressed;
void buttonHandler(void *pb)
{
    buttonPressed = 1;
}

void setTrigger(int waitForTrigger)
{
#ifndef BOARD_MAX32520FTHR
    int tmp;
    buttonPressed = 0;

    if (waitForTrigger) {
        while (!buttonPressed) {}
    }

    // Debounce the button press.
    for (tmp = 0; tmp < 0x800000; tmp++) {
        __NOP();
    }
#else
    MXC_Delay(MXC_DELAY_SEC(2));
#endif

    // Wait for serial transactions to complete.
#if USE_CONSOLE
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
#endif // USE_CONSOLE
}

int main(void)
{
    /** PLEASE READ **/
    // MCU shuts down SWD interface in LP modes
    // Delay at application start to prevent debugger losing access on restarts
    //
    // Only remove this if you are okay with losing debug access
    // If you can't flash/debug the device, please consult this guide:
    // https://analogdevicesinc.github.io/msdk//USERGUIDE/#how-to-unlock-a-microcontroller-that-can-no-longer-be-programmed
    MXC_Delay(MXC_DELAY_SEC(2));

    PRINT("****Low Power Mode Example****\n\n");

    PRINT("This code cycles through the MAX32520 power modes, "
#ifndef BOARD_MAX32520FTHR
          "using a push button (SW2) to exit from each mode and enter the next.\n\n");
    PB_RegisterCallback(0, buttonHandler);
    MXC_LP_EnableGPIOWakeup((mxc_gpio_cfg_t *)&pb_pin[0]);
    MXC_GPIO_SetWakeEn(pb_pin[0].port, pb_pin[0].mask);
#else
          "with a 2 second delay before entering the next mode.\n\n");
#endif

    PRINT("Running in ACTIVE mode.\n");
#if !USE_CONSOLE
    SYS_ClockDisable(SYS_PERIPH_CLOCK_UART0);
#endif // USE_CONSOLE
    setTrigger(1);

    MXC_LP_ROMLightSleepEnable();
    MXC_LP_ICache0LightSleepEnable();
    MXC_LP_SysRam4LightSleepEnable();
    MXC_LP_SysRam3LightSleepEnable();
    MXC_LP_SysRam2LightSleepEnable();

    MXC_LP_SysRam1LightSleepDisable();
    MXC_LP_SysRam0LightSleepDisable(); // Global variables are in RAM0 and RAM1

    PRINT("All unused RAMs placed in LIGHT SLEEP mode.\n");
    setTrigger(1);

    MXC_LP_ROMShutdown();
    MXC_LP_ICache0Shutdown();

    PRINT("All unused RAMs shutdown.\n");
    setTrigger(1);

    while (1) {
#if DO_SLEEP
        PRINT("Entering SLEEP mode.\n");
        setTrigger(0);
        MXC_LP_EnterSleepMode();
        PRINT("Wakeup from SLEEP mode.\n\n");
#endif // DO_SLEEP
#if DO_DEEPSLEEP
        PRINT("Entering DEEPSLEEP mode.\n");
        setTrigger(0);
        MXC_LP_EnterDeepSleepMode();
        PRINT("Wakeup from DEEPSLEEP mode.\n\n");
#endif // DO_DEEPSLEEP

#if DO_BACKUP
        PRINT("Entering BACKUP mode.\n");
        setTrigger(0);
        MXC_LP_EnterBackupMode();
#endif // DO_BACKUP
    }

    return 0;
}
