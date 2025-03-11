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
#include "board.h"
#include "gpio.h"
#include "icc.h"
#include "lp.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "pb.h"
#include "rtc.h"
#include "uart.h"

#define DELAY_IN_SEC 2
#define USE_CONSOLE 1

#define USE_BUTTON 1
#define USE_ALARM 0

#define DO_SLEEP 1
#define DO_DEEPSLEEP 1
#define DO_BACKUP 0
#define DO_STORAGE 0

#if (!(USE_BUTTON || USE_ALARM))
#error "You must set either USE_BUTTON or USE_ALARM to 1."
#endif
#if (USE_BUTTON && USE_ALARM)
#error "You must select either USE_BUTTON or USE_ALARM, not both."
#endif

#if (DO_BACKUP && DO_STORAGE)
#error "You must select either DO_BACKUP or DO_STORAGE or neither, not both."
#endif

#if USE_CONSOLE
#define PRINT(...) fprintf(stdout, __VA_ARGS__)
#else
#define PRINT(...)
#endif

// *****************************************************************************

#if USE_ALARM
volatile int alarmed;
void alarmHandler(void)
{
    int flags = MXC_RTC->ctrl;
    alarmed = 1;

    if ((flags & MXC_F_RTC_CTRL_SSEC_ALARM) >> MXC_F_RTC_CTRL_SSEC_ALARM_POS) {
        MXC_RTC->ctrl &= ~(MXC_F_RTC_CTRL_SSEC_ALARM);
    }

    if ((flags & MXC_F_RTC_CTRL_TOD_ALARM) >> MXC_F_RTC_CTRL_TOD_ALARM_POS) {
        MXC_RTC->ctrl &= ~(MXC_F_RTC_CTRL_TOD_ALARM);
    }
}

void setTrigger(int waitForTrigger)
{
    alarmed = 0;

    while (MXC_RTC_Init(0, 0) == E_BUSY) {}

    while (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY) {}

    while (MXC_RTC_SetTimeofdayAlarm(DELAY_IN_SEC) == E_BUSY) {}

    while (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY) {}

    while (MXC_RTC_Start() == E_BUSY) {}

    if (waitForTrigger) {
        while (!alarmed) {}
    }

    // Wait for serial transactions to complete.
#if USE_CONSOLE
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
#endif // USE_CONSOLE
}
#endif // USE_ALARM

#if USE_BUTTON
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
#endif // USE_BUTTON

void configure_gpios(void)
{
    mxc_gpio_cfg_t out_clr;
    mxc_gpio_cfg_t out_set;

    // Create list of pins that are in use (the "do not modify" list)
    uint32_t dnm = MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1 | MXC_GPIO_PIN_10;

    // Add UART TX pin (P0.9) to do not modify list if the console is being used
#if USE_CONSOLE
    dnm |= MXC_GPIO_PIN_9;
#endif // USE_CONSOLE

    // Add Push Button pin (P0.18) to the do not modify list if it's being used as the wakeup source
#if USE_BUTTON
    dnm |= MXC_GPIO_PIN_18;
#endif // USE_BUTTON

    // Set all GPIO pins low except for SWD (P0.0/P0.1), and P0.10 (and push button and UART pins if they're being used)
    out_clr.port = MXC_GPIO0;
    out_clr.mask = ~dnm;
    out_clr.func = MXC_GPIO_FUNC_OUT;
    out_clr.pad = MXC_GPIO_PAD_NONE;
    out_clr.vssel = MXC_GPIO_VSSEL_VDDIOH;
    out_clr.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&out_clr);
    MXC_GPIO_OutClr(out_clr.port, out_clr.mask);

    // Set GPIO P0.10 high (it's connected to an external pullup resistor)
    out_set.port = MXC_GPIO0;
    out_set.mask = MXC_GPIO_PIN_10;
    out_set.func = MXC_GPIO_FUNC_OUT;
    out_set.pad = MXC_GPIO_PAD_NONE;
    out_set.vssel = MXC_GPIO_VSSEL_VDDIOH;
    out_set.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&out_set);
    MXC_GPIO_OutSet(out_set.port, out_set.mask);
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

    // Delay to provide the debugger with a window to connect.
    // Low-power modes shut down SWD
    MXC_Delay(MXC_DELAY_SEC(2));

    PRINT("\n************ Low Power Mode Example ************\n\n");

#if USE_ALARM
    PRINT("This code cycles through the MAX32672 power modes, using the RTC alarm to exit from "
          "each mode.  The modes will change every %d seconds.\n\n",
          DELAY_IN_SEC);
    MXC_NVIC_SetVector(RTC_IRQn, alarmHandler);
#endif // USE_ALARM

#if USE_BUTTON
    PRINT("This code cycles through the MAX32672 power modes, using a push button (SW3) to exit "
          "from each mode and enter the next.\n\n");
    PB_RegisterCallback(0, buttonHandler);
#endif // USE_BUTTON

    // Set GPIO pins to known state
    configure_gpios();

    PRINT("Running in ACTIVE mode.\n");
#if !USE_CONSOLE
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
#endif // USE_CONSOLE
    setTrigger(1);

    // Put unused RAMs in light sleep
    MXC_LP_ROMLightSleepEnable();
    MXC_LP_ICache0LightSleepEnable();
    MXC_LP_SysRam3LightSleepEnable();
    MXC_LP_SysRam2LightSleepEnable();
    MXC_LP_SysRam1LightSleepDisable(); // Global variables are in RAM0 and RAM1
    MXC_LP_SysRam0LightSleepDisable();

    PRINT("All unused RAMs placed in LIGHT SLEEP mode.\n");
    setTrigger(1);

    // Shutdown unused RAMs
    MXC_LP_SysRam3Shutdown();
    MXC_LP_SysRam2Shutdown();
    MXC_LP_SysRam1PowerUp(); // Global variables are in RAM0 and RAM1
    MXC_LP_SysRam0PowerUp();

    PRINT("All unused RAMs shutdown.\n");
    setTrigger(1);

    // Enable Wakeup source
#if USE_BUTTON
    MXC_LP_EnableGPIOWakeup((mxc_gpio_cfg_t *)&pb_pin[0]);
    MXC_GPIO_SetWakeEn(pb_pin[0].port, pb_pin[0].mask);
#endif // USE_BUTTON
#if USE_ALARM
    MXC_LP_EnableRTCAlarmWakeup();
#endif // USE_ALARM

    while (1) {
#if DO_SLEEP
        PRINT("Entering SLEEP mode.\n");
        setTrigger(0);
        MXC_LP_EnterSleepMode();
        PRINT("Waking up from SLEEP mode.\n");
#endif // DO_SLEEP

#if DO_DEEPSLEEP
        PRINT("Entering DEEPSLEEP mode.\n");
        setTrigger(0);
        MXC_LP_EnterDeepSleepMode();
        PRINT("Waking up from DEEPSLEEP mode.\n");
#endif // DO_DEEPSLEEP

#if DO_BACKUP
        PRINT("Entering BACKUP mode.\n");
        setTrigger(0);
        MXC_LP_EnterBackupMode();
#endif // DO_BACKUP

#if DO_STORAGE
        PRINT("Entering STORAGE mode.\n");
        setTrigger(0);
        MXC_LP_EnterStorageMode();
#endif // DO_STORAGE
    }

    return 0;
}
