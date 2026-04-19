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
#include "mxc_delay.h"

#define DELAY_IN_SEC 2
#define USE_CONSOLE 1

#define USE_BUTTON 1
#define USE_ALARM 0

#define DO_SLEEP 1
#define DO_BACKGROUND 0
#define DO_DEEPSLEEP 1
#define DO_BACKUP 0

#if (!(USE_BUTTON || USE_ALARM))
#error "You must set either USE_BUTTON or USE_ALARM to 1."
#endif
#if (USE_BUTTON && USE_ALARM)
#error "You must select either USE_BUTTON or USE_ALARM, not both."
#endif

#if USE_CONSOLE
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...)
#endif

// *****************************************************************************
#if USE_ALARM
volatile int alarmed;
void alarmHandler(void)
{
    int flags = MXC_RTC_GetFlags();
    alarmed = 1;

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
    while (MXC_RTC_Init(0, 0) == E_BUSY) {}
    while (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_TOD_ALARM_EN) == E_BUSY) {}
    while (MXC_RTC_SetTimeofdayAlarm(DELAY_IN_SEC) == E_BUSY) {}
    while (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_EN) == E_BUSY) {}
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

// *****************************************************************************
#if USE_BUTTON
volatile int buttonPressed;
void buttonHandler(void *pb)
{
    buttonPressed = 1;
}

// *****************************************************************************
void setTrigger(int waitForTrigger)
{
    int tmp;

    buttonPressed = 0;

    if (waitForTrigger) {
        while (!buttonPressed) {}
    }

    // Debounce the button press.
    for (tmp = 0; tmp < 0x800000; tmp++) {
        __NOP();
    }

// Wait for serial transactions to complete.
#if USE_CONSOLE
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
#endif // USE_CONSOLE
}
#endif // USE_BUTTON

// *****************************************************************************
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

#if USE_ALARM
    PRINT("This code cycles through the MAX32650 power modes, using the RTC alarm to exit from ");
    PRINT("each mode.  The modes will change every %d seconds.\n\n", DELAY_IN_SEC);
    MXC_NVIC_SetVector(RTC_IRQn, alarmHandler);
#endif // USE_ALARM

#if USE_BUTTON
    PRINT("This code cycles through the MAX32650 power modes, using a push button (SW2) to exit ");
    PRINT("from each mode and enter the next.\n\n");
    PB_RegisterCallback(0, buttonHandler);
#endif // USE_BUTTON

    PRINT("Running in ACTIVE mode.\n");
#if !USE_CONSOLE
    SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
#endif // ! USE_CONSOLE
    setTrigger(1);

    MXC_LP_EnableROMLightSleep();
    MXC_LP_EnableUSBFIFOLightSleep();
    MXC_LP_EnableCryptoRAMLightSleep();
    MXC_LP_EnableSCacheLightSleep();
    MXC_LP_EnableICacheXIPLightSleep();
    MXC_ICC_Disable();
    MXC_LP_EnableICacheLightSleep();
    MXC_LP_EnableSysRAM1LightSleep();
    MXC_LP_EnableSysRAM2LightSleep();
    MXC_LP_EnableSysRAM3LightSleep();
    MXC_LP_EnableSysRAM4LightSleep();
    MXC_LP_EnableSysRAM5LightSleep();
    MXC_LP_EnableSysRAM6LightSleep();

    MXC_LP_DisableSysRAM0LightSleep(); // Global variables are in RAM0

    PRINT("All unused RAMs placed in LIGHT SLEEP mode.\n");
    setTrigger(1);

    MXC_LP_DisableROM();
    MXC_LP_DisableUSBFIFO();
    MXC_LP_DisableCryptoRAM();
    MXC_LP_DisableSCache();
    MXC_LP_DisableICacheXIP();
    MXC_LP_DisableICache();
    MXC_LP_DisableSRAM1();
    MXC_LP_DisableSRAM2();
    MXC_LP_DisableSRAM3();
    MXC_LP_DisableSRAM4();
    MXC_LP_DisableSRAM5();
    MXC_LP_DisableSRAM6();

    MXC_LP_EnableSRAM0(); // Global variables are in RAM0

    // Resets the USB block to its initial state - where the operating current is at its minimum.
    MXC_LP_USBClearPONRST();

    PRINT("All unused RAMs shutdown.\n");
    setTrigger(1);

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
#endif // DO_SLEEP

#if DO_BACKGROUND
        PRINT("Entering BACKGROUND mode.\n");
        setTrigger(0);
        MXC_LP_EnterBackgroundMode();
#endif // DO_BACKGROUND

#if DO_DEEPSLEEP
        PRINT("Entering DEEPSLEEP mode.\n");
        setTrigger(0);
        MXC_LP_EnterDeepSleepMode();
#endif // DO_DEEPSLEEP

#if DO_BACKUP
        PRINT("Entering BACKUP mode.\n");
        setTrigger(0);
        MXC_LP_EnterBackupMode();
#endif // DO_BACKUP
    }
}
