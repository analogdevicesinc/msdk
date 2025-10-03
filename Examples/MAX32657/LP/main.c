/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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
 *            3. Active mode power with unused RAMs shut down
 *            4. SLEEP mode
 *            5. LPM mode
 *            6. UPM mode
 *            7. BACKUP mode
 *            8. STANDBY mode
 */

#include <stdio.h>
#include <stdint.h>
#include "mxc.h"

#define DELAY_IN_SEC 2
#define USE_CONSOLE 1

#define USE_BUTTON 1
#define USE_ALARM 0

#define DO_SLEEP 1
#define DO_BACKUP 0
#define DO_STANDBY 0
#define DO_POWERDOWN 0 // will reset after wakeup

#if (!(USE_BUTTON || USE_ALARM))
#error "You must set either USE_BUTTON or USE_ALARM to 1."
#endif
#if (USE_BUTTON && USE_ALARM)
#error "You must select either USE_BUTTON or USE_ALARM, not both."
#endif

#if (DO_BACKUP && DO_STANDBY)
#error "You must select either DO_BACKUP or DO_STANDBY or neither, not both."
#endif

#if (DO_BACKUP && DO_POWERDOWN)
#error "You must select either DO_BACKUP or DO_POWERDOWN or neither, not both."
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
    volatile int tmp;

    buttonPressed = 0;

    if (waitForTrigger) {
        while (!buttonPressed) {}
    }

    // Debounce the button press.
    for (tmp = 0; tmp < 0x100000; tmp++) {
        __NOP();
    }

    // Wait for serial transactions to complete.
#if USE_CONSOLE

    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}

#endif // USE_CONSOLE
}
#endif // USE_BUTTON

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
    PRINT("This code cycles through the MAX32657 power modes, using the RTC alarm to exit from "
          "each mode.  The modes will change every %d seconds.\n\n",
          DELAY_IN_SEC);
    MXC_NVIC_SetVector(RTC_IRQn, alarmHandler);
#endif // USE_ALARM

#if USE_BUTTON
    PRINT("This code cycles through the MAX32657 power modes, using a push button (PB0) to exit "
          "from each mode and enter the next.\n\n");
    PB_RegisterCallback(0, buttonHandler);
#endif // USE_BUTTON

    PRINT("Running in ACTIVE mode.\n");
#if !USE_CONSOLE
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
#endif // !USE_CONSOLE
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
        PRINT("Waking up from SLEEP mode.\n");
#endif // DO_SLEEP

#if DO_BACKUP
        PRINT("Entering BACKUP mode.\n");
        setTrigger(0);
        MXC_LP_EnterBackupMode();
#endif // DO_BACKUP

#if DO_STANDBY
        PRINT("Entering STANDBY mode.\n");
        setTrigger(0);
        MXC_LP_EnterStandbyMode();
        PRINT("Waking up from STANDBY mode.\n");
#endif // DO_STANDBY

#if DO_POWERDOWN
        PRINT("Entering Power Down mode, press reset or P3.0 = 0 to restart.\n");
        setTrigger(0);

        MXC_LP_EnterPowerDownMode();
#endif // DO_POWERDOWN
    }
}
