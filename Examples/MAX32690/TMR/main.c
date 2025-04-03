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

/**
 * @file    main.c
 * @brief   Timer example
 * @details PWM Timer        - Outputs a PWM signal (2Hz, 30% duty cycle) on TMR2
 *          Continuous Timer - Outputs a continuous 1s timer on LED0 (GPIO toggles every 500s)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "gpio.h"
#include "board.h"
#include "tmr.h"
#include "led.h"
#include "pb.h"
#include "lp.h"
#include "lpgcr_regs.h"
#include "gcr_regs.h"
#include "pwrseq_regs.h"

#include "mxc_delay.h"

/***** Definitions *****/
#define SLEEP_MODE // Select between SLEEP_MODE and LP_MODE for LPTIMER

// Parameters for PWM output
#define OST_CLOCK_SOURCE MXC_TMR_IBRO_CLK // \ref mxc_tmr_clock_t
#define PWM_CLOCK_SOURCE MXC_TMR_ISO_CLK // \ref mxc_tmr_clock_t
#define CONT_CLOCK_SOURCE MXC_TMR_APB_CLK // \ref mxc_tmr_clock_t

// Parameters for Continuous timer
#define OST_FREQ 1 // (Hz)
#define OST_TIMER MXC_TMR4 // Can be MXC_TMR0 through MXC_TMR5

#define FREQ 1000 // (Hz)
#define DUTY_CYCLE 20 // (%)
#define PWM_TIMER MXC_TMR1 // must change PWM_PORT and PWM_PIN if changed

// Parameters for Continuous timer
#define CONT_FREQ 2 // (Hz)
#define CONT_TIMER MXC_TMR0 // Can be MXC_TMR0 through MXC_TMR5

// Check Frequency bounds
#if (FREQ == 0)
#error "Frequency cannot be 0."
#elif (FREQ > 100000)
#error "Frequency cannot be over 100000."
#endif

// Check duty cycle bounds
#if (DUTY_CYCLE < 0) || (DUTY_CYCLE > 100)
#error "Duty Cycle must be between 0 and 100."
#endif

/***** Functions *****/
void PWMTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr; // to configure timer
    unsigned int periodTicks = MXC_TMR_GetPeriod(PWM_TIMER, PWM_CLOCK_SOURCE, 16, FREQ);
    unsigned int dutyTicks = periodTicks * DUTY_CYCLE / 100;

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the pre-scale value
    3. Set polarity, PWM parameters
    4. Configure the timer for PWM mode
    5. Enable Timer
    */

    MXC_TMR_Shutdown(PWM_TIMER);

    tmr.pres = TMR_PRES_16;
    tmr.mode = TMR_MODE_PWM;
    tmr.bitMode = TMR_BIT_MODE_16A;
    tmr.clock = PWM_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 1;

    if (MXC_TMR_Init(PWM_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed PWM timer Initialization.\n");
        return;
    }

    if (MXC_TMR_SetPWM(PWM_TIMER, dutyTicks) != E_NO_ERROR) {
        printf("Failed TMR_PWMConfig.\n");
        return;
    }

    MXC_TMR_Start(PWM_TIMER);

    printf("PWM started.\n\n");
}

// Toggles GPIO when continuous timer repeats
void ContinuousTimerHandler(void)
{
    // Clear interrupt
    MXC_TMR_ClearFlags(CONT_TIMER);
    LED_Toggle(0);
}

void ContinuousTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, CONT_CLOCK_SOURCE, 32, CONT_FREQ);

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres = TMR_PRES_32;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.bitMode = TMR_BIT_MODE_32;
    tmr.clock = CONT_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    if (MXC_TMR_Init(CONT_TIMER, &tmr, 0) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    MXC_TMR_EnableInt(CONT_TIMER);

    MXC_NVIC_SetVector(TMR0_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(TMR0_IRQn);

    MXC_TMR_Start(CONT_TIMER);

    printf("Continuous timer started.\n\n");
}

void OneshotTimerHandler(void)
{
    // Clear interrupt
    MXC_TMR_ClearFlags(OST_TIMER);

    // Clear interrupt
    if (MXC_TMR4->wkfl & MXC_F_TMR_WKFL_A) {
        MXC_TMR4->wkfl = MXC_F_TMR_WKFL_A;
        LED_Toggle(1);
    }
}

void OneshotTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(OST_TIMER, OST_CLOCK_SOURCE, 128, OST_FREQ);
    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(OST_TIMER);

    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_ONESHOT;
    tmr.bitMode = TMR_BIT_MODE_32;
    tmr.clock = OST_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    if (MXC_TMR_Init(OST_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    MXC_TMR_EnableInt(OST_TIMER);

    // Clear Wakeup status
    MXC_LP_ClearWakeStatus();
    // Enable wkup source in Poower seq register
    MXC_LP_EnableTimerWakeup(OST_TIMER);
    // Enable Timer wake-up source
    MXC_TMR_EnableWakeup(OST_TIMER, &tmr);

    printf("Oneshot timer started.\n\n");

    MXC_TMR_Start(OST_TIMER);
}

void SWHandler(void)
{
    // Start PWM timer
    PWMTimer();

    // Start continuous timer
    ContinuousTimer();
}

// *****************************************************************************
int main(void)
{
    //Exact timer operations can be found in tmr_utils.c

    printf("\n**************************Timer Example **************************\n\n");
    printf("1. A oneshot mode timer, Timer 4 (lptimer) is used to create\n");
    printf("   an interrupt at a freq of %d Hz. LED 2 (Port 2.12) will toggle\n", OST_FREQ);
    printf("   when the interrupt occurs.\n\n");
    printf("2. Timer 0 is used to output a PWM signal on Port 0.7.\n");
    printf("   The PWM frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ, DUTY_CYCLE);
    printf("3. Timer 1 is configured as a 32-bit timer used in continuous\n");
    printf("   mode which is used to create an interrupt at a freq of %d Hz.\n", CONT_FREQ);
    printf("   LED 1 (Port 0.14) will toggle each time the oneshot timer is\n   finished "
           "running.\n\n");
#if defined(EvKit_V1)
    printf("Push SW2 to start the PWM and continuous timers initially. Then\nuse SW2 to run the "
           "lptimer in oneshot mode thereafter.\n\n");
#else
    printf("Push SW3 to start the PWM and continuous timers initially. Then\nuse SW3 to run the "
           "lptimer in oneshot mode thereafter.\n\n");
#endif

    // Wait until button press to start PWM and continuous timers
    while (!PB_Get(0)) {}
    SWHandler();
    MXC_Delay(MXC_DELAY_SEC(1));

    while (1) {
        // If button pressed, run oneshot timer
        if (PB_Get(0)) {
            MXC_NVIC_SetVector(TMR4_IRQn, OneshotTimerHandler);
            NVIC_EnableIRQ(TMR4_IRQn);

            OneshotTimer();

#ifdef SLEEP_MODE
            MXC_LP_EnterSleepMode();
#else
            MXC_LP_EnterLowPowerMode();
#endif
        }
    }

// Unreachable statement that results in warning for IAREW Arm compiler
#ifndef __ICCARM__
    return 0;
#endif
}
