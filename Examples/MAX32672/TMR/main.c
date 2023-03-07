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

/**
 * @file    main.c
 * @brief   Timer example
 * @details PWM Timer        - Outputs a PWM signal (1KHz, 50% duty cycle) on P0.5
 *          Continuous Timer - Outputs a continuous 1s timer on LED0 (GPIO toggles every 250ms)
 *          One shoot Timer  - Outputs: Toggle LED1 after  1s
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

/***** Definitions *****/

// Parameters for PWM output
#define CLOCK_SOURCE MXC_TMR_8M_CLK // must be mxc_tmr_clock_t

// Parameters for Continuous timer
#define OST_FREQ 1 // (Hz)
#define OST_TIMER MXC_TMR1 // Can be MXC_TMR0 through MXC_TMR5

#define FREQ 1000 // (Hz)
#define DUTY_CYCLE 75 // (%)
#define PWM_TIMER MXC_TMR2 // must change PWM_PORT and PWM_PIN if changed

// Parameters for Continuous timer
#define CONT_FREQ 4 // (Hz)
#define CONT_TIMER MXC_TMR3 // Can be MXC_TMR0 through MXC_TMR5

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

/***** Globals *****/

/***** Functions *****/

void PWMTimer()
{
    mxc_tmr_cfg_t tmr;
    unsigned int periodTicks;
    unsigned int dutyTicks;

    /*
    Steps for configuring a timer for PWM Mode:
    1. Disable the timer
    2. Set the pre-scale value
    3. Set polarity, PWM parameters
    4. Configure the timer for PWM mode
    5. Enable Timer
    */

    MXC_TMR_Shutdown(PWM_TIMER);

    // Calculate number of ticks to achieve desired PWM frequency and duty cycle
    periodTicks = MXC_TMR_GetPeriod(PWM_TIMER, CLOCK_SOURCE, 16, FREQ);
    dutyTicks = periodTicks * DUTY_CYCLE / 100;

    // Configure PWM timer
    tmr.pres = TMR_PRES_16;
    tmr.mode = TMR_MODE_PWM;
    tmr.clock = CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 1;
    MXC_TMR_Init(PWM_TIMER, &tmr, true);

    // Set duty cycle
    if (MXC_TMR_SetPWM(PWM_TIMER, dutyTicks) != E_NO_ERROR) {
        printf("Failed TMR_PWMConfig.\n");
    }

    // Start PWM timer
    MXC_TMR_Start(PWM_TIMER);

    printf("PWM started.\n\n");
}

// Toggles LED0 when continuous timer repeats
void ContinuousTimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(CONT_TIMER);
    LED_Toggle(0);
}

void ContinuousTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks;

    /*
    Steps for configuring a timer for Continuous Mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for Continuous Mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    // Calculate number of ticks in continuous timer period
    periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, CLOCK_SOURCE, 128, CONT_FREQ);

    // Configure continuous timer
    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.clock = CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    MXC_TMR_Init(CONT_TIMER, &tmr, true);
    MXC_TMR_EnableInt(CONT_TIMER);

    // Start continuous timer
    MXC_TMR_Start(CONT_TIMER);

    printf("Continuous timer started.\n\n");
}

// Turns on LED0 when Oneshot expires
void OneshotTimerHandler()
{
    MXC_TMR_ClearFlags(OST_TIMER);
    LED_Toggle(1);
}

void OneshotTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks;

    /*
    Steps for configuring a timer for Oneshot Mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for Oneshot Mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(OST_TIMER);

    // Calculate number of ticks in the oneshot timer period
    periodTicks = MXC_TMR_GetPeriod(OST_TIMER, CLOCK_SOURCE, 128, OST_FREQ);

    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_ONESHOT;
    tmr.bitMode = TMR_BIT_MODE_16B;
    tmr.clock = CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    MXC_TMR_Init(OST_TIMER, &tmr, true);

    printf("Oneshot timer started.\n\n");
}

// *****************************************************************************
int main(void)
{
    printf("\n************************** Timer Example **************************\n\n");
    printf("1. Timer 3 is used in continuous mode to generate an interrupt at\n");
    printf("   a frequency of %d Hz. LED0 will toggle each time the interrupt\n", CONT_FREQ);
    printf("   is triggered.\n\n");
    printf("2. Timer 2 is used to output a PWM signal on Port 0.5. The PWM\n");
    printf("   frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ, DUTY_CYCLE);
    printf("3. Timer 1 is configured as a 16-bit timer in oneshot mode that\n");
    printf("   generates an interrupt at a frequency of %d Hz. LED1 will toggle\n", OST_FREQ);
    printf("   when the oneshot interrupt is triggered.\n\n");

    // Start PWM Output
    PWMTimer();

    // Start Continuous Timer
    MXC_NVIC_SetVector(TMR3_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(TMR3_IRQn);
    ContinuousTimer();

    // Start Oneshot Timer
    MXC_NVIC_SetVector(TMR1_IRQn, OneshotTimerHandler);
    NVIC_EnableIRQ(TMR1_IRQn);
    OneshotTimer();

    while (1) {}

    return 0;
}
