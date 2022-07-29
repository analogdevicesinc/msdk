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

/**
 * @file    main.c
 * @brief   Timer example
 * @details PWM Timer        - Outputs a PWM signal (2Hz, 30% duty cycle) on 3.7
 *          Continuous Timer - Outputs a continuous 1s timer on LED0 (GPIO toggles every 500s)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "led.h"
#include "lp.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "pb.h"

/***** Definitions *****/
#ifdef BOARD_EVKIT_V1
#define BTN_PREFIX "PB"
#else
#define BTN_PREFIX "SW"
#endif

#define SLEEP_MODE // Select between SLEEP_MODE and DEEPSLEEP_MODE for LPTIMER
//#define DEEPSLEEP_MODE

#define PB2 1

// Parameters for PWM output
#define OST_CLOCK_SOURCE  MXC_TMR_8K_CLK  // \ref mxc_tmr_clock_t
#define PWM_CLOCK_SOURCE  MXC_TMR_32K_CLK // \ref mxc_tmr_clock_t
#define CONT_CLOCK_SOURCE MXC_TMR_8M_CLK  // \ref mxc_tmr_clock_t

// Parameters for Continuous timer
#define OST_FREQ  1        // (Hz)
#define OST_TIMER MXC_TMR5 // Can be MXC_TMR0 through MXC_TMR5

#define FREQ       10       // (Hz)
#define DUTY_CYCLE 50       // (%)
#define PWM_TIMER  MXC_TMR4 // must change PWM_PORT and PWM_PIN if changed

// Parameters for Continuous timer
#define CONT_FREQ  2        // (Hz)
#define CONT_TIMER MXC_TMR1 // Can be MXC_TMR0 through MXC_TMR5

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
void PWMTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr; // to configure timer
    unsigned int periodTicks = MXC_TMR_GetPeriod(PWM_TIMER, PWM_CLOCK_SOURCE, 16, FREQ);
    unsigned int dutyTicks   = periodTicks * DUTY_CYCLE / 100;

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the pre-scale value
    3. Set polarity, PWM parameters
    4. Configure the timer for PWM mode
    5. Enable Timer
    */

    MXC_TMR_Shutdown(PWM_TIMER);

    tmr.pres    = TMR_PRES_16;
    tmr.mode    = TMR_MODE_PWM;
    tmr.bitMode = TMR_BIT_MODE_32;
    tmr.clock   = PWM_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol     = 1;

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
void ContinuousTimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(CONT_TIMER);
    LED_Toggle(LED1);
}

void ContinuousTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, CONT_CLOCK_SOURCE, 128, CONT_FREQ);

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres    = TMR_PRES_128;
    tmr.mode    = TMR_MODE_CONTINUOUS;
    tmr.bitMode = TMR_BIT_MODE_16B;
    tmr.clock   = CONT_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol     = 0;

    if (MXC_TMR_Init(CONT_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    printf("Continuous timer started.\n\n");
}

void OneshotTimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(OST_TIMER);

    // Clear interrupt
    if (MXC_TMR5->wkfl & MXC_F_TMR_WKFL_A) {
        MXC_TMR5->wkfl = MXC_F_TMR_WKFL_A;
#ifdef BOARD_EVKIT_V1
        LED_Toggle(LED2);
#endif
        printf("Oneshot timer expired!\n");
    }
}

void OneshotTimer()
{
    for (int i = 0; i < 5000; i++)
        ; //Button debounce

    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(OST_TIMER, OST_CLOCK_SOURCE, 1, OST_FREQ);
    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(OST_TIMER);

    tmr.pres    = TMR_PRES_1;
    tmr.mode    = TMR_MODE_ONESHOT;
    tmr.bitMode = TMR_BIT_MODE_32;
    tmr.clock   = OST_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol     = 0;

    if (MXC_TMR_Init(OST_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed one-shot timer Initialization.\n");
        return;
    }

    MXC_TMR_EnableInt(OST_TIMER);

    // Enable wkup source in Poower seq register
    MXC_LP_EnableTimerWakeup(OST_TIMER);
    // Enable Timer wake-up source
    MXC_TMR_EnableWakeup(OST_TIMER, &tmr);

    printf("Oneshot timer started.\n\n");

    MXC_TMR_Start(OST_TIMER);
}

void PB1Handler()
{
    PWMTimer();

    MXC_NVIC_SetVector(TMR1_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(TMR1_IRQn);
    ContinuousTimer();
}

// *****************************************************************************
int main(void)
{
    //Exact timer operations can be found in tmr_utils.c

    printf("\n************************** Timer Example **************************\n\n");
    printf("1. A oneshot mode timer, Timer 5 (low-power timer) is used to create an\n");
    printf("   interrupt at a freq of %d Hz. If running the example on the MAX78000EVKIT,\n",
           OST_FREQ);
    printf("   LED2 will toggle when the interrupt occurs.\n\n");
    printf("2. Timer 4 is used to output a PWM signal on P2.4 (labeled \"AIN1\" on\n");
    printf("   MAX78000FTHR). The PWM frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ,
           DUTY_CYCLE);
    printf("3. Timer 1 is configured as a 16-bit timer used in continuous mode.\n");
    printf("   It creates an interrupt at freq of %d Hz. LED1 will toggle when\n", CONT_FREQ);
    printf("   the interrupt occurs.\n\n");
    printf("Push %s1 to start the PWM and continuous timers and %s2 to start the\n", BTN_PREFIX,
           BTN_PREFIX);
    printf("oneshot timer.\n\n");

    PB_RegisterCallback(0, (pb_callback)PB1Handler);

    while (1) {
        if (PB_Get(1) == 1) {
            MXC_NVIC_SetVector(TMR5_IRQn, OneshotTimerHandler);
            NVIC_EnableIRQ(TMR5_IRQn);

            OneshotTimer();

#ifdef SLEEP_MODE
            MXC_LP_EnterSleepMode();

#else
            MXC_LP_EnterDeepSleepMode();
#endif
        }
    }

    return 0;
}
