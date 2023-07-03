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
 * @details PWM Timer        - Outputs a PWM signal (2Hz, 30% duty cycle) on 3.7
 *          Continuous Timer - Outputs a continuous 1s timer on LED0 (GPIO toggles every 500s)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "gpio.h"
#include "board.h"
#include "tmr.h"
#include "led.h"

/***** Definitions *****/
#define MAX_TICKS pow(2, 32)

// Parameters for PWM output
#define FREQ 1000 // (Hz)
#define DUTY_CYCLE 50 // (%)
#define PWM_TIMER MXC_TMR0
#define PWM_PORT MXC_GPIO0
#define PWM_PIN MXC_GPIO_PIN_3

// Parameters for One Shot timer
#define OST_FREQ 1 //(Hz)
#define OST_TIMER MXC_TMR1

// Parameters for Continuous timer
#define CONT_FREQ 2 // (Hz)
#define CONT_TIMER MXC_TMR2 // Can be MXC_TMR0 through MXC_TMR5

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
int maxTicks = (int)MAX_TICKS;
volatile int TMR_FLAG = 0;

/***** Functions *****/
void PWMTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr; // to configure timer
    unsigned int periodTicks = MXC_TMR_GetPeriod(PWM_TIMER, 1, FREQ);
    unsigned int dutyTicks = periodTicks / 100 * 50;

    //Configure PWM GPIO Pin
    mxc_gpio_cfg_t pwmOut;
    pwmOut.port = PWM_PORT;
    pwmOut.mask = PWM_PIN;
    pwmOut.func = MXC_GPIO_FUNC_ALT3;
    pwmOut.pad = MXC_GPIO_PAD_NONE;
    pwmOut.vssel = MXC_GPIO_VSSEL_VDDIO;
    MXC_GPIO_Config(&pwmOut);

    /*    
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the pre-scale value
    3. Set polarity, PWM parameters
    4. Configure the timer for PWM mode
    5. Enable Timer
    */

    MXC_TMR_Shutdown(PWM_TIMER);

    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_PWM;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 1;

    MXC_TMR_Init(PWM_TIMER, &tmr);

    if (MXC_TMR_SetPWM(PWM_TIMER, dutyTicks) != E_NO_ERROR) {
        printf("Failed TMR_PWMConfig.\n");
    }

    MXC_TMR_Start(PWM_TIMER);

    printf("\nPWM started.\n\n");
}

// Toggles GPIO when continuous timer repeats
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
    uint32_t periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, 1, CONT_FREQ);

    /*    
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    MXC_TMR_Init(CONT_TIMER, &tmr);
    MXC_TMR_Start(CONT_TIMER);

    printf("Continuous timer started.\n\n");
}

void OneshotTimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(OST_TIMER);
    TMR_FLAG = 1;
}

void OneshotTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(OST_TIMER, 1, OST_FREQ);

    /*    
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(OST_TIMER);

    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_ONESHOT;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    TMR_FLAG = 0;
    MXC_TMR_Init(OST_TIMER, &tmr);
    MXC_TMR_Start(OST_TIMER);

    printf("Oneshot timer started.\n");

    while (TMR_FLAG != 1) {}

    printf("Oneshot timer finished.\n\n");
}

// *****************************************************************************
int main(void)
{
    //Exact timer operations can be found in tmr_utils.c
    printf("\n************************** Timer Example **************************\n\n");
    printf("1. A continuous mode timer is used to create an interrupt at freq of %d Hz.\n",
           CONT_FREQ);
    printf("   LED0 (Port 0.13) will toggle each time the interrupt occurs.\n\n");
    printf("2. Timer 0 is used to output a PWM signal on Port 0.3.\n");
    printf("   The PWM frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ, DUTY_CYCLE);
    printf("3. Timer 1 is configured in oneshot mode. It is used once and prints its\n");
    printf("   status before and after it is used.\n");

    PWMTimer();

    MXC_NVIC_SetVector(TMR1_IRQn, OneshotTimerHandler);
    NVIC_EnableIRQ(TMR1_IRQn);
    OneshotTimer();

    MXC_NVIC_SetVector(TMR2_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(TMR2_IRQn);
    ContinuousTimer();

    printf("\nStarted Timer");

    while (1) {}

    return 0;
}
