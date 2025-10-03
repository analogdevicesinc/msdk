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

/**
 * @file    main.c
 * @brief   Timer example
 * @details PWM Timer        - Outputs a PWM signal (2Hz, 30% duty cycle) on TMR2
 *          Continuous Timer - Outputs a continuous 1s timer on LED0 (GPIO toggles every 500s)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc.h"

/***** Definitions *****/
// #define SLEEP_MODE // Select between SLEEP_MODE and LP_MODE for LPTIMER

// Parameters for PWM output
#define OST_CLOCK_SOURCE MXC_TMR_ERTCO_CLK // \ref mxc_tmr_clock_t
#define PWM_CLOCK_SOURCE MXC_TMR_IBRO_CLK // \ref mxc_tmr_clock_t
#define CONT_CLOCK_SOURCE MXC_TMR_APB_CLK // \ref mxc_tmr_clock_t

// Parameters for Continuous timer
#define OST_FREQ 1 // (Hz)
#define OST_TIMER MXC_TMR3 // Can be MXC_TMR0 through MXC_TMR5
mxc_gpio_cfg_t ost_pin;

#define FREQ 10 // (Hz)
#define DUTY_CYCLE 25 // (%)
#define PWM_TIMER \
    MXC_TMR4 // TMR4 maps to P0_3.  If this is changed, the PWM output pin will change.

// Parameters for Continuous timer
#define CONT_FREQ 2 // (Hz)
#define CONT_TIMER MXC_TMR5 // Can be MXC_TMR0 through MXC_TMR5

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
    tmr.bitMode = TMR_BIT_MODE_32;
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
    // LED_Toggle(0);
}

void ContinuousTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, CONT_CLOCK_SOURCE, 512, CONT_FREQ);

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres = TMR_PRES_512;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.bitMode = TMR_BIT_MODE_16B;
    tmr.clock = CONT_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;

    if (MXC_TMR_Init(CONT_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(CONT_TIMER)), ContinuousTimerHandler);
    NVIC_EnableIRQ(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(CONT_TIMER)));
    MXC_TMR_EnableInt(CONT_TIMER);

    MXC_TMR_Start(CONT_TIMER);

    printf("Continuous timer started.\n\n");
}

void OneshotTimerHandler(void)
{
    MXC_GPIO_OutClr(ost_pin.port, ost_pin.mask);

    // Clear interrupt
    if (MXC_TMR_GetFlags(OST_TIMER)) {
        printf("Oneshot timer expired!\n");
        // Clear interrupt
        MXC_TMR_ClearFlags(OST_TIMER);
    }

    LED_Toggle(0);
}

void OneshotTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(OST_TIMER, OST_CLOCK_SOURCE, 1024, OST_FREQ);
    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(OST_TIMER);

    tmr.pres = TMR_PRES_1024;
    tmr.mode = TMR_MODE_ONESHOT;
    tmr.bitMode = TMR_BIT_MODE_16A;
    tmr.clock = OST_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    if (MXC_TMR_Init(OST_TIMER, &tmr, false) != E_NO_ERROR) {
        printf("Failed one-shot timer Initialization.\n");
        return;
    }

    MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(OST_TIMER)), OneshotTimerHandler);
    NVIC_EnableIRQ(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(OST_TIMER)));
    MXC_TMR_EnableInt(OST_TIMER);

    // Enable Timer wake-up source
    MXC_TMR_EnableWakeup(OST_TIMER, &tmr);

    ost_pin.port = MXC_GPIO0;
    ost_pin.mask = MXC_GPIO_PIN_2;
    ost_pin.func = MXC_GPIO_FUNC_OUT;
    ost_pin.pad = MXC_GPIO_PAD_NONE;
    ost_pin.vssel = MXC_GPIO_VSSEL_VDDIOH;
    ost_pin.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&ost_pin);
    MXC_GPIO_OutClr(ost_pin.port, ost_pin.mask);
}

// *****************************************************************************
int main(void)
{
    printf("\n************************** Timer Example **************************\n\n");
    printf("This example demonstrates the following timer modes:\n");
    printf("1. Continuous timer: this timer is used to generate an interrupt at a\n");
    printf("   frequency of %d Hz. In the continuous timer ISR LED0 is toggled.\n", CONT_FREQ);
    printf("   Additionally, the continuous timer output signal is enabled and is\n");
    printf("   viewable on pin P0.4.\n\n");
    printf("2. PWM Timer:  this timer is used to output a PWM signal on pin P0.3.\n");
    printf("   The PWM frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ, DUTY_CYCLE);
    printf("3. Oneshot Timer:  this timer is configured in oneshot mode. Pressing\n");
    printf("   the pushbutton (PB0) will start the timer and P0.2 will be set high.\n");
    printf("   After the oneshot timer expires a message will be printed to the\n");
    printf("   console and P0.2 will be cleared.\n\n");

    ContinuousTimer();
    PWMTimer();
    OneshotTimer();

    while (1) {
        if (PB_Get(0)) {
            LED_Toggle(0);
            MXC_GPIO_OutSet(ost_pin.port, ost_pin.mask);
            MXC_TMR_Start(OST_TIMER);
            printf("Oneshot timer started.\n\n");
        }

#ifdef SLEEP_MODE
        MXC_LP_EnterSleepMode();
#endif
    }

    return 0;
}
