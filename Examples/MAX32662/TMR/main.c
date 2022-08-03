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
#include "gpio.h"
#include "led.h"
#include "lp.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "pb.h"
#include "tmr.h"

/***** Definitions *****/
//#define SLEEP_MODE          // Uncomment to enable sleep mode

// Parameters for PWM output
#define OST_CLOCK_SOURCE  MXC_TMR_32M_CLK // \ref mxc_tmr_clock_t
#define PWM_CLOCK_SOURCE  MXC_TMR_8M_CLK  // \ref mxc_tmr_clock_t
#define CONT_CLOCK_SOURCE MXC_TMR_APB_CLK // \ref mxc_tmr_clock_t

// Parameters for Continuous timer
#define OST_FREQ  1        // (Hz)
#define OST_TIMER MXC_TMR2 // Can be MXC_TMR0 through MXC_TMR3
mxc_gpio_cfg_t ost_pin;

#define FREQ       10 // (Hz)
#define DUTY_CYCLE 25 // (%)
#define PWM_TIMER \
    MXC_TMR0 // TMR0 maps to P0_3.  If this is changed, the PWM output pin will change.

// Parameters for Continuous timer
#define CONT_FREQ  2        // (Hz)
#define CONT_TIMER MXC_TMR1 // Can be MXC_TMR0 through MXC_TMR3

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

    if (MXC_TMR_Init(PWM_TIMER, &tmr, true, MAP_A) != E_NO_ERROR) {
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
    LED_Toggle(0);
}

void ContinuousTimer()
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

    tmr.pres    = TMR_PRES_512;
    tmr.mode    = TMR_MODE_CONTINUOUS;
    tmr.bitMode = TMR_BIT_MODE_16A;
    tmr.clock   = CONT_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol     = 0;

    if (MXC_TMR_Init(CONT_TIMER, &tmr, true, MAP_A) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(CONT_TIMER)), ContinuousTimerHandler);
    NVIC_EnableIRQ(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(CONT_TIMER)));
    MXC_TMR_EnableInt(CONT_TIMER);

    MXC_TMR_Start(CONT_TIMER);

    printf("Continuous timer started.\n\n");
}

void OneshotTimerHandler()
{
    MXC_GPIO_OutClr(ost_pin.port, ost_pin.mask);

    // Clear interrupt
    if (MXC_TMR_GetFlags(OST_TIMER)) {
        printf("Oneshot timer expired!\n");
        // Clear interrupt
        MXC_TMR_ClearFlags(OST_TIMER);
    }
}

void OneshotTimer()
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

    tmr.pres    = TMR_PRES_1024;
    tmr.mode    = TMR_MODE_ONESHOT;
    tmr.bitMode = TMR_BIT_MODE_16A;
    tmr.clock   = OST_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol     = 0;

    if (MXC_TMR_Init(OST_TIMER, &tmr, false, MAP_A) != E_NO_ERROR) {
        printf("Failed one-shot timer Initialization.\n");
        return;
    }

    MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(OST_TIMER)), OneshotTimerHandler);
    NVIC_EnableIRQ(MXC_TMR_GET_IRQ(MXC_TMR_GET_IDX(OST_TIMER)));
    MXC_TMR_EnableInt(OST_TIMER);

    // Enable wkup source in Poower seq register
    MXC_LP_EnableTimerWakeup(OST_TIMER);
    // Enable Timer wake-up source
    MXC_TMR_EnableWakeup(OST_TIMER, &tmr);

    ost_pin.port  = MXC_GPIO0;
    ost_pin.mask  = MXC_GPIO_PIN_7;
    ost_pin.func  = MXC_GPIO_FUNC_OUT;
    ost_pin.pad   = MXC_GPIO_PAD_NONE;
    ost_pin.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&ost_pin);
    MXC_GPIO_OutClr(ost_pin.port, ost_pin.mask);
}

void PB0Handler(void* pb)
{
    MXC_GPIO_OutSet(ost_pin.port, ost_pin.mask);
    MXC_TMR_Start(OST_TIMER);
    printf("Oneshot timer started.\n\n");
}

// *****************************************************************************
int main(void)
{
    //Exact timer operations can be found in tmr_utils.c

    printf("\n************************** Timer Example **************************\n\n");
    printf("This example demonstrates the following timer modes:\n");
    printf("1. Continuous timer: this timer is used to generate an interrupt at a\n");
    printf("   frequency of %d Hz. In the continuous timer ISR LED0 is toggled.\n", CONT_FREQ);
    printf("   Additionally, the continuous timer output signal is enabled and is\n");
    printf("   viewable on pin P0.5.\n\n");
    printf("2. PWM Timer:  this timer is used to output a PWM signal on pin P0.3.\n");
    printf("   The PWM frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ, DUTY_CYCLE);
    printf("3. Oneshot Timer:  this timer is configured in oneshot mode. Pressing\n");
    printf("   the pushbutton (SW2) will start the timer and P0.7 will be set high.\n");
    printf("   After the oneshot timer expires a message will be printed to the\n");
    printf("   console and P0.7 will be cleared.\n\n");

    ContinuousTimer();
    PWMTimer();
    OneshotTimer();

    PB_RegisterCallback(0, PB0Handler);

    while (1) {
#ifdef SLEEP_MODE
        MXC_LP_EnterSleepMode();
#endif
    }

    return 0;
}
