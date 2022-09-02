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
 * @details PWM Timer        - Outputs a PWM signal (200 Hz, 75% duty cycle) on 0.14
 *          Continuous Timer - Outputs a continuous 1s timer on LED0 (GPIO toggles every 500s)
 *          One Shot Timer   - Starts a one shot timer - LED1 turns on when one shot time (3 sec) is complete
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
#define PORT_PWM   MXC_GPIO0       //port
#define PIN_PWM    MXC_GPIO_PIN_16 //pin
#define FREQ       200             // (Hz)
#define DUTY_CYCLE 75              // (%)
#define PWM_TIMER  MXC_TMR4        // must change PWM_PORT and PWM_PIN if changed

// Parameters for Continuous timer
#define INTERVAL_TIME_CONT 1        // (s) will toggle after every interval
#define CONT_TIMER         MXC_TMR0 // Can be MXC_TMR0 through MXC_TMR5
#define CONT_TIMER_IRQn    TMR0_IRQn

// Parameters for One-shot timer
#define INTERVAL_TIME_OST 3        // (s)
#define OST_TIMER         MXC_TMR1 // Can be MXC_TMR0 through MXC_TMR5
#define OST_TIMER_IRQn    TMR1_IRQn

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
    // Declare variables
    mxc_gpio_cfg_t gpio_pwm; //to configure GPIO
    mxc_tmr_cfg_t tmr;       // to configure timer
    unsigned int periodTicks = PeripheralClock / FREQ;
    unsigned int dutyTicks   = periodTicks * DUTY_CYCLE / 100;

    // Congfigure GPIO port and pin for PWM
    gpio_pwm.port = PORT_PWM;
    gpio_pwm.mask = PIN_PWM;
    gpio_pwm.pad  = MXC_GPIO_PAD_PULL_DOWN;
    gpio_pwm.func = MXC_GPIO_FUNC_ALT4;
    MXC_GPIO_Config(&gpio_pwm);

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the pre-scale value
    3. Set polarity, PWM parameters
    4. Configure the timer for PWM mode
    5. Enable Timer
    */

    MXC_TMR_Shutdown(PWM_TIMER);

    tmr.pres    = TMR_PRES_1;
    tmr.mode    = TMR_MODE_PWM;
    tmr.cmp_cnt = periodTicks;
    tmr.pol     = 1;

    MXC_TMR_Init(PWM_TIMER, &tmr);

    if (MXC_TMR_SetPWM(PWM_TIMER, dutyTicks) != E_NO_ERROR) {
        printf("Failed TMR_PWMConfig.\n");
    }

    MXC_TMR_Start(PWM_TIMER);

    printf("PWM started.\n\n");
}

// Toggles GPIO when continuous timer repeats
void ContinuousTimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(CONT_TIMER);
    MXC_GPIO_OutToggle(led_pin[0].port, led_pin[0].mask);
}

void ContinuousTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = PeripheralClock / 4 * INTERVAL_TIME_CONT;

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres    = TMR_PRES_4;
    tmr.mode    = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol     = 0;

    MXC_TMR_Init(CONT_TIMER, &tmr);

    MXC_TMR_Start(CONT_TIMER);

    printf("Continuous timer started.\n\n");
}

void OneshotTimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(OST_TIMER);
    MXC_GPIO_OutToggle(led_pin[1].port, led_pin[1].mask);
}

void OneshotTimer()
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = PeripheralClock / 128 * INTERVAL_TIME_OST;
    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(OST_TIMER);

    tmr.pres    = TMR_PRES_128;
    tmr.mode    = TMR_MODE_ONESHOT;
    tmr.cmp_cnt = periodTicks;
    tmr.pol     = 0;

    MXC_TMR_Init(OST_TIMER, &tmr);

    MXC_TMR_Start(OST_TIMER);

    printf("Oneshot timer started.\n\n");
}

// *****************************************************************************
int main(void)
{
    printf("\n************************** Timer Example **************************\n\n");
    printf("1. A continuous mode timer is used to create an interrupt every %d sec.\n",
           INTERVAL_TIME_CONT);
    printf("   LED0 will toggle each time the interrupt occurs.\n\n");
    printf("2. Timer 4 is used to output a PWM signal on Port 0.16.\n");
    printf("   The PWM frequency is %d Hz and the duty cycle is %d%%.\n\n", FREQ, DUTY_CYCLE);
    printf("3. A one shot mode timer is used to turn on LED1 after %d sec.\n\n", INTERVAL_TIME_OST);

    PWMTimer();

    MXC_NVIC_SetVector(CONT_TIMER_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(CONT_TIMER_IRQn);
    ContinuousTimer();

    MXC_NVIC_SetVector(OST_TIMER_IRQn, OneshotTimerHandler);
    NVIC_EnableIRQ(OST_TIMER_IRQn);
    OneshotTimer();

    while (1) {
        ;
    }

    return 0;
}
