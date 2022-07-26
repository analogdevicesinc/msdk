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
 * @brief   ADC demo application
 * @details Continuously monitors the ADC channels
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "adc.h"
#include "led.h"

/***** Definitions *****/

/* Change to #undef USE_INTERRUPTS for polling mode */
// #define USE_INTERRUPTS

#define ADC_CHANNEL MXC_ADC_CH_0

/***** Globals *****/
#ifdef USE_INTERRUPTS
volatile unsigned int adc_done = 0;
#endif

static uint16_t adc_val;

/***** Functions *****/

#ifdef USE_INTERRUPTS
void adc_complete_cb(void* req, int error)
{
    adc_done = 1;
    return;
}
void ADC_IRQHandler(void)
{
    MXC_ADC_Handler();
}
#endif

int main(void)
{
    // unsigned int overflow;

    printf("\n***** ADC Example ***** \n");

    /* Initialize ADC */
    if (MXC_ADC_Init() != E_NO_ERROR) {
        printf("Error Bad Parameter\n");

        while (1)
            ;
    }

    /* Set up LIMIT0 to monitor high and low trip points */
    while (MXC_ADC->status & (MXC_F_ADC_STATUS_ACTIVE | MXC_F_ADC_STATUS_PWR_UP_ACTIVE))
        ;
    MXC_ADC_SetMonitorChannel(MXC_ADC_MONITOR_3, ADC_CHANNEL);
    MXC_ADC_SetMonitorHighThreshold(MXC_ADC_MONITOR_3, 0x300);
    MXC_ADC_SetMonitorLowThreshold(MXC_ADC_MONITOR_3, 0x25);
    MXC_ADC_EnableMonitor(MXC_ADC_MONITOR_3);

#ifdef USE_INTERRUPTS
    NVIC_EnableIRQ(ADC_IRQn);
#endif

    while (1) {
        /* Flash LED when starting ADC cycle */
        LED_On(1);
        MXC_Delay(MXC_DELAY_MSEC(10));
        LED_Off(1);

        /* Convert channel 0 */
#ifdef USE_INTERRUPTS
        adc_done = 0;
        MXC_ADC_StartConversionAsync(ADC_CHANNEL, adc_complete_cb);

        while (!adc_done) {
        };

#else
        MXC_ADC_StartConversion(ADC_CHANNEL);

#endif
        static uint8_t overflow;

        overflow = (MXC_ADC_GetData(&adc_val) == E_OVERFLOW ? 1 : 0);

        /* Determine if programmable limits on AIN0 were exceeded */
        //MXC_ADC_GetFlags()
        if (MXC_ADC_GetFlags() & (MXC_F_ADC_INTR_LO_LIMIT_IF | MXC_F_ADC_INTR_HI_LIMIT_IF)) {
            printf("%s Limit on AIN0 ",
                   (MXC_ADC_GetFlags() & MXC_F_ADC_INTR_LO_LIMIT_IF) ? "Low" : "High");
            MXC_ADC_ClearFlags(MXC_F_ADC_INTR_LO_LIMIT_IF | MXC_F_ADC_INTR_HI_LIMIT_IF);
        } else {
            printf("                   ");
        }
        printf("\n");

        /* Display results on OLED display, display asterisk if overflow */
        printf("0: 0x%04x%s\n\n", adc_val, overflow ? "*" : " ");

        /* Delay for 1/4 second before next reading */
        MXC_Delay(MXC_DELAY_MSEC(1000));
    }
}
