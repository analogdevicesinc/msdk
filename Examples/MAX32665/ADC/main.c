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

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_pins.h"
#include "led.h"
#include "adc.h"
#include "tmr.h"

/* **** Definitions **** */

/* Use Interrupt Mode */
#define USE_INTERRUPTS

/* Uncomment if using polling mode */
#undef USE_INTERRUPTS

/* **** Globals **** */
#ifdef USE_INTERRUPTS
volatile unsigned int adc_done = 0;
#endif

/* **** Functions **** */

#ifdef USE_INTERRUPTS
void adc_complete_cb(void *req, int error)
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
    uint16_t adc_val[4];
    unsigned int overflow[4];
    uint8_t fmtstr[40];

    printf("ADC Example\n");

    /* Initialize ADC */
    if (MXC_ADC_Init() != E_NO_ERROR) {
        printf("Error Bad Parameter\n");

        while (1) {}
    }

    // Configure Inputs as ADC inputs
    MXC_GPIO_Config(&gpio_cfg_adc0);
    MXC_GPIO_Config(&gpio_cfg_adc1);
    MXC_GPIO_Config(&gpio_cfg_adc2);
    MXC_GPIO_Config(&gpio_cfg_adc3);

    /* Set up LIMIT0 to monitor high and low trip points */
    MXC_ADC_SetMonitorChannel(MXC_ADC_MONITOR_0, MXC_ADC_CH_0);
    MXC_ADC_SetMonitorHighThreshold(MXC_ADC_MONITOR_0, 0x300);
    MXC_ADC_SetMonitorLowThreshold(MXC_ADC_MONITOR_0, 0x25);
    MXC_ADC_EnableMonitor(MXC_ADC_MONITOR_0);

#ifdef USE_INTERRUPTS
    NVIC_EnableIRQ(ADC_IRQn);
#endif

    while (1) {
        /* Flash LED when starting ADC cycle */
        LED_On(0);
        MXC_TMR_Delay(MXC_TMR0, MXC_DELAY_MSEC(10));
        LED_Off(0);

        /* Convert channel 0 */
#ifdef USE_INTERRUPTS
        adc_done = 0;
        MXC_ADC_StartConversionAsync(MXC_ADC_CH_0, adc_complete_cb);
        while (!adc_done) {}
#else
        MXC_ADC_StartConversion(MXC_ADC_CH_0);
#endif
        overflow[0] = (MXC_ADC_GetData(&adc_val[0]) == E_OVERFLOW ? 1 : 0);

        /* Convert channel 1 */
#ifdef USE_INTERRUPTS
        adc_done = 0;
        MXC_ADC_StartConversionAsync(MXC_ADC_CH_1, adc_complete_cb);
        while (!adc_done) {}
#else
        MXC_ADC_StartConversion(MXC_ADC_CH_1);
#endif
        overflow[1] = (MXC_ADC_GetData(&adc_val[1]) == E_OVERFLOW ? 1 : 0);

        /* Convert channel 2 */
#ifdef USE_INTERRUPTS
        adc_done = 0;
        MXC_ADC_StartConversionAsync(MXC_ADC_CH_2, adc_complete_cb);
        while (!adc_done) {}
#else
        MXC_ADC_StartConversion(MXC_ADC_CH_2);
#endif
        overflow[2] = (MXC_ADC_GetData(&adc_val[2]) == E_OVERFLOW ? 1 : 0);

        /* Convert channel 3 */
#ifdef USE_INTERRUPTS
        adc_done = 0;
        MXC_ADC_StartConversionAsync(MXC_ADC_CH_3, adc_complete_cb);
        while (!adc_done) {}
#else
        MXC_ADC_StartConversion(MXC_ADC_CH_3);
#endif
        overflow[3] = (MXC_ADC_GetData(&adc_val[3]) == E_OVERFLOW ? 1 : 0);

        /* Display results on OLED display, display asterisk if overflow */
        snprintf((char *)fmtstr, sizeof((char *)fmtstr), "0: 0x%04x%s 2: 0x%04x%s", adc_val[0],
                 overflow[0] ? "*" : " ", adc_val[2], overflow[2] ? "*" : " ");
        printf("%s\n", fmtstr);

        snprintf((char *)fmtstr, sizeof((char *)fmtstr), "1: 0x%04x%s 3: 0x%04x%s", adc_val[1],
                 overflow[1] ? "*" : " ", adc_val[3], overflow[3] ? "*" : " ");
        printf("%s\n", fmtstr);
        /* Determine if programmable limits on AIN1 were exceeded */
        if (MXC_ADC_GetFlags() & (MXC_F_ADC_INTR_LO_LIMIT_IF | MXC_F_ADC_INTR_HI_LIMIT_IF)) {
            snprintf((char *)fmtstr, sizeof((char *)fmtstr), " %s Limit on AIN0 ",
                     (MXC_ADC_GetFlags() & MXC_F_ADC_INTR_LO_LIMIT_IF) ? "Low" : "High");
            MXC_ADC_ClearFlags(MXC_F_ADC_INTR_LO_LIMIT_IF | MXC_F_ADC_INTR_HI_LIMIT_IF);
        } else {
            snprintf((char *)fmtstr, sizeof((char *)fmtstr), "                   ");
        }
        printf("%s\n", fmtstr);

        /* Delay for 1/4 second before next reading */
        MXC_TMR_Delay(MXC_TMR0, MXC_DELAY_MSEC(250));
    }
}
