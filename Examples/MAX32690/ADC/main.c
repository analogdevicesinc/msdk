/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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
 * @brief   ADC demo application
 * @details Continuously monitors the ADC channels
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "adc.h"
#include "dma.h"
#include "fcr_regs.h"
#include "led.h"
#include "tmr.h"

/***** Definitions *****/

#define POLLING // Uncomment to perform ADC conversions using blocking/polling method
// #define INTERRUPT    // Uncomment to perform ADC conversions using interrupt driven method
// #define DMA          // Uncomment to perform ADC conversions using DMA driven method.

#define REF_TRIM //Uncomment to perform the reference trim after initialization

#define SOFTWARE // Uncomment to perform ADC conversions using a software trigger
// #define HARDWARE     // Uncomment to perform ADC conversions using a hardware trigger

#define CH MXC_ADC_CH_0

/* Supported three ADC examples */
#define SINGLE_CH 1
#define TEMP_SENSOR 2
#define MULTI_CHS 3
#define TEMP_SENSOR_READ_OUT 1

/***** Globals *****/
#ifdef INTERRUPT
volatile unsigned int adc_done = 0;
#endif

/* Temperature Sensor firmware average*/
#define SAMPLE_AVG 16
unsigned int temp_samples = 0;

/* Single Channel */
mxc_adc_slot_req_t single_slot = { MXC_ADC_CH_3 };

int adc_val[8];
uint32_t adc_index = 0;

mxc_adc_conversion_req_t adc_conv;

/***** Functions *****/

#ifdef INTERRUPT
void adc_complete_cb(void *req, int flags)
{
    if (flags & MXC_F_ADC_INTFL_SEQ_DONE) {
        adc_index += MXC_ADC_GetData(&adc_val[adc_index]);
        adc_done = 1;
    }

    if (flags & MXC_F_ADC_INTFL_FIFO_LVL) {
        adc_index += MXC_ADC_GetData(&adc_val[adc_index]);
    }
    return;
}

void ADC_IRQHandler(void)
{
    MXC_ADC_DisableConversion();

    MXC_ADC_Handler();
}
#endif

#ifdef HARDWARE
void StartHWTriggerTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr; // to configure timer
    unsigned int periodTicks = MXC_TMR_GetPeriod(MXC_TMR1, MXC_TMR_APB_CLK, 32, 1);

    /*
	Steps for configuring a timer for PWM mode:
	1. Disable the timer
	2. Set the pre-scale value
	3. Set polarity, PWM parameters
	4. Configure the timer for PWM mode
	5. Enable Timer
	*/

    MXC_TMR_Shutdown(MXC_TMR1);

    tmr.pres = TMR_PRES_32;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.bitMode = TMR_BIT_MODE_32;
    tmr.clock = MXC_TMR_APB_CLK;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 1;

    if (MXC_TMR_Init(MXC_TMR1, &tmr, false) != E_NO_ERROR) {
        printf("Failed PWM timer Initialization.\n");
        return;
    }

    MXC_TMR_Start(MXC_TMR1);

    printf("Timer started.\n\n");
}
#endif

/*ADC initialization*/
void adc_init(void)
{
    mxc_adc_req_t adc_cfg;

    adc_cfg.clock = MXC_ADC_HCLK;
    adc_cfg.clkdiv = MXC_ADC_CLKDIV_16;
    adc_cfg.cal = MXC_ADC_EN_CAL;
    adc_cfg.trackCount = 4;
    adc_cfg.idleCount = 17;
    adc_cfg.ref = MXC_ADC_REF_INT_1V25;

    /* Initialize ADC */
    if (MXC_ADC_Init(&adc_cfg) != E_NO_ERROR) {
        printf("Error Bad Parameter\n");
        while (1) {}
    }
}

/* Single channel */
void adc_example1_configuration(void)
{
    adc_conv.mode = MXC_ADC_ATOMIC_CONV;
    adc_conv.trig = MXC_ADC_TRIG_SOFTWARE;
    adc_conv.trig = MXC_ADC_TRIG_HARDWARE;
    adc_conv.hwTrig = MXC_ADC_TRIG_SEL_TMR0;
    adc_conv.avg_number = MXC_ADC_AVG_16;
    adc_conv.fifo_format = MXC_ADC_DATA_STATUS;
    adc_conv.fifo_threshold = MAX_ADC_FIFO_LEN >> 1;
    adc_conv.lpmode_divder = MXC_ADC_DIV_2_5K_50K_ENABLE;
    adc_conv.num_slots = 0;

    MXC_ADC_Configuration(&adc_conv);

    MXC_ADC_SlotConfiguration(&single_slot, 0);
}

void ShowAdcResult(void)
{
    unsigned int loop_count;

    // Stop the ADC
    MXC_ADC_DisableConversion();

    if (adc_conv.fifo_format == MXC_ADC_DATA_STATUS) {
        printf("CH : Data\n");
    } else {
        printf("Data\n");
    }
    for (loop_count = 0; loop_count < adc_index; loop_count++) {
        if (adc_conv.fifo_format == MXC_ADC_DATA_STATUS) {
            printf("%02X : %03X\n", (adc_val[loop_count] >> 16), (adc_val[loop_count] & 0x0FFF));
        } else {
            printf("%03X \n", adc_val[loop_count]);
        }
    }

    adc_index = 0;

    if (which_example == TEMP_SENSOR) {
        printTemperature();
    }
}

int main(void)
{
    printf("********** ADC Example **********\n");
    printf("\nThe voltage applied to analog pin continuously\n");
    printf("measured and the result is printed to the terminal.\n");
    printf("\nThe example can be configured to take the measurements\n");
    printf("by polling, using interrupts, or using DMA.\n\n");

    /* ADC Initialization */
    adc_init();

    /* Start with Single Channel Example */
    adc_example1_configuration();

#if defined(INTERRUPT)
    NVIC_EnableIRQ(ADC_IRQn);
#endif

    while (1) {
        /* Flash LED when starting ADC cycle */
        LED_On(0);
        MXC_TMR_Delay(MXC_TMR0, MSEC(10));
        LED_Off(0);

#ifdef INTERRUPT
        adc_done = 0;

        MXC_ADC_StartConversionAsync(adc_complete_cb);

        while (!adc_done) {}
#endif

        ShowAdcResult();

        printf("\n");
    }
}

//End
