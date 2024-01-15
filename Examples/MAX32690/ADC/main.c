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

#ifdef DMA
volatile unsigned int dma_done = 0;
#endif

#ifdef REF_TRIM
//These values are dependent on the sample rate of the ADC. For 250ksps, the
//values are below. See table 11-4 in the user guide for other sample rates
#define TRIM_BIAS_COUNT 5
#define TRIM_WAKE_COUNT 3
#endif

/* Temperature Sensor firmware average*/
#define SAMPLE_AVG 16
float TEMP_SAMPLES[SAMPLE_AVG] = {};
float sum = 0;
unsigned int temp_samples = 0;

unsigned int which_example = 0; //0 - Single, 1 - Temperature (3 slots) and 2 - Eight slot

/* Single Channel */
mxc_adc_slot_req_t single_slot = { MXC_ADC_CH_3 };

/* It is recommended to use below sequence if only user wants to measure only temperature measurement.*/
mxc_adc_slot_req_t three_slots[3] = { { MXC_ADC_CH_VDDA_DIV2 },
                                      { MXC_ADC_CH_TEMP_SENS },
                                      { MXC_ADC_CH_VCOREA } };

mxc_adc_slot_req_t multi_slots[8] = { { MXC_ADC_CH_0 }, { MXC_ADC_CH_1 }, { MXC_ADC_CH_2 },
                                      { MXC_ADC_CH_3 }, { MXC_ADC_CH_4 }, { MXC_ADC_CH_5 },
                                      { MXC_ADC_CH_6 }, { MXC_ADC_CH_7 } };

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

#ifdef DMA
void ADC_IRQHandler(void)
{
    MXC_ADC_ClearFlags(0xFFFFFFFF);
}

void adc_dma_callback(int ch, int err)
{
    adc_index = adc_conv.num_slots + 1;

    MXC_ADC_DisableConversion();

    dma_done = 1;
}

void DMA_IRQHandler(void)
{
    MXC_DMA_Handler();
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

#ifdef REF_TRIM
/* Performs the 1.25V Internal Reference Trim
 * Derived from UG7618 Section 11.6.3 - 1.25V Internal Reference Trim
 */
void adc_internal_1v25_ref_trim()
{
    uint32_t tempData;

    //Enter nap state
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_ADC_EN;

    //Clear Skip Cal
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_SKIP_CAL;

    MXC_ADC->sfraddr = 0x0B;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xC0;
    tempData |= ((MXC_FCR->adcreftrim0 & MXC_F_FCR_ADCREFTRIM0_VX2_TUNE) >>
                 MXC_F_FCR_ADCREFTRIM0_VX2_TUNE_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0C;
    tempData = ((MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_IBOOST_1P25) >>
                MXC_F_FCR_ADCREFTRIM2_IBOOST_1P25_POS)
               << 7;
    tempData |=
        ((MXC_FCR->adcreftrim0 & MXC_F_FCR_ADCREFTRIM0_VREFP) >> MXC_F_FCR_ADCREFTRIM0_VREFP_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0D;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0x80;
    tempData |=
        ((MXC_FCR->adcreftrim0 & MXC_F_FCR_ADCREFTRIM0_VREFM) >> MXC_F_FCR_ADCREFTRIM0_VREFM_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0E;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0x0C;
    tempData |= ((MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_IDRV_1P25) >>
                 MXC_F_FCR_ADCREFTRIM2_IDRV_1P25_POS)
                << 4;
    tempData |=
        ((MXC_FCR->adcreftrim0 & MXC_F_FCR_ADCREFTRIM0_VCM) >> MXC_F_FCR_ADCREFTRIM0_VCM_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x05;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= TRIM_BIAS_COUNT;
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x06;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= TRIM_WAKE_COUNT;
    MXC_ADC->sfrwrdata = tempData;

    //Enable
    MXC_ADC->ctrl0 |= MXC_F_ADC_CTRL0_ADC_EN;

    //Wait for 30us
    MXC_Delay(30);

    //wait for calibration to complete
    while (!(MXC_ADC->intfl & MXC_F_ADC_INTFL_READY)) {}
}

/* Performs the 2.048V Internal Reference Trim
 * Derived from UG7618 Section 11.6.4 - 2.048V Internal Reference Trim
 */
void adc_internal_2v048_ref_trim()
{
    uint32_t tempData;

    //Enter nap state
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_ADC_EN;

    //Clear Skip Cal
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_SKIP_CAL;

    MXC_ADC->sfraddr = 0x0B;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xC0;
    tempData |= ((MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VX2_TUNE) >>
                 MXC_F_FCR_ADCREFTRIM1_VX2_TUNE_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0C;
    tempData = ((MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048) >>
                MXC_F_FCR_ADCREFTRIM2_IBOOST_2P048_POS)
               << 7;
    tempData |=
        ((MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VREFP) >> MXC_F_FCR_ADCREFTRIM1_VREFP_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0D;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0x80;
    tempData |=
        ((MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VREFM) >> MXC_F_FCR_ADCREFTRIM1_VREFM_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0E;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0x0C;
    tempData |= ((MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_IDRV_2P048) >>
                 MXC_F_FCR_ADCREFTRIM2_IDRV_2P048_POS)
                << 4;
    tempData |=
        ((MXC_FCR->adcreftrim1 & MXC_F_FCR_ADCREFTRIM1_VCM) >> MXC_F_FCR_ADCREFTRIM1_VCM_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x05;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= TRIM_BIAS_COUNT;
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x06;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= TRIM_WAKE_COUNT;
    MXC_ADC->sfrwrdata = tempData;

    //Enable
    MXC_ADC->ctrl0 |= MXC_F_ADC_CTRL0_ADC_EN;

    //Wait for 30us
    MXC_Delay(30);

    //wait for calibration to complete
    while (!(MXC_ADC->intfl & MXC_F_ADC_INTFL_READY)) {}
}

/* Performs the External Reference Trim
 * Derived from UG7618 Section 11.6.5 - External Reference Trim
 */
void adc_external_ref_trim()
{
    uint32_t tempData;

    //Enter nap state
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_ADC_EN;

    //Clear Skip Cal
    MXC_ADC->ctrl0 &= ~MXC_F_ADC_CTRL0_SKIP_CAL;

    MXC_ADC->sfraddr = 0x0B;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0x80;
    tempData |= ((MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_VX2_TUNE) >>
                 MXC_F_FCR_ADCREFTRIM2_VX2_TUNE_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x0E;
    tempData = MXC_ADC->sfrrddata;
    tempData |=
        ((MXC_FCR->adcreftrim2 & MXC_F_FCR_ADCREFTRIM2_VCM) >> MXC_F_FCR_ADCREFTRIM2_VCM_POS);
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x05;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= TRIM_BIAS_COUNT;
    MXC_ADC->sfrwrdata = tempData;

    MXC_ADC->sfraddr = 0x06;
    tempData = MXC_ADC->sfrrddata;
    tempData &= 0xF0;
    tempData |= TRIM_WAKE_COUNT;
    MXC_ADC->sfrwrdata = tempData;

    //Enable
    MXC_ADC->ctrl0 |= MXC_F_ADC_CTRL0_ADC_EN;

    //Wait for 30us
    MXC_Delay(30);

    //wait for calibration to complete
    while (!(MXC_ADC->intfl & MXC_F_ADC_INTFL_READY)) {}
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

#ifdef REF_TRIM
    switch (adc_cfg.ref) {
    case MXC_ADC_REF_INT_1V25:
        adc_internal_1v25_ref_trim();
        break;
    case MXC_ADC_REF_INT_2V048:
        adc_internal_2v048_ref_trim();
        break;
    case MXC_ADC_REF_EXT:
        adc_external_ref_trim();
        break;
    }
#endif
}

/* Single channel */
void adc_example1_configuration(void)
{
    adc_conv.mode = MXC_ADC_ATOMIC_CONV;
    adc_conv.trig = MXC_ADC_TRIG_SOFTWARE;
    //adc_conv.trig = MXC_ADC_TRIG_HARDWARE;
    //adc_conv.hwTrig = MXC_ADC_TRIG_SEL_TEMP_SENS;
    adc_conv.avg_number = MXC_ADC_AVG_16;
    adc_conv.fifo_format = MXC_ADC_DATA_STATUS;
#ifdef DMA
    adc_conv.fifo_threshold = 0;
#else
    adc_conv.fifo_threshold = MAX_ADC_FIFO_LEN >> 1;
#endif
    adc_conv.lpmode_divder = MXC_ADC_DIV_2_5K_50K_ENABLE;
    adc_conv.num_slots = 0;

    MXC_ADC_Configuration(&adc_conv);

    MXC_ADC_SlotConfiguration(&single_slot, 0);
}

/* Temperature Sensor */
void adc_example2_configuration(void)
{
    adc_conv.mode = MXC_ADC_ATOMIC_CONV;
    //adc_conv.trig = MXC_ADC_TRIG_SOFTWARE;
    adc_conv.trig = MXC_ADC_TRIG_HARDWARE;
    adc_conv.hwTrig = MXC_ADC_TRIG_SEL_TEMP_SENS;
    adc_conv.avg_number = MXC_ADC_AVG_8;
    adc_conv.fifo_format = MXC_ADC_DATA_STATUS;
#ifdef DMA
    adc_conv.fifo_threshold = 2;
#else
    adc_conv.fifo_threshold = MAX_ADC_FIFO_LEN >> 1;
#endif
    adc_conv.lpmode_divder = MXC_ADC_DIV_2_5K_50K_ENABLE;
    adc_conv.num_slots = 2;

    MXC_ADC_Configuration(&adc_conv);

    MXC_ADC_SlotConfiguration(three_slots, 2);
}

/* Multi Channel Example */
void adc_example3_configuration(void)
{
    adc_conv.mode = MXC_ADC_ATOMIC_CONV;
    adc_conv.trig = MXC_ADC_TRIG_SOFTWARE;
    //adc_conv.trig = MXC_ADC_TRIG_HARDWARE;
    //adc_conv.hwTrig = MXC_ADC_TRIG_SEL_TEMP_SENS;
    adc_conv.avg_number = MXC_ADC_AVG_1;
    adc_conv.fifo_format = MXC_ADC_DATA_STATUS;
#ifdef DMA
    adc_conv.fifo_threshold = 7;
#else
    adc_conv.fifo_threshold = MAX_ADC_FIFO_LEN >> 1;
#endif
    //    adc_conv.fifo_threshold = 8;
    adc_conv.lpmode_divder = MXC_ADC_DIV_2_5K_50K_ENABLE;
    adc_conv.num_slots = 7;

    MXC_ADC_Configuration(&adc_conv);

    MXC_ADC_SlotConfiguration(multi_slots, 7);
}

void temperature_average(float temperature)
{
    unsigned int loop_counter = 0;
    float average;
    TEMP_SAMPLES[temp_samples++] = temperature;
    sum += temperature;

    if (temp_samples == SAMPLE_AVG) {
        average = sum / SAMPLE_AVG;
        printf("Average = %0.2fC\n", (double)average);

        for (loop_counter = 0; loop_counter < SAMPLE_AVG; loop_counter++) {
            printf("%0.2fC ", (double)TEMP_SAMPLES[loop_counter]);
            if (loop_counter == 15) {
                printf("\n");
            }
        }
        printf("\n");

        temp_samples = 0;
        sum = 0;
        MXC_TMR_Delay(MXC_TMR0, MSEC(3000));
    }
}

void printTemperature(void)
{
    float temperature;
    MXC_ConvertTemperature_ToK((adc_val[TEMP_SENSOR_READ_OUT] & 0x0FFF), MXC_ADC_REF_INT_2V048, 0,
                               &temperature);

    MXC_ConvertTemperature_ToC((adc_val[TEMP_SENSOR_READ_OUT] & 0x0FFF), MXC_ADC_REF_INT_2V048, 0,
                               &temperature);
    temperature_average(temperature);
}

void WaitforConversionComplete(void)
{
    unsigned int flags;
    while (1) {
        flags = MXC_ADC_GetFlags();

        if (flags & MXC_F_ADC_INTFL_SEQ_DONE) {
            adc_index += MXC_ADC_GetData(&adc_val[adc_index]);
            //printf("ADC Count2 = %X\n", adc_index);
            break;
        }

        if (flags & MXC_F_ADC_INTFL_FIFO_LVL) {
            adc_index += MXC_ADC_GetData(&adc_val[adc_index]);
            //printf("ADC Count1 = %X\n", adc_index);
        }
    }
}

void ShowAdcResult(void)
{
    unsigned int loop_count;

    if (which_example == TEMP_SENSOR) {
        /* Disable Temperature Sensor Select */
        MXC_ADC_TS_SelectDisable();
    }

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

void adc_temp_conversion(void)
{
    if (which_example == TEMP_SENSOR) {
        /* Enable Temperature Sensor Select */
        MXC_ADC_TS_SelectEnable();
        /* Wait for Temperature measurement to be ready */
        MXC_TMR_Delay(MXC_TMR0, USEC(500));
    }

    MXC_ADC_StartConversion();
}

void run_examples(void)
{
    which_example++;

    if (which_example > MULTI_CHS) {
        which_example = SINGLE_CH;
    }

    switch (which_example) {
    case SINGLE_CH:
        printf("\nRunning Single Channel Example\n");
        adc_example1_configuration();
        break;

    case TEMP_SENSOR:
        printf("\nRunning Temperature Sensor Example\n");
        adc_example2_configuration();
        break;

    case MULTI_CHS:
        printf("\nRunning Multi Channel Example\n");
        adc_example3_configuration();
        break;

    default:
        which_example = SINGLE_CH;
        adc_example1_configuration();
        break;
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

#if defined(INTERRUPT) || defined(DMA)
    NVIC_EnableIRQ(ADC_IRQn);
#endif

#ifdef DMA
    MXC_DMA_Init();
#endif
    while (1) {
        /* Flash LED when starting ADC cycle */
        LED_On(0);
        MXC_TMR_Delay(MXC_TMR0, MSEC(10));
        LED_Off(0);

#ifdef POLLING
        adc_temp_conversion();
        WaitforConversionComplete();
#endif

#ifdef INTERRUPT
        adc_done = 0;
        if (which_example == TEMP_SENSOR) {
            /* Enable Temperature Sensor Select */
            MXC_ADC_TS_SelectEnable();

            /* Wait for Temperature measurement to be ready */
            MXC_TMR_Delay(MXC_TMR0, USEC(500));
        }

        MXC_ADC_StartConversionAsync(adc_complete_cb);

        while (!adc_done) {}
#endif

#ifdef DMA
        dma_done = 0;
        if (which_example == TEMP_SENSOR) {
            /* Enable Temperature Sensor Select */
            MXC_ADC_TS_SelectEnable();

            /* Wait for Temperature measurement to be ready */
            MXC_TMR_Delay(MXC_TMR0, USEC(500));
        }

        int dma_channel = MXC_DMA_AcquireChannel();
        adc_conv.dma_channel = dma_channel;

        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(dma_channel), DMA_IRQHandler);
        NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(dma_channel));
        MXC_ADC_StartConversionDMA(&adc_conv, &adc_val[0], adc_dma_callback);

        while (!dma_done) {}

        MXC_DMA_ReleaseChannel(adc_conv.dma_channel);
#endif
        ShowAdcResult();

        printf("\n");

        /* Delay for 1 second before next reading */
        MXC_TMR_Delay(MXC_TMR0, MSEC(1000));
        run_examples();
    }
}

//End
