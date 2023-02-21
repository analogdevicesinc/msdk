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
 * @brief   ADC Example
 * @details This example configures the AFE of the MAX32680 ADCs to sample input
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "board.h"
#include "led.h"
#include "mxc_delay.h"
#include "afe.h"

/***** Definitions *****/
#define ADC_CONVERSIONS_PER_SECOND 50
#define ADC_SAMPLE_RATE_120SPS 7
#define ADC_CH0 0
#define ADC_CH1 1
#define ADC_CH2 2
#define ADC_CH3 3
#define GPIO_OUT 3
#define DATA_READY_INT 2

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    MXC_Delay(MXC_DELAY_SEC(1));
    uint32_t read_val = 0;
    uint64_t avg_adc_0 = 0;
    uint64_t avg_adc_1 = 0;

    printf("\n\n\n\n\nMAX32680 ADC Example\n\n");

    printf("ADC0 and ADC1 are set to sample differentially between AIN2 (pos)\n");
    printf("and AIN3 (neg). Both ADCs are configured to a sampling rate of 120\n");
    printf("samples per second. ADC0 has a PGA gain of 8X for its input, ADC1 uses 16X\n");
    printf("input gain. ADC sample data is stored in the DATA0 register of each ADC.\n");
    printf("This example gets 50 samples from each, and calculates and prints\n");
    printf("the average reading.\n");

    printf("\nSampling will begin after 5 seconds...\n");

    MXC_Delay(MXC_DELAY_SEC(5));

    //
    // Enable ADCs
    //

    // Set AFE power state to normal
    afe_write_register(MXC_R_AFE_ADC_ZERO_PD, MXC_S_AFE_ADC_ZERO_PD_PD_RESET);
    afe_write_register(MXC_R_AFE_ADC_ONE_PD, MXC_S_AFE_ADC_ONE_PD_PD_RESET);

    afe_write_register(MXC_R_AFE_ADC_ZERO_PD, MXC_S_AFE_ADC_ZERO_PD_PD_NORMAL_MODE);
    afe_write_register(MXC_R_AFE_ADC_ONE_PD, MXC_S_AFE_ADC_ONE_PD_PD_NORMAL_MODE);

    // Set Reference voltages to be the Analog Power supply and ground
    afe_write_register(MXC_R_AFE_ADC_ZERO_CTRL, MXC_S_AFE_ADC_ZERO_CTRL_REF_SEL_AVDD_AND_AGND &
                                                    MXC_F_AFE_ADC_ZERO_CTRL_REF_SEL);
    afe_write_register(MXC_R_AFE_ADC_ONE_CTRL, MXC_S_AFE_ADC_ONE_CTRL_REF_SEL_AVDD_AND_AGND &
                                                   MXC_F_AFE_ADC_ONE_CTRL_REF_SEL);

    // Select AINP and AINN for AIN0
    afe_write_register(MXC_R_AFE_ADC_ZERO_MUX_CTRL0,
                       (((ADC_CH2 << MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINP_SEL_POS) &
                         MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINP_SEL) |
                        ((ADC_CH3 << MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINN_SEL_POS) &
                         MXC_F_AFE_ADC_ZERO_MUX_CTRL0_AINN_SEL)));

    afe_write_register(MXC_R_AFE_ADC_ONE_MUX_CTRL0,
                       (((ADC_CH2 << MXC_F_AFE_ADC_ONE_MUX_CTRL0_AINP_SEL_POS) &
                         MXC_F_AFE_ADC_ONE_MUX_CTRL0_AINP_SEL) |
                        ((ADC_CH3 << MXC_F_AFE_ADC_ONE_MUX_CTRL0_AINN_SEL_POS) &
                         MXC_F_AFE_ADC_ONE_MUX_CTRL0_AINN_SEL)));

    // Filter Options, Select SINC4 @ 120 samples per second
    afe_write_register(MXC_R_AFE_ADC_ZERO_FILTER,
                       MXC_S_AFE_ADC_ZERO_FILTER_LINEF_SINC4 |
                           ((ADC_SAMPLE_RATE_120SPS << MXC_F_AFE_ADC_ZERO_FILTER_RATE_POS) &
                            MXC_F_AFE_ADC_ZERO_FILTER_RATE));

    afe_write_register(MXC_R_AFE_ADC_ONE_FILTER,
                       MXC_S_AFE_ADC_ONE_FILTER_LINEF_SINC4 |
                           ((ADC_SAMPLE_RATE_120SPS << MXC_F_AFE_ADC_ONE_FILTER_RATE_POS) &
                            MXC_F_AFE_ADC_ONE_FILTER_RATE));

    // Bypass mode for input
    afe_write_register(MXC_R_AFE_ADC_ZERO_PGA, MXC_S_AFE_ADC_ZERO_PGA_SIG_PATH_PGA_PATH |
                                                   MXC_S_AFE_ADC_ZERO_PGA_GAIN_GAIN_8X);
    afe_write_register(MXC_R_AFE_ADC_ONE_PGA, MXC_S_AFE_ADC_ONE_PGA_SIG_PATH_PGA_PATH |
                                                  MXC_S_AFE_ADC_ONE_PGA_GAIN_GAIN_16X);

    //Enable ADC_RDY GPIO outputs
    afe_write_register(MXC_R_AFE_ADC_ZERO_GP0_CTRL,
                       (((GPIO_OUT << MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_DIR_POS) &
                         MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_DIR) |
                        ((DATA_READY_INT << MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_OSEL_POS) &
                         MXC_F_AFE_ADC_ZERO_GP0_CTRL_GP0_OSEL)));

    afe_write_register(MXC_R_AFE_ADC_ONE_GP0_CTRL,
                       (((GPIO_OUT << MXC_F_AFE_ADC_ONE_GP0_CTRL_GP0_DIR_POS) &
                         MXC_F_AFE_ADC_ONE_GP0_CTRL_GP0_DIR) |
                        ((DATA_READY_INT << MXC_F_AFE_ADC_ONE_GP0_CTRL_GP0_OSEL_POS) &
                         MXC_F_AFE_ADC_ONE_GP0_CTRL_GP0_OSEL)));

    //
    // Infinite ADC reading loop
    //
    while (1) {
        //
        // Get 50 samples from each ADC
        //
        for (int adc_loop_count = 0; adc_loop_count < ADC_CONVERSIONS_PER_SECOND;
             adc_loop_count++) {
            // Start a single Conversion
            afe_write_register(MXC_R_AFE_ADC_ZERO_CONV_START,
                               MXC_S_AFE_ADC_ZERO_CONV_START_CONV_TYPE_SINGLE);

            afe_write_register(MXC_R_AFE_ADC_ONE_CONV_START,
                               MXC_S_AFE_ADC_ONE_CONV_START_CONV_TYPE_SINGLE);

            // NOTE: Wait for DATA_RDY on ADC1, so we don't need to bank swap back to 0, and assume it will finish first.
            // Also, this is only safe to do if ADCs are sampling at the same RATE.

            while (1) {
                afe_read_register(MXC_R_AFE_ADC_ONE_STATUS, &read_val);

                if (read_val & MXC_F_AFE_ADC_ONE_STATUS_DATA_RDY) {
                    break;
                }
            }

            // Now read data
            afe_read_register(MXC_R_AFE_ADC_ONE_DATA0, &read_val);
            avg_adc_1 += read_val;

            // Now read data
            afe_read_register(MXC_R_AFE_ADC_ZERO_DATA0, &read_val);
            avg_adc_0 += read_val;

            // Reading this data last also save another bank swap at the top.
        }

        // Calculate and display average readings
        avg_adc_0 = avg_adc_0 / ADC_CONVERSIONS_PER_SECOND;
        printf("ADC0 AVG val: %08llX\n", avg_adc_0);

        avg_adc_1 = avg_adc_1 / ADC_CONVERSIONS_PER_SECOND;
        printf("ADC1 AVG val: %08llX\n\n", avg_adc_1);

        // Reset ADC value averages
        avg_adc_0 = 0;
        avg_adc_1 = 0;
    } // END of ADC read loop
}
