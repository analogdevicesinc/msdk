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
 * @brief   MAX11261 ADC demo application
 * @details Continuously monitors the ADC channels
 */

/***** Includes *****/
#include <errno.h>
#include <stdio.h>
#include <stdint.h>

#include "mxc_delay.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "gpio.h"
#include "i2c.h"
#include "tmr.h"

#include "max11261.h"

#include "led.h"
#include "pb.h"

/***** Definitions *****/
#define I2C_MASTER MXC_I2C1 // SDA P2_17; SCL P2_18
#define I2C_FREQ 400 // 100kHz

#define ADC_V_AVDD 3000 // 3V
#define ADC_V_REF 2500 // 2.5V

#define ADC_SLAVE_ADDR 0x30 // Depends on ADR0 and ADR1 pins

#define PB_0 0 // Use push button 0 for channel switching
#define PB_1 1 // Use push button 1 for channel switching

/**
 * Event flags that will be handled in main loop
 */
#define FLAG_CHANNEL_PRESSED (0x01UL << 0)
#define FLAG_RATE_PRESSED (0x01UL << 1)

/***** Globals *****/
volatile uint32_t ticksUs = 0;
static volatile uint32_t flags = 0;

/***** Functions *****/
static void print_results(const max11261_adc_result_t *res, int count, uint32_t ticks);
static void pb_irq_handler(void *pb);
static void sys_timer_handler(void);

int main(void)
{
    int error;
    mxc_tmr_cfg_t tmrCfg;
    max11261_adc_channel_t channel = MAX11261_ADC_CHANNEL_0;
    max11261_single_rate_t rate = MAX11261_SINGLE_RATE_50;
    max11261_adc_result_t adcRes;
    uint32_t tickStart;
    uint16_t sampleCount;

    printf("\n******************** MAX11261 ADC Example ********************\n");
    printf("Demonstrates various features of MAX11261 ADC.\n\n");
    printf("An input voltage between -Vref and +Vref can be applied to AIN \n");
    printf("inputs. Conversion results for any input voltage outside this \n");
    printf("range will be clipped to the minimum or maximum level.\n\n");
    printf("Use PB0 to change the input channel being converted and PB1 to \n");
    printf("change the conversion rate\n\n\n");

    /* Setup I2C master */
    error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("Failed to initialize I2C%d master!\n", I2C_MASTER, error);
        return -1;
    }

    error = MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ * 1000);
    if (error < 0) {
        printf("Failed to set I2C bus speed to %d kHz\n", I2C_FREQ);
        return -1;
    }

    /* Enable push button interrupts */
    PB_IntEnable(PB_0);
    PB_IntEnable(PB_1);
    PB_RegisterCallback(PB_0, pb_irq_handler);
    PB_RegisterCallback(PB_1, pb_irq_handler);

    /* Setup timer 0 as system tick timer */
    /* Configure for 1us */
    tmrCfg.cmp_cnt = (PeripheralClock / 1000000) / 4;
    tmrCfg.mode = TMR_MODE_CONTINUOUS;
    tmrCfg.pol = 0;
    tmrCfg.pres = TMR_PRES_4;
    MXC_NVIC_SetVector(TMR0_IRQn, sys_timer_handler);
    NVIC_EnableIRQ(TMR0_IRQn);
    MXC_TMR_Init(MXC_TMR0, &tmrCfg);
    MXC_TMR_Start(MXC_TMR0);

    /* Set ADC hardware parameters */
    error = max11261_adc_config_init(ADC_V_AVDD, ADC_V_REF, I2C_FREQ, ADC_SLAVE_ADDR);
    if (error != E_NO_ERROR) {
        printf("Failed to initialize MAX11261\n");
        return -1;
    }

    /* Reset ADC */
    max11261_adc_reset();

    /* Set ADC sequencer parameters. Default values are already set by the
     * driver */
    error = max11261_adc_set_channel(channel);
    if (error < 0) {
        printf("Failed to set ADC channel to %d: %d\n", channel, error);
        return -1;
    }

    // Uncomment to use register poll mode
    //max11261_adc_set_ready_func(NULL);
    printf("\n\n\n\n\n\n\n");
    sampleCount = 0;
    while (1) {
        /* Handle channel switch GPIO */
        if (flags & FLAG_CHANNEL_PRESSED) {
            channel = (channel + 1) % MAX11261_ADC_CHANNEL_MAX;
            max11261_adc_set_channel(channel);
            max11261_adc_convert_prepare();
            flags &= ~FLAG_CHANNEL_PRESSED;
        }

        /* Handle speed switch GPIO */
        if (flags & FLAG_RATE_PRESSED) {
            rate = (rate + 1) % MAX11261_SINGLE_RATE_MAX;
            max11261_adc_set_rate_single(rate);
            sampleCount = 0;
            flags &= ~FLAG_RATE_PRESSED;
        }

        tickStart = ticksUs;
        if (max11261_adc_convert() < 0) {
            printf("Failed to start conversion\n");
            return -1;
        }
        error = max11261_adc_result(&adcRes, MAX11261_ADC_CHANNEL_MAX);

        if (error > 0) {
            /* Print in reasonable intervals since UART output cannot catch
             * up to high sample rates */
            sampleCount++;
            if (sampleCount == 1 + rate * 10) {
                sampleCount = 0;
                print_results(&adcRes, error, ticksUs - tickStart);
            }
        } else {
            printf("Error obtaining result: %d (%u us)\n", error, ticksUs - tickStart);
            return -1;
        }
        fflush(stdout);
    }

    return 0;
}

void print_results(const max11261_adc_result_t *res, int count, uint32_t ticks)
{
    int j;
    printf("\033[%dA", count);
    for (j = 0; j < count; j++) {
        printf("  CH%u:   %5d%s%s%s mV in %u us    \n", res->chn, res->val, (res->dor ? "*" : ""),
               (res->aor ? "[!]" : ""), (res->oor ? "[<>]" : ""), ticks / count);
        res++;
    }
}

static void pb_irq_handler(void *pb)
{
    if (pb == (void *)PB_0) {
        flags |= FLAG_CHANNEL_PRESSED;
    } else if (pb == (void *)PB_1) {
        flags |= FLAG_RATE_PRESSED;
    }
}

void sys_timer_handler(void)
{
    MXC_TMR_ClearFlags(MXC_TMR0);
    ticksUs++;
}
