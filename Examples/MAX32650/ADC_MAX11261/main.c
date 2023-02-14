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
#define I2C_MASTER	MXC_I2C1	// SDA P2_17; SCL P2_18
#define I2C_FREQ	100			// 100kHz

#define ADC_V_AVDD      3000	// 3V
#define ADC_V_REF       2500	// 2.5V

#define ADC_SLAVE_ADDR  0x30	// Depends on ADR0 and ADR1 pins

#define ADC_RST_PORT	MXC_GPIO_PORT_0
#define ADC_RST_PIN  	16
#define ADC_INT_PORT	MXC_GPIO_PORT_0
#define ADC_INT_PIN		17

#define ADC_RST_ACTIVE_LOW  1 // Reset is active low

#define PB_MODE_SWITCH      0 // Use push button 0 for mode switching
#define PB_CHANNEL_SWITCH   1 // Use push button 1 for channel switching

/**
 * Event flags that will be handled in main loop
 */
#define FLAG_CHANNEL_CHANGED    (0x01UL << 0)
#define FLAG_RATE_CHANGED       (0x01UL << 1)
#define FLAG_POLARITY_CHANGED   (0x01UL << 2)
#define FLAG_FORMAT_CHANGED     (0x01UL << 3)
#define FLAG_MODE_PRESSED       (0x01UL << 8)
#define FLAG_CHANNEL_PRESSED    (0x01UL << 9)
#define FLAG_TMR_100MS          (0x01UL << 16)

/***** Globals *****/
static volatile uint32_t ticksUs = 0;
static volatile uint32_t flags = 0;

/***** Functions *****/
static void pb_irq_handler(void *pb);
static void sys_timer_handler(void);

static int i2c1_transfer(uint8_t *txbuf, uint8_t txsize, uint8_t *rxbuf,
        uint8_t rxsize, uint8_t slave)
{
    mxc_i2c_req_t req;
    req.addr = slave;
    req.i2c = I2C_MASTER;
    req.restart = 0;
    req.rx_buf = rxbuf;
    req.rx_len = rxsize;
    req.tx_buf = txbuf;
    req.tx_len = txsize;

    return MXC_I2C_MasterTransaction(&req) == 0 ? 0 : -EIO;
}

static void max11261_reset_set(int ctrl)
{
    MXC_GPIO_OutPut(MXC_GPIO_GET_GPIO(ADC_RST_PORT), 1 << ADC_RST_PIN,
            (ctrl ^ ADC_RST_ACTIVE_LOW) << ADC_RST_PIN);
}

static int max11261_ready()
{
    return !MXC_GPIO_InGet(MXC_GPIO_GET_GPIO(ADC_INT_PORT), 1 << ADC_INT_PIN);
}

static inline void delay_us(uint32_t us)
{
    MXC_Delay(us);
}

int main(void)
{
	int error;
	mxc_gpio_cfg_t gpioCfg;
	mxc_tmr_cfg_t tmrCfg;
	max11261_conversion_mode_t convMode = MAX11261_SINGLE_CYCLE;
	max11261_sequencer_mode_t seqMode = MAX11261_SEQ_MODE_1;
	max11261_adc_channel_t channel = MAX11261_ADC_CHANNEL_0;
	max11261_pol_t polarity = MAX11261_POL_UNIPOLAR;
	max11261_fmt_t format = MAX11261_FMT_OFFSET_BINARY;
	int32_t adcVal;
	uint32_t tickStart;

    printf("\n******************** ADC Example ********************\n\n");
    printf("Demonstrates various features of MAX11261 ADC.\n");
    printf("An input voltage between -Vref and +Vref can be applied to AIN "
    		"inputs. Conversion results for the input values outside this "
    		"range will be clipped to the minimum or maximum level\n");

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

    /* Setup reset GPIO */
	gpioCfg.func = MXC_GPIO_FUNC_OUT;
	gpioCfg.mask = 1 << ADC_RST_PIN;
	gpioCfg.pad = MXC_GPIO_PAD_NONE;
	gpioCfg.port = MXC_GPIO_GET_GPIO(ADC_RST_PORT);
	gpioCfg.vssel = MXC_GPIO_VSSEL_VDDIO; /* 3V3 */
	error = MXC_GPIO_Config(&gpioCfg);
	if (error != E_NO_ERROR) {
		printf("Failed to configure reset GPIO\n");
		return -1;
	}

	/* Setup ready GPIO */
	gpioCfg.func = MXC_GPIO_FUNC_IN;
	gpioCfg.mask = 1 << ADC_INT_PIN;
	gpioCfg.pad = MXC_GPIO_PAD_NONE;
	gpioCfg.port = MXC_GPIO_GET_GPIO(ADC_INT_PORT);;
	gpioCfg.vssel = MXC_GPIO_VSSEL_VDDIO; /* 3V3 */
	//MXC_GPIO_RegisterCallback(&gpioCfg, gpio_irq_handler, NULL);
	MXC_GPIO_IntConfig(&gpioCfg, MXC_GPIO_INT_FALLING);
	if (error != E_NO_ERROR) {
		printf("Failed to configure ready GPIO\n");
		return -1;
	}
	//MXC_GPIO_EnableInt(gpioCfg.port, gpioCfg.mask);
	//NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(ADC_INT_PORT));

	/* Enable push button interrupts */
	PB_IntEnable(PB_MODE_SWITCH);
	PB_IntEnable(PB_CHANNEL_SWITCH);
	PB_RegisterCallback(PB_MODE_SWITCH, pb_irq_handler);
	PB_RegisterCallback(PB_CHANNEL_SWITCH, pb_irq_handler);

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

	/* Initialize MAX11261 platform specific functions */
	max11261_adc_platform_init(i2c1_transfer, max11261_reset_set,
	            delay_us);

	/* Set ADC hardware parameters */
	error = max11261_adc_config_init(ADC_V_AVDD, ADC_V_REF, I2C_FREQ,
			ADC_SLAVE_ADDR);
	if (error != E_NO_ERROR) {
		printf("Failed to initialize MAX11261\n");
		return -1;
	}

	/* Use max11261_ready to check ADC status */
	max11261_adc_set_ready_func(max11261_ready);

	/* Reset ADC */
	max11261_adc_reset();
	max11261_adc_standby();

	/* Set ADC sequencer parameters. Default values are already set by the
	 * driver */
	error = max11261_adc_set_channel(channel);
	if (error < 0) {
		printf("Failed to set ADC channel to %d: %d\n", channel, error);
		return -1;
	}

	error = max11261_adc_set_mode(convMode, seqMode);
	if (error < 0) {
		printf("Failed to set conversion and sequencer modes: %d\n", error);
		return -1;
	}

	error = max11261_adc_set_polarity(polarity);
	if (error < 0) {
		printf("Failed to set ADC polarity: %d\n", error);
		return -1;
	}

	error = max11261_adc_set_format(format);
	if (error < 0) {
		printf("Failed to set ADC format: %d\n", error);
	}

	error = max11261_adc_convert_prepare();
	if (error < 0) {
		printf("Failed to prepare for conversion: %d\n", error);
		return -1;
	}

    while (1) {

        tickStart = ticksUs;
        switch (convMode) {
        case MAX11261_LATENT_CONTINUOUS:
        	break;
        case MAX11261_SINGLE_CYCLE:
        	if (max11261_adc_convert() < 0) {
        		printf("Failed to start conversion\n");
        		return -1;
        	}

        	error = max11261_adc_result(&adcVal);
        	if (error == 0) {
        		printf("\r     %5d mV at %u us", adcVal, ticksUs - tickStart);
        	} else {
        		printf("Error obtaining result: %d\n", error);
        		return -1;
        	}
        	break;
        case MAX11261_SINGLE_CYCLE_CONTINUOUS:
        	break;
        }
        fflush(stdout);
    }

    return 0;
}

static void pb_irq_handler(void *pb)
{
    if (pb == (void *) PB_CHANNEL_SWITCH) {
        flags |= FLAG_CHANNEL_PRESSED;
    } else if (pb == (void *) PB_MODE_SWITCH) {
        flags |= FLAG_MODE_PRESSED;
    }
}

void sys_timer_handler(void)
{
    MXC_TMR_ClearFlags(MXC_TMR0);
    ticksUs++;
}
