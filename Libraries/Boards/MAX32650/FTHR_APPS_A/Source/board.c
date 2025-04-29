/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#include <errno.h>
#include <stdio.h>
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "i2c.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"

#include "max11261.h"

/* **** Definitions **** */
#define ADC_RST_ACTIVE_LOW 1 // Reset is active low

#define ADC_SLAVE_ADDR 0x30 // Depends on ADR0 and ADR1 pins

#define I2C_MASTER MXC_I2C1 // SDA P2_17; SCL P2_18
#define I2C_FREQ 400000 // 400kHz

/* **** Global Variables **** */
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;
extern uint8_t ChipRevision;

// clang-format off
const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO1, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO1, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO1, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO1, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

#ifdef ENABLE_MAX11261_ADC
/* **** Functions **** */
int MAX11261_Init(void);
#endif

/* ************************************************************************** */
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
    while (1) {}
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button interrupts in
 *       case the user does not define this interrupt handler in their application.
 **/
#if !defined(__ARMCC_VERSION) && !defined(__ICCARM__)
__weak
#endif
void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

/* ************************************************************************** */
int Board_Init(void)
{
    int err;

#ifdef ENABLE_MAX11261_ADC
    /* Setup I2C master */
    if ((err = MXC_I2C_Init(I2C_MASTER, 1, 0)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ)) < 0) {
        return err;
    }

    if ((err = MAX11261_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
#endif

    if ((err = Console_Init()) != E_NO_ERROR) {
        return err;
    }

    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

#if !defined(__ARMCC_VERSION) && !defined(__ICCARM__)
/* ************************************************************************** */
void NMI_Handler(void)
{
    __NOP();
}
#endif /* __GNUC__ */

#ifdef ENABLE_MAX11261_ADC
/* ************************************************************************** */
static int i2c1_transfer(uint8_t *txbuf, uint8_t txsize, uint8_t *rxbuf, uint8_t rxsize)
{
    mxc_i2c_req_t req;
    req.addr = ADC_SLAVE_ADDR;
    req.i2c = MXC_I2C1;
    req.restart = 0;
    req.rx_buf = rxbuf;
    req.rx_len = rxsize;
    req.tx_buf = txbuf;
    req.tx_len = txsize;

    return MXC_I2C_MasterTransaction(&req) == 0 ? 0 : -EIO;
}

static void max11261_reset_set(int ctrl)
{
    MXC_GPIO_OutPut(MXC_GPIO0, MXC_GPIO_PIN_16, (ctrl ^ ADC_RST_ACTIVE_LOW) << 16);
}

static int max11261_ready(void)
{
    return !MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_17);
}

static inline void delay_us(uint32_t us)
{
    MXC_Delay(us);
}

int MAX11261_Init(void)
{
    mxc_gpio_cfg_t adc_reset_n;
    mxc_gpio_cfg_t adc_int_n;

    /* Setup ADC reset GPIO */
    adc_reset_n.func = MXC_GPIO_FUNC_OUT;
    adc_reset_n.pad = MXC_GPIO_PAD_NONE;
    adc_reset_n.port = MXC_GPIO0;
    adc_reset_n.mask = MXC_GPIO_PIN_16;
    adc_reset_n.vssel = MXC_GPIO_VSSEL_VDDIO; /* 3V3 */
    adc_reset_n.drvstr = MXC_GPIO_DRVSTR_0; /* 3V3 */
    MXC_GPIO_Config(&adc_reset_n);

    /* Setup ready GPIO */
    adc_int_n.func = MXC_GPIO_FUNC_IN;
    adc_int_n.pad = MXC_GPIO_PAD_NONE;
    adc_int_n.port = MXC_GPIO0;
    adc_int_n.mask = MXC_GPIO_PIN_17;
    adc_int_n.vssel = MXC_GPIO_VSSEL_VDDIO; /* 3V3 */
    adc_int_n.drvstr = MXC_GPIO_DRVSTR_0; /* 3V3 */
    //MXC_GPIO_RegisterCallback(&gpioCfg, gpio_irq_handler, NULL);
    MXC_GPIO_Config(&adc_int_n);

    //MXC_GPIO_EnableInt(gpioCfg.port, gpioCfg.mask);
    //NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(ADC_INT_PORT));
    /* Initialize MAX11261 platform specific functions */
    max11261_adc_platform_init(i2c1_transfer, max11261_reset_set, delay_us);

    /* Use max11261_ready to check ADC status */
    max11261_adc_set_ready_func(max11261_ready);

    return E_NO_ERROR;
}
#endif

/******************************************************************************
 *
 *  These functions are defined multiple times in IAR and Keil startup files.
 *  Similar to the NMI and HardFault handlers, the GPIOn Handler functions
 *  that aren't used by the push button library will be defined here for the
 *  IAR and Keil.
 * 
 */
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

void GPIO2_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO2));
}

void GPIO3_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO3));
}
#endif
