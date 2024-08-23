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

#include <stdio.h>
#include "board.h"
#include "Ext_Flash.h"
#include "gpio.h"
#include "led.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "pb.h"
#include "spixf.h"
#include "tft_st7735.h"
#include "uart.h"
#include "icc.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

// clang-format off
const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO4, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    /* Logical LEDs for Bluetooth debug signals */
    { MXC_GPIO0, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

const mxc_gpio_cfg_t tft_mosi = { .port = MXC_GPIO2,
                                  .mask = MXC_GPIO_PIN_24,
                                  .func = MXC_GPIO_FUNC_OUT,
                                  .pad = MXC_GPIO_PAD_NONE,
                                  .vssel = MXC_GPIO_VSSEL_VDDIOH,
                                  .drvstr = MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t tft_clk = { .port = MXC_GPIO2,
                                 .mask = MXC_GPIO_PIN_25,
                                 .func = MXC_GPIO_FUNC_OUT,
                                 .pad = MXC_GPIO_PAD_NONE,
                                 .vssel = MXC_GPIO_VSSEL_VDDIOH,
                                 .drvstr = MXC_GPIO_DRVSTR_0 };
const mxc_gpio_cfg_t tft_cs = { .port = MXC_GPIO2,
                                .mask = MXC_GPIO_PIN_11,
                                .func = MXC_GPIO_FUNC_OUT,
                                .pad = MXC_GPIO_PAD_NONE,
                                .vssel = MXC_GPIO_VSSEL_VDDIOH,
                                .drvstr = MXC_GPIO_DRVSTR_0 };

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
#ifdef DEBUG
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
#endif

    while (1) {}
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button interrupts in
 *       case the user does not define this interrupt handler in their application.
 **/
__weak void GPIOWAKE_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO4));
}

// Default handler for generic GPIO interrupts on port 2 (used by GPIO example)
__weak void GPIO2_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO2));
}

/******************************************************************************/
int Board_Init(void)
{
    //Disable to prevent cache bug
    MXC_ICC_Disable(MXC_ICC0);

    int err;

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO3);

    if ((err = Console_Init()) < E_NO_ERROR) {
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

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Shutdown(void)
{
    int err;

    if ((err = MXC_UART_Shutdown(ConsoleUart)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_PrepForSleep(void)
{
    return MXC_UART_ReadyForSleep(ConsoleUart);
}

/******************************************************************************/
void TFT_SPI_Init(void)
{
    // The TFT SPI on the MAX32690 EV Kit is bit-banged so there's no need to initialize
    // a SPI instance. Instead, we only need to configure the GPIO pins which are used to
    // connect to the SPI bus.

    // Configure GPIO pins
    MXC_GPIO_Config(&tft_mosi);
    MXC_GPIO_Config(&tft_clk);
    MXC_GPIO_Config(&tft_cs);

    // Set chip select and clock lines high (idle state)
    MXC_GPIO_OutSet(tft_cs.port, tft_cs.mask);
    MXC_GPIO_OutSet(tft_clk.port, tft_clk.mask);
}

/******************************************************************************/
void TFT_SPI_Write(uint8_t *datain, uint32_t count, bool data)
{
    // The TFT SPI on the MAX32690 EV Kit is not connected to a SPI instance so it
    // must be bit-banged.

    uint8_t tx_byte;

    if (datain == NULL) {
        return;
    }

    MXC_GPIO_OutClr(tft_cs.port, tft_cs.mask); // Assert chip select (active low)

    for (int i = 0; i < count; i++) {
        tx_byte = datain[i];

        for (int j = 0; j < 9; j++) { // 9 Bits per character
            tft_clk.port->out_clr = tft_clk.mask; //Clk low

            if (j == 0) { //Send D/CX bit before MSB
                if (data) {
                    tft_mosi.port->out_set = tft_mosi.mask;
                } else {
                    tft_mosi.port->out_clr = tft_mosi.mask;
                }
            } else {
                if (tx_byte & 0x80) { //Shift out data MSB first
                    tft_mosi.port->out_set = tft_mosi.mask;
                } else {
                    tft_mosi.port->out_clr = tft_mosi.mask;
                }
                tx_byte = tx_byte << 1;
            }

            for (volatile int k = 0; k < 2; k++) {}

            tft_clk.port->out_set = tft_clk.mask; // Clk high

            for (volatile int k = 0; k < 2; k++) {}
        }
    }

    MXC_GPIO_OutSet(tft_cs.port, tft_cs.mask); // De-assert chip select (active low)

    return;
}

/******************************************************************************/
__weak void NMI_Handler(void)
{
#ifdef DEBUG
    printf("NMI Handler\n");
#endif
    while (1) {}
}

/******************************************************************************/
__weak void HardFault_Handler(void)
{
#ifdef DEBUG
    printf("HardFault_Handler\n");
#endif
    while (1) {}
}
