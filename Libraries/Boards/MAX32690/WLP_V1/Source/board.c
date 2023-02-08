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

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {};
const unsigned int num_pbs = 0;

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t tft_mosi = (mxc_gpio_cfg_t){ .port = MXC_GPIO2,
                                                  .mask = MXC_GPIO_PIN_24,
                                                  .func = MXC_GPIO_FUNC_OUT,
                                                  .pad = MXC_GPIO_PAD_NONE,
                                                  .vssel = MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t tft_clk = (mxc_gpio_cfg_t){ .port = MXC_GPIO2,
                                                 .mask = MXC_GPIO_PIN_25,
                                                 .func = MXC_GPIO_FUNC_OUT,
                                                 .pad = MXC_GPIO_PAD_NONE,
                                                 .vssel = MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t tft_cs = (mxc_gpio_cfg_t){ .port = MXC_GPIO2,
                                                .mask = MXC_GPIO_PIN_11,
                                                .func = MXC_GPIO_FUNC_OUT,
                                                .pad = MXC_GPIO_PAD_NONE,
                                                .vssel = MXC_GPIO_VSSEL_VDDIOH };

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
#ifdef DEBUG
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
#endif

    while (1) {}
}

/******************************************************************************/
int Board_Init(void)
{
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
    MXC_Delay(10);

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

            for (int k = 0; k < 2; k++) {}

            tft_clk.port->out_set = tft_clk.mask; // Clk high

            for (int k = 0; k < 2; k++) {}
        }
    }

    MXC_Delay(10);
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
