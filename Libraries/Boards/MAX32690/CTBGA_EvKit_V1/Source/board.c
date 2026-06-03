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
#include <string.h>
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
    { MXC_GPIO2, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
    { MXC_GPIO3, MXC_GPIO_PIN_9,  MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
    { MXC_GPIO3, MXC_GPIO_PIN_8,  MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0},
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_16, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

#define TFT_SPI_INST MXC_SPI1
static uint8_t tft_spi_buff[(1 + LINEBUF_SIZE) * 2];                             
static mxc_spi_req_t tft_spi_req = {TFT_SPI_INST, 	// instance
	                                1,				// ssIdx
	                                1,				// ssDeassert
	                                tft_spi_buff,	// txData
	                                NULL,			// rxData
	                                0,				// txLen
	                                0,				// rxLen
	                                0,				// txCnt
	                                0,				// rxCnt
	                                NULL,			// callback
	                                0xFF};			// dummy value


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
	int error;

    mxc_spi_pins_t tft_pins;

    memset(&tft_pins, 0, sizeof(mxc_spi_pins_t));
    tft_pins.ss1 = true;
    tft_pins.vddioh = true;
    tft_pins.clock = true;
    tft_pins.mosi = true;

    error = MXC_SPI_Init(TFT_SPI_INST, MXC_SPI_TYPE_CONTROLLER, MXC_SPI_INTERFACE_STANDARD, 1, 0, 1000000, tft_pins);
    MXC_ASSERT(error == E_NO_ERROR);

    error = MXC_SPI_SetFrameSize(TFT_SPI_INST, 9);
    MXC_ASSERT(error == E_NO_ERROR);
}

/******************************************************************************/
void TFT_SPI_Write(uint8_t *datain, uint32_t count, bool data)
{
 	uint8_t *bptr;
    unsigned int txlen;
    int error;

    MXC_ASSERT(!((count > 0) && (datain == NULL)) || (count > (1 + LINEBUF_SIZE)));

    bptr = tft_spi_buff;
    txlen = 0;

    /* Each byte sent to the TFT gets a 9th bit added that indicates data or command. 
       Copy each byte passed in into the outgoing array followed by a byte that holds
       the data/cmd bit.  */
    while (count--) {
        *bptr++ = *datain++;
                if (data) {
            *bptr++ = 1; /* DATA */
                } else {
            *bptr++ = 0; /* CMD */
        }
        txlen++;
    }

    /* Set the appropriate fields in the static request object. */
    tft_spi_req.txLen = txlen;
    tft_spi_req.txCnt = 0;

    /* Do the transaction. */
    error = MXC_SPI_ControllerTransaction(&tft_spi_req);
    MXC_ASSERT(error == E_NO_ERROR);
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
