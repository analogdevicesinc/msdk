/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2017-08-10 11:01:15 -0500 (Thu, 10 Aug 2017) $
 * $Revision: 29282 $
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "board.h"
#include "gpio.h"
#include "led.h"
#include "pb.h"
#include "uart.h"
#include "Ext_Flash.h"

/***** Global Variables *****/
mxc_uart_regs_t* ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    {MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO},
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    {MXC_GPIO0, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO0, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    /* Logical LEDs for Bluetooth debug signals */
    {MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO2, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO2, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO2, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO}
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/******************************************************************************/
static int ext_flash_board_init(void)
{
    int err = E_NO_ERROR;
    mxc_spi_pins_t qspi_flash_pins;

    qspi_flash_pins.clock   = true;
    qspi_flash_pins.ss0     = true;
    qspi_flash_pins.ss1     = false;
    qspi_flash_pins.ss2     = false;
    qspi_flash_pins.miso    = true;
    qspi_flash_pins.mosi    = true;
    qspi_flash_pins.sdio2   = true;
    qspi_flash_pins.sdio3   = true;
    qspi_flash_pins.vddioh  = true;

    err = MXC_SPI_Init(MXC_SPI0, 1, 1, 1, 0, EXT_FLASH_BAUD, qspi_flash_pins);
    if(err != E_NO_ERROR) {
        return err;
    }

    MXC_SPI_SetDataSize(MXC_SPI0, 8);
    MXC_SPI_SetMode(MXC_SPI0, SPI_MODE_0);

    return err;
}

/******************************************************************************/
static int ext_flash_board_read(uint8_t* read, unsigned len, unsigned deassert, Ext_Flash_DataLine_t width)
{
    mxc_spi_req_t qspi_read_req;
    mxc_spi_width_t spi_width;

    switch(width) {
        case Ext_Flash_DataLine_Single:
            spi_width = SPI_WIDTH_STANDARD;
            break;
        case Ext_Flash_DataLine_Dual:
            spi_width = SPI_WIDTH_DUAL;
            break;
        case Ext_Flash_DataLine_Quad:
            spi_width = SPI_WIDTH_QUAD;
            break;
        default:
            return E_BAD_PARAM;
    }

    MXC_SPI_SetWidth(MXC_SPI0, spi_width);

    qspi_read_req.spi        = MXC_SPI0;
    qspi_read_req.ssIdx      = 0;
    qspi_read_req.ssDeassert = deassert;
    qspi_read_req.txData     = NULL;
    qspi_read_req.rxData     = read;
    qspi_read_req.txLen      = 0;
    qspi_read_req.rxLen      = len;
    qspi_read_req.txCnt      = 0;
    qspi_read_req.rxCnt      = 0;
    qspi_read_req.completeCB = NULL;

    return MXC_SPI_MasterTransaction(&qspi_read_req);
}

/******************************************************************************/
static int ext_flash_board_write(const uint8_t* write, unsigned len, unsigned deassert, Ext_Flash_DataLine_t width)
{
    mxc_spi_req_t qspi_write_req;
    mxc_spi_width_t spi_width;

    switch(width) {
        case Ext_Flash_DataLine_Single:
            spi_width = SPI_WIDTH_STANDARD;
            break;
        case Ext_Flash_DataLine_Dual:
            spi_width = SPI_WIDTH_DUAL;
            break;
        case Ext_Flash_DataLine_Quad:
            spi_width = SPI_WIDTH_QUAD;
            break;
        default:
            return E_BAD_PARAM;
    }

    MXC_SPI_SetWidth(MXC_SPI0, spi_width);

    qspi_write_req.spi        = MXC_SPI0;
    qspi_write_req.ssIdx      = 0;
    qspi_write_req.ssDeassert = deassert;
    qspi_write_req.txData     = (uint8_t*) write;
    qspi_write_req.rxData     = NULL;
    qspi_write_req.txLen      = len;
    qspi_write_req.rxLen      = 0;
    qspi_write_req.txCnt      = 0;
    qspi_write_req.rxCnt      = 0;
    qspi_write_req.completeCB = NULL;

    return MXC_SPI_MasterTransaction(&qspi_write_req);
}

/******************************************************************************/
static int ext_flash_clock(unsigned len, unsigned deassert)
{
    mxc_spi_req_t qspi_dummy_req;
    mxc_spi_width_t width;

    if(MXC_SPI_GetDataSize(MXC_SPI0) != 8) {
        return E_BAD_STATE;
    }

    width = MXC_SPI_GetWidth(MXC_SPI0);

    switch(width) {
        case SPI_WIDTH_STANDARD:
            len /= 8; 
            break;
        case SPI_WIDTH_DUAL:
            len /= 4;
            break;
        case SPI_WIDTH_QUAD:
            len /= 2;
            break;
        default:
            return E_BAD_STATE;
    }

    uint8_t write[len];
    memset(write, 0, sizeof(write)); 

    qspi_dummy_req.spi        = MXC_SPI0;
    qspi_dummy_req.ssIdx      = 0;
    qspi_dummy_req.ssDeassert = deassert;
    qspi_dummy_req.txData     = write;
    qspi_dummy_req.rxData     = NULL;
    qspi_dummy_req.txLen      = len;
    qspi_dummy_req.rxLen      = 0;
    qspi_dummy_req.txCnt      = 0;
    qspi_dummy_req.rxCnt      = 0;
    qspi_dummy_req.completeCB = NULL;

    return MXC_SPI_MasterTransaction(&qspi_dummy_req);
}

/******************************************************************************/
void mxc_assert(const char* expr, const char* file, int line)
{
#ifdef DEBUG
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
#endif
    while (1);
}

/******************************************************************************/
int Board_Init(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = {
                            .init = ext_flash_board_init,
                            .read = ext_flash_board_read,
                            .write = ext_flash_board_write,
                            .clock = ext_flash_clock
                         };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);

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

#ifdef __riscv
/******************************************************************************/
int Debug_Init(void)
{
    // Set up RISCV JTAG pins (P1[0..3] AF2)
    MXC_GPIO_Config(&gpio_cfg_rv_jtag);
    
    return E_NO_ERROR;
}
#endif // __riscv

/******************************************************************************/
__weak void NMI_Handler(void)
{
    #ifdef DEBUG
    printf("NMI Handler\n");
    #endif
    while(1) {}
}

/******************************************************************************/
__weak void HardFault_Handler(void)
{
    #ifdef DEBUG
    printf("HandFault_Handler\n");
    #endif
    while(1) {}
}
