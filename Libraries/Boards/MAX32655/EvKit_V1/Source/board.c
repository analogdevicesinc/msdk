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
#include <stdlib.h>
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
#include "tft_ssd2119.h"
#include "tsc2046.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

// clang-format off
const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_24, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO0, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    /* Logical LEDs for Bluetooth debug signals */
    { MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
    { MXC_GPIO2, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 }
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button/Touchscreen interrupt
 *       in case the user does not define this interrupt handler in their application.
 **/
#if !defined(__ARMCC_VERSION) && !defined(__ICCARM__)
__weak
#endif
void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

#ifndef __riscv /* RISCV does not have access to MXC_SPI0 */

/******************************************************************************/
static void ext_flash_board_init_quad(bool quadEnabled)
{
    mxc_gpio_cfg_t sdio23;

    sdio23.port = MXC_GPIO0;
    sdio23.mask = (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9);
    sdio23.pad = MXC_GPIO_PAD_NONE;
    sdio23.vssel = MXC_GPIO_VSSEL_VDDIOH;
    sdio23.drvstr = MXC_GPIO_DRVSTR_0;

    if (quadEnabled) {
        /* Enable these pins as SPI SDIO2/3*/
        sdio23.func = MXC_GPIO_FUNC_ALT1;
        MXC_GPIO_Config(&sdio23);

    } else {
        /* Control these pins as GPIO and set high when not using quad mode.
         * The W25 used on this board multiplexes the HOLD and WP functions on these
         * pins when not using quad mode
         */
        sdio23.func = MXC_GPIO_FUNC_OUT;
        MXC_GPIO_Config(&sdio23);
        MXC_GPIO_OutSet(sdio23.port, sdio23.mask);
    }
}

/******************************************************************************/
static int ext_flash_board_init(void)
{
    mxc_spi_pins_t qspi_flash_pins;
    int err = E_NO_ERROR;

    qspi_flash_pins.clock = true;
    qspi_flash_pins.ss0 = true;
    qspi_flash_pins.ss1 = true;
    qspi_flash_pins.ss2 = true;
    qspi_flash_pins.miso = true;
    qspi_flash_pins.mosi = true;
    qspi_flash_pins.vddioh = true;

    err = MXC_SPI_Init(MXC_SPI0, 1, 1, 1, 0, EXT_FLASH_BAUD, qspi_flash_pins);
    if (err != E_NO_ERROR) {
        return err;
    }

    MXC_SPI_SetDataSize(MXC_SPI0, 8);
    MXC_SPI_SetMode(MXC_SPI0, SPI_MODE_0);

    /* Leave the quad pins disabled, enable for quad transactions. */
    ext_flash_board_init_quad(false);

    return err;
}

/******************************************************************************/
static int ext_flash_board_read(uint8_t *read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spi_req_t qspi_read_req;
    mxc_spi_width_t spi_width;
    int err = E_NO_ERROR;

    switch (width) {
    case Ext_Flash_DataLine_Single:
        spi_width = SPI_WIDTH_STANDARD;
        break;
    case Ext_Flash_DataLine_Dual:
        spi_width = SPI_WIDTH_DUAL;
        break;
    case Ext_Flash_DataLine_Quad:
        spi_width = SPI_WIDTH_QUAD;
        ext_flash_board_init_quad(true);
        break;
    default:
        return E_BAD_PARAM;
    }

    MXC_SPI_SetWidth(MXC_SPI0, spi_width);

    qspi_read_req.spi = MXC_SPI0;
    qspi_read_req.ssIdx = 0;
    qspi_read_req.ssDeassert = deassert;
    qspi_read_req.txData = NULL;
    qspi_read_req.rxData = read;
    qspi_read_req.txLen = 0;
    qspi_read_req.rxLen = len;
    qspi_read_req.txCnt = 0;
    qspi_read_req.rxCnt = 0;
    qspi_read_req.completeCB = NULL;

    err = MXC_SPI_MasterTransaction(&qspi_read_req);
    if (err != E_NO_ERROR) {
        if (width == Ext_Flash_DataLine_Quad) {
            /* Restore the SPI config to disable quad pins */
            ext_flash_board_init_quad(false);
        }
        return err;
    }

    if (width == Ext_Flash_DataLine_Quad) {
        /* Restore the SPI config to disable quad pins */
        ext_flash_board_init_quad(false);
    }

    return err;
}

/******************************************************************************/
static int ext_flash_board_write(const uint8_t *write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spi_req_t qspi_write_req;
    mxc_spi_width_t spi_width;
    int err = E_NO_ERROR;

    switch (width) {
    case Ext_Flash_DataLine_Single:
        spi_width = SPI_WIDTH_STANDARD;
        break;
    case Ext_Flash_DataLine_Dual:
        spi_width = SPI_WIDTH_DUAL;
        break;
    case Ext_Flash_DataLine_Quad:
        spi_width = SPI_WIDTH_QUAD;
        ext_flash_board_init_quad(true);
        break;
    default:
        return E_BAD_PARAM;
    }

    MXC_SPI_SetWidth(MXC_SPI0, spi_width);

    qspi_write_req.spi = MXC_SPI0;
    qspi_write_req.ssIdx = 0;
    qspi_write_req.ssDeassert = deassert;
    qspi_write_req.txData = (uint8_t *)write;
    qspi_write_req.rxData = NULL;
    qspi_write_req.txLen = len;
    qspi_write_req.rxLen = 0;
    qspi_write_req.txCnt = 0;
    qspi_write_req.rxCnt = 0;
    qspi_write_req.completeCB = NULL;

    err = MXC_SPI_MasterTransaction(&qspi_write_req);
    if (err != E_NO_ERROR) {
        if (width == Ext_Flash_DataLine_Quad) {
            /* Restore the SPI config to disable quad pins */
            ext_flash_board_init_quad(false);
        }
        return err;
    }

    if (width == Ext_Flash_DataLine_Quad) {
        /* Restore the SPI config to disable quad pins */
        ext_flash_board_init_quad(false);
    }

    return err;
}

/******************************************************************************/
static int ext_flash_clock(unsigned int len, unsigned int deassert)
{
    mxc_spi_req_t qspi_dummy_req;
    mxc_spi_width_t width;
    uint8_t *write;
    int res;

    if (MXC_SPI_GetDataSize(MXC_SPI0) != 8) {
        return E_BAD_STATE;
    }

    width = MXC_SPI_GetWidth(MXC_SPI0);

    switch (width) {
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

    write = (uint8_t *)malloc(len);
    memset(write, 0, len);

    qspi_dummy_req.spi = MXC_SPI0;
    qspi_dummy_req.ssIdx = 0;
    qspi_dummy_req.ssDeassert = deassert;
    qspi_dummy_req.txData = write;
    qspi_dummy_req.rxData = NULL;
    qspi_dummy_req.txLen = len;
    qspi_dummy_req.rxLen = 0;
    qspi_dummy_req.txCnt = 0;
    qspi_dummy_req.rxCnt = 0;
    qspi_dummy_req.completeCB = NULL;

    res = MXC_SPI_MasterTransaction(&qspi_dummy_req);
    free(write);
    return res;
}

void TS_SPI_Init(void)
{
    mxc_spi_pins_t ts_pins = {
        // CLK, MISO, MOSI enabled, SS IDx = 1
        .clock = true, .ss0 = false, .ss1 = true,    .ss2 = false,
        .miso = true,  .mosi = true, .sdio2 = false, .sdio3 = false,
    };

    MXC_SPI_Init(TS_SPI, true, false, 1, 0, TS_SPI_FREQ, ts_pins);
    MXC_GPIO_SetVSSEL(MXC_GPIO0, MXC_GPIO_VSSEL_VDDIOH,
                      MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_26);
    MXC_SPI_SetDataSize(TS_SPI, 8);
    MXC_SPI_SetWidth(TS_SPI, SPI_WIDTH_STANDARD);
}

void TS_SPI_Transmit(uint8_t datain, uint16_t *dataout)
{
    int i;
    uint8_t rx[2] = { 0, 0 };
    mxc_spi_req_t request;

    request.spi = TS_SPI;
    request.ssDeassert = 0;
    request.txData = (uint8_t *)(&datain);
    request.rxData = NULL;
    request.txLen = 1;
    request.rxLen = 0;
    request.ssIdx = 1;

    MXC_SPI_SetFrequency(TS_SPI, TS_SPI_FREQ);
    MXC_SPI_SetDataSize(TS_SPI, 8);

    MXC_SPI_MasterTransaction(&request);

    // Wait to clear TS busy signal
    for (i = 0; i < 100; i++) {
        __asm volatile("nop\n");
    }

    request.ssDeassert = 1;
    request.txData = NULL;
    request.rxData = (uint8_t *)(rx);
    request.txLen = 0;
    request.rxLen = 2;

    MXC_SPI_MasterTransaction(&request);

    if (dataout != NULL) {
        *dataout = (rx[1] | (rx[0] << 8)) >> 4;
    }
}

#endif /* __riscv */

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

#ifndef __riscv /* RISCV does not have access to MXC_SPI0 */
    Ext_Flash_Config_t exf_cfg = { .init = ext_flash_board_init,
                                   .read = ext_flash_board_read,
                                   .write = ext_flash_board_write,
                                   .clock = ext_flash_clock };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }
#endif

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);

#ifdef DEBUG
    if ((err = Console_Init()) < E_NO_ERROR) {
        return err;
    }
#endif

    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

#ifndef __riscv
    /* TFT reset and backlight signal */
    mxc_tft_spi_config tft_spi_config = {
        .regs = MXC_SPI1,
        .gpio = { MXC_GPIO0, MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_20,
                  MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 },
        .freq = 12000000,
        .ss_idx = 0,
    };

    mxc_gpio_cfg_t tft_reset_pin = { MXC_GPIO3,         MXC_GPIO_PIN_0,        MXC_GPIO_FUNC_OUT,
                                     MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    mxc_gpio_cfg_t tft_bl_pin = { MXC_GPIO0,         MXC_GPIO_PIN_27,       MXC_GPIO_FUNC_OUT,
                                  MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

    /* Initialize TFT display */
    MXC_TFT_PreInit(&tft_spi_config, &tft_reset_pin, &tft_bl_pin);

    /* Enable Touchscreen */
    mxc_ts_spi_config ts_spi_config = {
        .regs = MXC_SPI1,
        .gpio = { MXC_GPIO0, MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_26,
                  MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 },
        .freq = 200000,
        .ss_idx = 1,
    };

    /* Touch screen controller interrupt signal */
    mxc_gpio_cfg_t int_pin = { MXC_GPIO0,         MXC_GPIO_PIN_13,       MXC_GPIO_FUNC_IN,
                               MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    /* Touch screen controller busy signal */
    mxc_gpio_cfg_t busy_pin = { MXC_GPIO0,         MXC_GPIO_PIN_12,       MXC_GPIO_FUNC_IN,
                                MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    /* Initialize Touch Screen controller */
    MXC_TS_PreInit(&ts_spi_config, &int_pin, &busy_pin);
#endif

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
void GPIO_PrepForSleep(void)
{
    /* Setup the unused GPIO high */
    mxc_gpio_cfg_t lpGpio;

    /* Set the wakeup pins high */
    lpGpio.port = MXC_GPIO3;
    lpGpio.mask = (0x3);
    lpGpio.func = MXC_GPIO_FUNC_OUT;
    lpGpio.pad = MXC_GPIO_PAD_NONE;
    lpGpio.vssel = MXC_GPIO_VSSEL_VDDIO;
    MXC_GPIO_Config(&lpGpio);
    MXC_GPIO_OutSet(MXC_GPIO3, 0x3);

    lpGpio.port = MXC_GPIO0;
    lpGpio.mask = (0xE7F3FFF0);
    lpGpio.func = MXC_GPIO_FUNC_OUT;
    lpGpio.pad = MXC_GPIO_PAD_NONE;
    lpGpio.vssel = MXC_GPIO_VSSEL_VDDIO;
    MXC_GPIO_Config(&lpGpio);
    MXC_GPIO_OutSet(MXC_GPIO0, 0xE7F3FFF0);
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

#if !defined(__ARMCC_VERSION) && !defined(__ICCARM__)
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
void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

void GPIO2_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO2));
}
#endif
