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
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"
#include "lpgcr_regs.h"
// #include "simo_regs.h"  // AI87-TODO: ? - Look below

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO2, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

// The following pins are pulled up to 3V3 via external resistors on the AI87 EVKIT,
// and therefore must be initialized to VDDIOH to prevent current injection into VDDIO/VDDA
// in standby, backup, and powerdown modes.
const uint32_t _port0_vddioh_mask = MXC_GPIO_PIN_0;
const uint32_t _port1_vddioh_mask = (MXC_GPIO_PIN_10 | MXC_GPIO_PIN_11 | MXC_GPIO_PIN_12 |
                                     MXC_GPIO_PIN_13 | MXC_GPIO_PIN_14 | MXC_GPIO_PIN_16);
const uint32_t _port2_vddioh_mask = MXC_GPIO_PIN_2;

// TFT Data/Command pin
const mxc_gpio_cfg_t tft_dc_pin = { TFT_DC_PORT, TFT_DC_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                    MXC_GPIO_VSSEL_VDDIOH };
// TFT Slave Select pin
const mxc_gpio_cfg_t tft_ss_pin = { TFT_SS_PORT, TFT_SS_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                    MXC_GPIO_VSSEL_VDDIOH };
// TS IRQ pin
mxc_gpio_cfg_t ts_irq_pin = { TS_IRQ_PORT, TS_IRQ_PIN, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_NONE,
                              MXC_GPIO_VSSEL_VDDIOH };
// TS SS pin
const mxc_gpio_cfg_t ts_ss_pin = { TS_SS_PORT, TS_SS_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                   MXC_GPIO_VSSEL_VDDIOH };

/***** File Scope Variables *****/
// const uart_cfg_t uart_cfg = {
//     UART_PARITY_DISABLE,
//     UART_DATA_SIZE_8_BITS,
//     UART_STOP_1,
//     UART_FLOW_CTRL_DIS,
//     UART_FLOW_POL_DIS,
//     CONSOLE_BAUD
// };

// const sys_cfg_uart_t uart_sys_cfg = {MAP_A,Enable};    // There is no special system configuration parameters for UART on MAX32650
// const sys_cfg_i2c_t i2c_sys_cfg = NULL;     // There is no special system configuration parameters for I2C on MAX32650
// const sys_cfg_spixc_t spixc_sys_cfg = NULL;   // There is no special system configuration parameters for SPIXC on MAX32650

// const spixc_cfg_t mx25_spixc_cfg = {
//     0, //mode
//     0, //ssel_pol
//     1000000 //baud
// };

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1) {}
}

#ifdef TFT_NEWHAVEN
void TFT_SPI_Init(void)
{
    // Initialize SPI0
    int master = 1;
    int quadMode = 0;
    int numSlaves = 0;
    int ssPol = 0;
    int tft_hz = TFT_SPI_FREQ;
    mxc_spi_pins_t tft_pins = {
        // CLK, MISO, MOSI enabled, software controlled SS
        .clock = true, .ss0 = false, .ss1 = false,   .ss2 = false,
        .miso = true,  .mosi = true, .sdio2 = false, .sdio3 = false,
    };

    MXC_SPI_Init(TFT_SPI, master, quadMode, numSlaves, ssPol, tft_hz, tft_pins);

    // Set SPI pins to VDDIOH (3.3V) to be compatible with TFT display
    MXC_GPIO_SetVSSEL(TFT_SPI_PORT, MXC_GPIO_VSSEL_VDDIOH, TFT_SPI_PINS);
    MXC_SPI_SetDataSize(TFT_SPI, 8);
    MXC_SPI_SetWidth(TFT_SPI, SPI_WIDTH_STANDARD);

    // Initialize SPI GPIOs
    MXC_GPIO_Config(&tft_dc_pin); // Data/Command select
    MXC_GPIO_Config(&tft_ss_pin); // Slave Select
    MXC_GPIO_OutSet(tft_ss_pin.port, tft_ss_pin.mask);

    // If the touchscreen controller pin is unitialized, it will default
    // to logic level 0 (active slave select) and intercept TFT traffic.
    // Therefore it must be initialized and set to logic level 1 here.
    MXC_GPIO_Config(&ts_ss_pin);
    MXC_GPIO_OutSet(ts_ss_pin.port, ts_ss_pin.mask);
}

void TFT_SPI_Write(uint8_t data, bool cmd)
{
    // TFT and TS share the same SPI bus, and TS will set a lower SPI freq
    // So explicity set the TFT speed again before transmitting
    MXC_SPI_SetFrequency(TFT_SPI, TFT_SPI_FREQ);

    // Software controlled Slave Select
    MXC_GPIO_OutClr(TFT_SS_PORT, TFT_SS_PIN);

    if (cmd)
        MXC_GPIO_OutClr(TFT_DC_PORT, TFT_DC_PIN);
    else
        MXC_GPIO_OutSet(TFT_DC_PORT, TFT_DC_PIN);

    TFT_SPI->dma = MXC_F_SPI_DMA_TX_FIFO_EN;

    TFT_SPI->ctrl1 = 1 << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;

    *TFT_SPI->fifo8 = data;

    TFT_SPI->ctrl0 |= MXC_F_SPI_CTRL0_START;

    // MAX78002 Evaluation Kit is designed for firmware control of device select.
    // Wait here until done as caller will negate select on return, possibly before FIFO is empty.
    while (!(TFT_SPI->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}

    TFT_SPI->intfl = TFT_SPI->intfl;

    MXC_GPIO_OutSet(TFT_SS_PORT, TFT_SS_PIN);
}

void TFT_SPI_Transmit(void *src, int count)
{
    uint8_t *buf = (uint8_t *)src;

    // TFT and TS share the same SPI bus, and TS will set a lower SPI freq
    // So explicity set the TFT speed again before transmitting
    MXC_SPI_SetFrequency(TFT_SPI, TFT_SPI_FREQ);

    // Software controlled Slave Select
    MXC_GPIO_OutClr(TFT_SS_PORT, TFT_SS_PIN);

    MXC_GPIO_OutSet(TFT_DC_PORT, TFT_DC_PIN);

    TFT_SPI->dma = MXC_F_SPI_DMA_TX_FIFO_EN;

    TFT_SPI->ctrl1 = count << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;

    while (count--) {
        while ((TFT_SPI->dma & MXC_F_SPI_DMA_TX_LVL) ==
               (MXC_SPI_FIFO_DEPTH << MXC_F_SPI_DMA_TX_LVL_POS)) {}

        *TFT_SPI->fifo8 = *buf++;

        if (!(TFT_SPI->ctrl0 & MXC_F_SPI_CTRL0_START))
            TFT_SPI->ctrl0 |= MXC_F_SPI_CTRL0_START;
    }

    // MAX78002 Evaluation Kit is designed for firmware control of device select.
    // Wait here until done as caller will negate select on return, possibly before FIFO is empty.
    while (!(TFT_SPI->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}

    TFT_SPI->intfl = TFT_SPI->intfl;

    MXC_GPIO_OutSet(TFT_SS_PORT, TFT_SS_PIN);
}

void TS_SPI_Init(void)
{
    int master = 1;
    int quadMode = 0;
    int numSlaves = 0;
    int ssPol = 0;

    mxc_spi_pins_t ts_pins = {
        // CLK, MISO, MOSI enabled, software controlled SS
        .clock = true, .ss0 = false, .ss1 = false,   .ss2 = false,
        .miso = true,  .mosi = true, .sdio2 = false, .sdio3 = false,
    };

    MXC_SPI_Init(TS_SPI, master, quadMode, numSlaves, ssPol, TS_SPI_FREQ, ts_pins);

    // Set SPI pins to VDDIOH (3.3V) to be compatible with TFT display
    MXC_GPIO_SetVSSEL(MXC_GPIO0, MXC_GPIO_VSSEL_VDDIOH,
                      MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7);
    MXC_SPI_SetDataSize(TS_SPI, 8);
    MXC_SPI_SetWidth(TS_SPI, SPI_WIDTH_STANDARD);

    // Configure Slave Select and IRQ GPIOs
    MXC_GPIO_Config(&ts_irq_pin);
    MXC_GPIO_Config(&ts_ss_pin);
    MXC_GPIO_OutSet(ts_ss_pin.port, ts_ss_pin.mask);
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
    request.ssIdx = 0;

    MXC_SPI_SetFrequency(TS_SPI, TS_SPI_FREQ);
    MXC_SPI_SetDataSize(TS_SPI, 8);

    // Software controlled Slave Select
    MXC_GPIO_OutClr(ts_ss_pin.port, ts_ss_pin.mask);

    MXC_SPI_MasterTransaction(&request);

    // Wait to clear TS busy signal
    for (i = 0; i < 200; i++) {
        __asm volatile("nop\n");
    }

    request.ssDeassert = 1;
    request.txData = NULL;
    request.rxData = (uint8_t *)(rx);
    request.txLen = 0;
    request.rxLen = 2;

    MXC_SPI_MasterTransaction(&request);

    // MAX78002 Evaluation Kit is designed for firmware control of device select.
    // Wait here until done as caller will negate select on return, possibly before FIFO is empty.
    while (!(TS_SPI->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}

    TS_SPI->intfl = TS_SPI->intfl;

    if (dataout != NULL) {
        *dataout = (rx[1] | (rx[0] << 8)) >> 4;
    }

    MXC_GPIO_OutSet(ts_ss_pin.port, ts_ss_pin.mask);
}
#endif // TFT_NEWHAVEN

/******************************************************************************/
int Board_Init(void)
{
#ifndef __riscv
    int err;

    // Set SWDCLK and SWDIO pads to 3.3V
    // MXC_GPIO0->vssel |= (3 << 28);

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);

    // Set VSSEL to VDDIOH to for any pins that are pulled
    // up to 3V3 on the EVKIT.
    MXC_GPIO_SetVSSEL(MXC_GPIO0, MXC_GPIO_VSSEL_VDDIOH, _port0_vddioh_mask);
    MXC_GPIO_SetVSSEL(MXC_GPIO1, MXC_GPIO_VSSEL_VDDIOH, _port1_vddioh_mask);
    MXC_GPIO_SetVSSEL(MXC_GPIO2, MXC_GPIO_VSSEL_VDDIOH, _port2_vddioh_mask);

    if ((err = Console_Init()) < E_NO_ERROR) {
        return err;
    }

    // Set UART 0 pads to 3.3V
    MXC_GPIO0->vssel |= (0xF << 0);

    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

#ifdef TFT_NEWHAVEN
    // Set falling edge triggered interrupt for TouchScreen
    // MXC_TS_PreInit is not used because we need to take more direct
    // control of SPI routines and initialization
    ts_irq_pin.port->intmode |= ts_irq_pin.mask;
    ts_irq_pin.port->intpol &= ~(ts_irq_pin.mask);
    MXC_TS_AssignInterruptPin(ts_irq_pin);
#endif

// AI87-TODO: What's the reason for this deletion?
// MXC_SIMO->vrego_c = 0x43; // Set CNN voltage
#endif // __riscv

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
void NMI_Handler(void)
{
    __NOP();
}

#ifdef __riscv
/******************************************************************************/
int Debug_Init(void)
{
    // Set up RISCV JTAG pins (P1[0..3] AF2)
    MXC_GPIO1->en0_clr = 0x0f;
    MXC_GPIO1->en1_set = 0x0f;
    MXC_GPIO1->en2_clr = 0x0f;

    return E_NO_ERROR;
}
#endif // __riscv

/******************************************************************************/
int Camera_Power(int on)
{
    return E_NOT_SUPPORTED;
}
