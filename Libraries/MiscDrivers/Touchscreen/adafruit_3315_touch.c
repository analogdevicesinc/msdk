/*******************************************************************************
 * Copyright (C) 2021 Maxim Integrated Products, Inc., All Rights Reserved.
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

/*
  The Adafruit 2.4" TFT Wing includes a ILI9341 240x320 TFT LCD controller
  and a STMPE610 touchscreen controller.

  The SPI interface is shared by the TFT and touchscreen controllers.

  The touchscreen events generate an external interrupt.  The TFT driver
  disables interrupts to prevent the touchscreen driver from accessing the
  SPI bus during TFT activity.  Firmware control of the TFT Data/Command signal
  and the use of GPIO as device delect signals bars a interrupt SPI interface.

  The maximum SPI clock rate differs between the TFT and touchscreen controllers.
  The maximum TFT clock is 10MHz, the maximum touchscreen clock is 1MHz. The
  touchscreen driver set the clock rate to the touchscreen maximum, performs
  SPI operations and restores the TFT maximum clock rate before returning.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "mxc.h"
#include "adafruit_3315_touch.h"

#define TS_SPI_FREQ 1000000

#define IRQ_PORT MXC_GPIO0
#define IRQ_PIN MXC_GPIO_PIN_19

#define SS_PORT MXC_GPIO1
#define SS_PIN MXC_GPIO_PIN_1

#define WRITE_OP_FLAG 0x00
#define READ_OP_FLAG 0x80

#define CHIP_ID 0x00
#define SYS_CTRL1 0x03
#define SYS_CTRL2 0x04
#define INT_CTRL 0x09
#define INT_EN 0x0A
#define INT_STA 0x0B
#define ADC_CTRL1 0x20
#define ADC_CTRL2 0x21
#define TSC_CTRL 0x40
#define TSC_CFG 0x41
#define FIFO_TH 0x4A
#define FIFO_CTRL_STA 0x4B
#define TSC_DATA_AUTO_INC 0x57
#define TSC_I_DRIVE 0x58
#define TSC_DATA_NO_INC 0xD7

#define CHIP_ID_VAL 0x0811

typedef struct {
    int x0, y0;
    int x1, y1;
    int key_code;
} TS_Buttons_t;

static TS_Buttons_t ts_buttons[TS_MAX_BUTTONS];

volatile bool ts_event = false;
static mxc_spi_regs_t *spi;
static volatile uint8_t buf[3];

static uint8_t read_reg(uint8_t reg)
{
    MXC_SPI_SetFrequency(spi, TS_SPI_FREQ);

    MXC_GPIO_OutClr(SS_PORT, SS_PIN);

    spi->dma = MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_RX_FIFO_EN;

    spi->ctrl1 = (3 << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS) | (3 << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS);

    *spi->fifo8 = reg | READ_OP_FLAG;
    *spi->fifo8 = 0;
    *spi->fifo8 = 0;

    spi->ctrl0 |= MXC_F_SPI_CTRL0_START;

    while (!(spi->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}
    spi->intfl = spi->intfl;

    MXC_GPIO_OutSet(SS_PORT, SS_PIN);

    MXC_SPI_SetFrequency(spi, TFT_SPI_FREQ);

    // drain don't care bytes
    *spi->fifo8;
    *spi->fifo8;

    return *spi->fifo8;
}

static void write_reg(uint8_t addr, uint8_t val)
{
    MXC_SPI_SetFrequency(spi, TS_SPI_FREQ);

    MXC_GPIO_OutClr(SS_PORT, SS_PIN);

    spi->dma = MXC_F_SPI_DMA_TX_FIFO_EN;

    spi->ctrl1 = 2 << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;

    *spi->fifo8 = addr | WRITE_OP_FLAG;
    *spi->fifo8 = val;

    spi->ctrl0 |= MXC_F_SPI_CTRL0_START;

    while (!(spi->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}

    spi->intfl = spi->intfl;

    MXC_GPIO_OutSet(SS_PORT, SS_PIN);

    MXC_SPI_SetFrequency(spi, TFT_SPI_FREQ);
}

static void ts_handler(void *unused)
{
    (void)unused;

    while (!(read_reg(FIFO_CTRL_STA) & 0x20))
        for (uint32_t i = 0; i < 3; i++) buf[i] = read_reg(TSC_DATA_NO_INC);

    if (read_reg(FIFO_CTRL_STA) & 0x20)
        write_reg(INT_STA, 0xDF);

    ts_event = true;

    MXC_GPIO_ClearFlags(IRQ_PORT, IRQ_PIN);
}

static int isInBox(int x, int y, int x0, int y0, int x1, int y1)
{
    return ((x >= x0) && (x <= x1) && (y >= y0) && (y <= y1));
}

void MXC_TS_Start(void)
{
    write_reg(INT_CTRL, 0x03);
    MXC_GPIO_EnableInt(IRQ_PORT, IRQ_PIN);
}

void MXC_TS_Stop(void)
{
    MXC_GPIO_DisableInt(IRQ_PORT, IRQ_PIN);
    write_reg(INT_CTRL, 0x00);
}

int MXC_TS_AddButton(int x0, int y0, int x1, int y1, int key_code)
{
    int idx;

    for (idx = TS_MAX_BUTTONS - 1; idx >= 0; idx--) {
        if (ts_buttons[idx].key_code == TS_INVALID_KEY_CODE) {
            ts_buttons[idx].x0 = x0;
            ts_buttons[idx].y0 = y0;
            ts_buttons[idx].x1 = x1;
            ts_buttons[idx].y1 = y1;
            ts_buttons[idx].key_code = key_code;
            break;
        }
    }

    return idx;
}

void MXC_TS_RemoveButton(int x0, int y0, int x1, int y1)
{
    (void)x1;
    (void)y1;

    for (int i = 0; i < TS_MAX_BUTTONS; i++)
        if (ts_buttons[i].key_code != TS_INVALID_KEY_CODE)
            if (isInBox(x0, y0, ts_buttons[i].x0, ts_buttons[i].y0, ts_buttons[i].x1,
                        ts_buttons[i].y1))
                ts_buttons[i].key_code = TS_INVALID_KEY_CODE;
}

void MXC_TS_RemoveAllButton(void)
{
    for (int i = 0; i < TS_MAX_BUTTONS; i++) ts_buttons[i].key_code = TS_INVALID_KEY_CODE;
}

int MXC_TS_GetKey(void)
{
    uint16_t x, y;

    if (!ts_event)
        return 0;

    ts_event = false;

    MXC_TS_GetTouch(&x, &y);

    for (int i = 0; i < TS_MAX_BUTTONS; i++)
        if (ts_buttons[i].key_code != TS_INVALID_KEY_CODE)
            if (isInBox(x, y, ts_buttons[i].x0, ts_buttons[i].y0, ts_buttons[i].x1,
                        ts_buttons[i].y1))
                return ts_buttons[i].key_code;

    return 0;
}

void MXC_TS_GetTouch(uint16_t *x, uint16_t *y)
{
    *x = (buf[0] << 4) | (buf[1] >> 4);
    *y = ((buf[1] << 8) | buf[2]) & 0x0FFF;
}

int MXC_TS_Init(mxc_spi_regs_t *ts_spi, int ss_idx, mxc_gpio_cfg_t *int_pin,
                mxc_gpio_cfg_t *busy_pin)
{
    (void)ss_idx;
    (void)int_pin;
    (void)busy_pin;

    static const mxc_gpio_cfg_t ts_irq_pin = { IRQ_PORT, IRQ_PIN, MXC_GPIO_FUNC_IN,
                                               MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH };
    static const mxc_gpio_cfg_t ts_ss_pin = { SS_PORT, SS_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                              MXC_GPIO_VSSEL_VDDIOH };
    uint16_t id;

    spi = ts_spi;

    MXC_GPIO_Config(&ts_ss_pin);
    MXC_GPIO_OutSet(SS_PORT, SS_PIN);

    MXC_GPIO_Config(&ts_irq_pin);
    MXC_GPIO_RegisterCallback(&ts_irq_pin, ts_handler, NULL);
    MXC_GPIO_IntConfig(&ts_irq_pin, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(IRQ_PORT, IRQ_PIN);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(IRQ_PORT)));

    write_reg(SYS_CTRL1, 0x02); // soft reset
    MXC_Delay(MXC_DELAY_MSEC(10));

    id = read_reg(CHIP_ID) << 8;
    id |= read_reg(CHIP_ID + 1);
    if (id != CHIP_ID_VAL)
        return E_NO_DEVICE;

    static const struct {
        uint8_t cmd;
        uint8_t arg;
    } * pcmd, cmds[] = {
        { SYS_CTRL2, 0x00 }, // enable clocks
        { TSC_CTRL, 0x03 }, // no tracking, X,Y acquisition, enable TSC
        { INT_EN, 0x02 }, // interrupt on FIFO >= threshold
        { ADC_CTRL1, 0x50 }, // internal ref, 10-bit resolution, 96 clock conversion
        { ADC_CTRL2, 0x02 }, // 6.5MHz ADC clock
        { TSC_CFG, 0xE4 }, // 8 sample avg, 1ms touch detect delay, 5ms settling time
        { FIFO_TH, 0x01 }, // FIFO interrupt threshold
        { FIFO_CTRL_STA, 0x01 }, // assert FIFO reset
        { FIFO_CTRL_STA, 0x00 }, // negate FIFO reset
        { TSC_I_DRIVE, 0x01 }, // 50mA max drive
        { INT_STA, 0xDF }, // clear interrupts
        { INT_CTRL, 0x03 }, // Falling edge interrupts, global enable
    };

    int ncmds = sizeof(cmds) / sizeof(*cmds);

    for (pcmd = cmds; ncmds; ncmds--, pcmd++) write_reg(pcmd->cmd, pcmd->arg);

    MXC_TS_RemoveAllButton();

    return E_NO_ERROR;
}
