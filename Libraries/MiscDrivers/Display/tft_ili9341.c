/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "mxc_device.h"
#include "mxc_delay.h"

#include "tft_ili9341.h"
#include "spi.h"
#include "gpio.h"

/*
Adafruit 2.4" TFT Wing uses ILI9341 240x320 TFT LCD controller
It supports 4-wire (TFT_CS/TFT_DC/SCLK/MISO/MOSI) 8-bit data serial interface only (IM0=0, IM1=IM2=IM3=1, Page 26 of ILI9341 datasheet)
*/

static mxc_spi_regs_t *spi;
static int ssel;
static mxc_gpio_cfg_t *reset_pin;
static mxc_gpio_cfg_t *blen_pin;

static tft_rotation_t tft_rotation = ROTATE_0;
static unsigned int g_foreground_color;
static unsigned int g_background_color;
static unsigned char *g_font;

/********************************* Static Functions **************************/
#if 0
static void spi_transmit(void* datain, unsigned int count)
{
    mxc_spi_req_t request = {
        spi,    // spi
        ssel,       // ssIdx
        1,      // ssDeassert
        (uint8_t*) datain, // txData
        NULL,       // rxData
        count,      // txCnt
        0       // rxCnt
    };

    MXC_SPI_MasterTransaction(&request);
}
#else
static void spi_transmit(void *datain, unsigned int count)
{
    unsigned int offset;
    unsigned int fifo;
    volatile uint16_t *u16ptrin = (volatile uint16_t *)datain;
    unsigned int start = 0;

    // HW requires disabling/renabling SPI block at end of each transaction (when SS is inactive).
    spi->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);

    // Setup the slave select
    MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_CTRL0_SS_ACTIVE,
                 ((1 << ssel) << MXC_F_SPI_CTRL0_SS_ACTIVE_POS));

    // number of RX Char is 0xffff
    spi->ctrl1 &= ~(MXC_F_SPI_CTRL1_RX_NUM_CHAR);

    //DMA RX FIFO disabled
    spi->dma &= ~(MXC_F_SPI_DMA_RX_FIFO_EN);

    // set number of char to be transmit
    MXC_SETFIELD(spi->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR, count << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    // DMA TX fifo enable
    spi->dma |= MXC_F_SPI_DMA_TX_FIFO_EN;

    /* Clear TX and RX FIFO in DMA
        TX: Set this bit to clear the TX FIFO and all TX FIFO flags in the QSPIn_INT_FL register.
            Note: The TX FIFO should be disabled (QSPIn_DMA.tx_fifo_en = 0) prior to setting this field.
            Note: Setting this field to 0 has no effect.
        RX: Clear the RX FIFO and any pending RX FIFO flags in QSPIn_INTFL.
            This should be done when the RX FIFO is inactive.
    */
    spi->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);
    // QSPIn port is enabled
    spi->ctrl0 |= (MXC_F_SPI_CTRL0_EN);

    // Clear master done flag
    spi->intfl = MXC_F_SPI_INTFL_MST_DONE;

    /* Loop until all data is transmitted */
    offset = 0;

    do {
        fifo = (count > 8) ? 8 : count;
        count -= fifo;

        while (fifo > 0) {
            /* Send data */
            spi->fifo16[0] = u16ptrin[offset];
            offset++;
            fifo--;
        }

        /*
            Master Start Data Transmission
                Set this field to 1 to start a SPI master mode transaction.
                0: No master mode transaction active.
                1: Master initiates a data transmission. Ensure that all pending transactions are
                complete before setting this field to 1.
                Note: This field is only used when the QSPIn is configured for Master Mode
                (QSPIn_CTRL0.master = 1).
        */
        if (start == 0) {
            spi->ctrl0 |= MXC_F_SPI_CTRL0_START;
            start = 1;
        }

        /* Wait for data transmitting complete and then Deasserts nSS I/O */
        // Deassert slave select at the end of the transaction
        spi->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_CTRL;
    } while (count);

    while (!(spi->intfl & MXC_F_SPI_INTFL_MST_DONE)) {
        // wait until done
    }

    return;
}
#endif

static void write_command(unsigned char command)
{
    unsigned char val = command;
    MXC_GPIO_OutClr(MXC_GPIO0, MXC_GPIO_PIN_8);
    spi_transmit(&val, 1);
    MXC_GPIO_OutSet(MXC_GPIO0, MXC_GPIO_PIN_8);
}

static void write_data(unsigned char data)
{
    unsigned char val = data;

    MXC_GPIO_OutSet(MXC_GPIO0, MXC_GPIO_PIN_8);
    spi_transmit(&val, 1);
}

static void window(unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
    write_command(0x2A);
    write_data(x >> 8);
    write_data(x);
    write_data((x + w - 1) >> 8);
    write_data(x + w - 1);

    write_command(0x2B);
    write_data(y >> 8);
    write_data(y);
    write_data((y + h - 1) >> 8);
    write_data(y + h - 1);
}

static void pixel(int x, int y, int color)
{
    write_command(0x2A);
    write_data(x >> 8);
    write_data(x);
    write_command(0x2B);
    write_data(y >> 8);
    write_data(y);
    write_command(0x2C); // send pixel
    write_data(color >> 8);
    write_data(color & 0xff);
}

static int width()
{
    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180)
        return 240;
    else
        return 320;
}

static int height()
{
    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180)
        return 320;
    else
        return 240;
}

static void WindowMax(void)
{
    window(0, 0, width(), height());
}

static void hline(int x0, int x1, int y, int color)
{
    int w;
    int j;

    w = x1 - x0 + 1;
    window(x0, y, w, 1);
    write_command(0x2C); // send pixel

    for (j = 0; j < w; j++) {
        write_data(color >> 8);
        write_data(color & 0xff);
    }

    WindowMax();
    return;
}

static void vline(int x, int y0, int y1, int color)
{
    int h;

    h = y1 - y0 + 1;
    window(x, y0, 1, h);
    write_command(0x2C); // send pixel

    for (int y = 0; y < h; y++) {
        write_data(color >> 8);
        write_data(color & 0xff);
    }

    WindowMax();
    return;
}

static void fillrect(int x0, int y0, int x1, int y1, int color)
{
    int h = y1 - y0 + 1;
    int w = x1 - x0 + 1;
    int pixel = h * w;

    window(x0, y0, w, h);

    write_command(0x2C); // send pixel

    for (int p = 0; p < pixel; p++) {
        write_data(color >> 8);
        write_data(color & 0xff);
    }

    WindowMax();
    return;
}

unsigned int char_x;
unsigned int char_y;

void locate(int x, int y)
{
    char_x = x;
    char_y = y;
}

int columns()
{
    return width() / g_font[1];
}

int rows()
{
    return height() / g_font[2];
}

void tft_character(int x, int y, int c)
{
    unsigned int hor, vert, offset, bpl, j, i, b;
    unsigned char *sym;
    unsigned char z, w;

    if ((c < 31) || (c > 127))
        return; // test char range

    // read font parameter from start of array
    offset = g_font[0]; // bytes / char
    hor = g_font[1]; // get hor size of font
    vert = g_font[2]; // get vert size of font
    bpl = g_font[3]; // bytes per line

    if (char_x + hor > width()) {
        char_x = 0;
        char_y = char_y + vert;
        if (char_y >= height() - g_font[2]) {
            char_y = 0;
        }
    }

    window(char_x, char_y, hor, vert); // char box
    write_command(0x2C); // send pixel

    sym = &g_font[((c - 32) * offset) + 4]; // start of char bitmap
    w = sym[0]; // width of actual char
    for (j = 0; j < vert; j++) { //  vertical line
        for (i = 0; i < hor; i++) { //  horizontal line
            z = sym[bpl * i + ((j & 0xF8) >> 3) + 1];
            b = 1 << (j & 0x07);
            if ((z & b) == 0x00) {
                write_data(g_background_color >> 8);
                write_data(g_background_color & 0xff);
            } else {
                write_data(g_foreground_color >> 8);
                write_data(g_foreground_color & 0xff);
            }
        }
    }

    WindowMax();

    if ((w + 2) < hor) { // x offset to next char
        char_x += w + 2;
    } else {
        char_x += hor;
    }
}

void set_font(unsigned char *f)
{
    g_font = f;
}

void ShowImage(unsigned int x0, unsigned int y0, unsigned int w, unsigned int h,
               unsigned char *bitmap)
{
    unsigned int j, i;
    int padd;

    // the lines are padded to multiple of 4 bytes in a bitmap
    padd = -1;
    do {
        padd++;
    } while (2 * (w + padd) % 4 != 0);

    window(x0, y0, w, h);

    bitmap += ((h - 1) * (w + padd)) * 2;

    write_command(0x2C); // send pixel

    for (j = 0; j < h; j++) { //Lines
        for (i = 0; i < w; i++) { // one line
            write_data(*(bitmap + 1));
            write_data(*bitmap);
            bitmap += 2;
        }
        bitmap -= 4 * w;
        bitmap -= padd * 2;
    }

    WindowMax();
}

static void tft_spi_init(void)
{
    int master = 1;
    int quadMode = 0;
    int numSlaves = 2;
    int ssPol = 0;
    unsigned int tft_hz = TFT_SPI_FREQ;

    mxc_spi_pins_t tft_pins;

    tft_pins.clock = true;
    tft_pins.ss0 = (ssel == 0); ///< Slave select pin 0
    tft_pins.ss1 = (ssel == 1); ///< Slave select pin 1
    tft_pins.ss2 = (ssel == 2); ///< Slave select pin 2
    tft_pins.miso = true; ///< miso pin
    tft_pins.mosi = true; ///< mosi pin
    tft_pins.sdio2 = false; ///< SDIO2 pin
    tft_pins.sdio3 = false; ///< SDIO3 pin

    MXC_SPI_Init(spi, master, quadMode, numSlaves, ssPol, tft_hz, tft_pins);

    // Set  SPI0 pins to VDDIOH (3.3V) to be compatible with TFT display
    MXC_GPIO_SetVSSEL(MXC_GPIO0, MXC_GPIO_VSSEL_VDDIOH, TFT_SPI0_PINS);
    MXC_SPI_SetDataSize(spi, 8);
    MXC_SPI_SetWidth(spi, SPI_WIDTH_STANDARD);
}

static void displayInit(void)
{
    if (reset_pin) {
        // CLR Reset pin;
        MXC_GPIO_OutClr(reset_pin->port, reset_pin->mask);

        // at least 50 usec low
        MXC_Delay(50);

        // SET Reset pin;
        MXC_GPIO_OutSet(reset_pin->port, reset_pin->mask);

        // delay after reset 5 ms
        MXC_Delay(5000);
    }

    write_command(0x01); // SW reset
    MXC_Delay(5000); // 5ms
    write_command(0x28); // display off

    /* Start Initial Sequence */
    write_command(0xCF);
    write_data(0x00);
    write_data(0x83);
    write_data(0x30);

    write_command(0xED);
    write_data(0x64);
    write_data(0x03);
    write_data(0x12);
    write_data(0x81);

    write_command(0xE8);
    write_data(0x85);
    write_data(0x01);
    write_data(0x79);

    write_command(0xCB);
    write_data(0x39);
    write_data(0x2C);
    write_data(0x00);
    write_data(0x34);
    write_data(0x02);

    write_command(0xF7);
    write_data(0x20);

    write_command(0xEA);
    write_data(0x00);
    write_data(0x00);

    write_command(0xC0); // POWER_CONTROL_1
    write_data(0x26);

    write_command(0xC1); // POWER_CONTROL_2
    write_data(0x11);

    write_command(0xC5); // VCOM_CONTROL_1
    write_data(0x35);
    write_data(0x3E);

    write_command(0xC7); // VCOM_CONTROL_2
    write_data(0xBE);

    write_command(0x36); // MEMORY_ACCESS_CONTROL
    write_data(0x48);

    write_command(0x3A); // COLMOD_PIXEL_FORMAT_SET
    write_data(0x55); // 16 bit pixel

    write_command(0xB1); // Frame Rate
    write_data(0x00);
    write_data(0x1B);

    write_command(0xF2); // Gamma Function Disable
    write_data(0x08);

    write_command(0x26);
    write_data(0x01); // gamma set for curve

    write_command(0xE0); // positive gamma correction
    write_data(0x1F);
    write_data(0x1A);
    write_data(0x18);
    write_data(0x0A);
    write_data(0x0F);
    write_data(0x06);
    write_data(0x45);
    write_data(0x87);
    write_data(0x32);
    write_data(0x0A);
    write_data(0x07);
    write_data(0x02);
    write_data(0x07);
    write_data(0x05);
    write_data(0x00);

    write_command(0xE1); // negative gamma correction
    write_data(0x00);
    write_data(0x25);
    write_data(0x27);
    write_data(0x05);
    write_data(0x10);
    write_data(0x09);
    write_data(0x3A);
    write_data(0x78);
    write_data(0x4D);
    write_data(0x05);
    write_data(0x18);
    write_data(0x0D);
    write_data(0x38);
    write_data(0x3A);
    write_data(0x1F);

    WindowMax();

    //write_command(0x34);    // tearing effect off
    //write_command(0x35);    // tearing effect on

    write_command(0xB7); // entry mode
    write_data(0x07);

    write_command(0xB6); // display function control
    write_data(0x0A);
    write_data(0x82);
    write_data(0x27);
    write_data(0x00);

    write_command(0x11); // sleep out
    MXC_Delay(100000); //100ms

    write_command(0x29); // display on
    MXC_Delay(100000); //100ms
}

/******************************** Public Functions ***************************/
int MXC_TFT_Init(mxc_spi_regs_t *tft_spi, int ss_idx, mxc_gpio_cfg_t *reset_ctrl,
                 mxc_gpio_cfg_t *bl_ctrl)
{
    int result = E_NO_ERROR;

    spi = tft_spi;
    ssel = ss_idx;
    reset_pin = reset_ctrl;
    blen_pin = bl_ctrl;

    /* TFT DC signal */
    mxc_gpio_cfg_t tft_dc_pin = { MXC_GPIO0, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                  MXC_GPIO_VSSEL_VDDIOH };

    /*
     *      Configure GPIO Pins
     */
    if (reset_pin) {
        MXC_GPIO_Config(reset_pin);
    }

    if (blen_pin) {
        MXC_GPIO_Config(blen_pin);
    }

    // Configure SPI Pins
    tft_spi_init();

    // Configure DC pin (write command/data)
    MXC_GPIO_Config(&tft_dc_pin);

    // Turn off backlight
    MXC_TFT_Backlight(0);

    // Send commands to configure display
    displayInit();

    // Set background color
    MXC_TFT_SetBackGroundColor(BLACK);

    // Turn on backlight
    MXC_TFT_Backlight(1);

    return result;
}

void MXC_TFT_SetRotation(tft_rotation_t rotation)
{
    tft_rotation = rotation;
    write_command(0x36); // MEMORY_ACCESS_CONTROL
    switch (tft_rotation) {
    case 0:
        write_data(0x48);
        break;
    case 1:
        write_data(0x28);
        break;
    case 2:
        write_data(0x88);
        break;
    case 3:
        write_data(0xE8);
        break;
    default:
        write_data(0x48);
        break;
    }

    WindowMax();
}

void MXC_TFT_SetBackGroundColor(unsigned int color)
{
    __disable_irq();

    fillrect(0, 0, width() - 1, height() - 1, color);

    __enable_irq();

    // keep color
    g_background_color = color;

    return;
}

void MXC_TFT_SetForeGroundColor(unsigned int color)
{
    g_foreground_color = color;

    return;
}

int MXC_TFT_SetPalette(int img_id)
{
    return E_NOT_SUPPORTED;
}

void MXC_TFT_Backlight(int on)
{
    if (blen_pin) {
        if (on) {
            MXC_GPIO_OutSet(blen_pin->port, blen_pin->mask);
        } else {
            MXC_GPIO_OutClr(blen_pin->port, blen_pin->mask);
        }
    }
}

void MXC_TFT_ClearScreen(void)
{
    MXC_TFT_SetBackGroundColor(g_background_color);
}

void MXC_TFT_FillRect(area_t *area, int color)
{
    int x0, y0, x1, y1;

    __disable_irq();
    x0 = area->x;
    y0 = area->y;

    x1 = x0 + area->w;
    y1 = y0 + area->h;

    fillrect(x0, y0, x1, y1, color);
    __enable_irq();
}

void MXC_TFT_WritePixel(int pixelX, int pixelY, int width, int height, uint32_t color)
{
    int x1, y1;

    __disable_irq();
    x1 = pixelX + width;
    y1 = pixelY + height;

    fillrect(pixelX, pixelY, x1, y1, color);
    __enable_irq();
}

void MXC_TFT_ShowImage(int x0, int y0, int id)
{
    unsigned char *bitmap_ptr;
    unsigned int w, h;

    bitmap_ptr = (unsigned char *)id;
    w = *bitmap_ptr++;
    w |= ((unsigned int)*bitmap_ptr++) << 8;
    h = *bitmap_ptr++;
    h |= ((unsigned int)*bitmap_ptr++) << 8;

    ShowImage((unsigned int)x0, (unsigned int)y0, w, h, bitmap_ptr);
}

void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t *image, int width, int height)
{
    unsigned int j, i;

    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180)
        window(x0, y0, height, width);
    else
        window(x0, y0, width, height);

    write_command(0x2C); // send pixel

    for (j = 0; j < width; j++) { //Lines
        for (i = 0; i < width * height; i += width) { // one line
            write_data(*(image + ((i + j) * 2)));
            write_data(*(image + ((i + j) * 2) + 1));
        }
    }

    WindowMax();
}

/* This function writes an image data line by line, required by most UI libraries like LVGL */
void MXC_TFT_WriteBufferRGB565(int x0, int y0, uint8_t *image, int width, int height)
{
    unsigned int x, y;

    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180) {
        window(x0, y0, height, width);
    } else {
        window(x0, y0, width, height);
    }

    write_command(0x2C); // send pixel

    for (y = 0; y < width * height; y += width) { //height
        for (x = 0; x < width; x++) { //width
            //Byteswap, 16 bit transfer */
            write_data(*(image + ((x + y) * 2) + 1));
            write_data(*(image + ((x + y) * 2)));
        }
    }

    WindowMax();
}

void MXC_TFT_PrintPalette(void)
{
    int i;
    area_t area = { 10, 10, 2, 25 };

    for (i = 0; i < 256; i++) {
        area.x += 4;
        MXC_TFT_FillRect(&area, i);

        if ((i & 63) == 63) {
            area.y += 25;
            area.x = 10;
        }
    }
}

void MXC_TFT_Circle(int x0, int y0, int r, int color)
{
    int x = -r, y = 0, err = 2 - 2 * r, e2;
    do {
        pixel(x0 - x, y0 + y, color);
        pixel(x0 + x, y0 + y, color);
        pixel(x0 + x, y0 - y, color);
        pixel(x0 - x, y0 - y, color);
        e2 = err;
        if (e2 <= y) {
            err += ++y * 2 + 1;
            if (-x == y && e2 <= x)
                e2 = 0;
        }
        if (e2 > x)
            err += ++x * 2 + 1;
    } while (x <= 0);
}

void MXC_TFT_FillCircle(int x0, int y0, int r, int color)
{
    int x = -r, y = 0, err = 2 - 2 * r, e2;
    do {
        vline(x0 - x, y0 - y, y0 + y, color);
        vline(x0 + x, y0 - y, y0 + y, color);
        e2 = err;
        if (e2 <= y) {
            err += ++y * 2 + 1;
            if (-x == y && e2 <= x)
                e2 = 0;
        }
        if (e2 > x)
            err += ++x * 2 + 1;
    } while (x <= 0);
}

void MXC_TFT_Line(int x0, int y0, int x1, int y1, int color)
{
    //WindowMax();
    int dx = 0, dy = 0;
    int dx_sym = 0, dy_sym = 0;
    int dx_x2 = 0, dy_x2 = 0;
    int di = 0;

    dx = x1 - x0;
    dy = y1 - y0;

    if (dx == 0) { /* vertical line */
        if (y1 > y0)
            vline(x0, y0, y1, color);
        else
            vline(x0, y1, y0, color);
        return;
    }

    if (dx > 0) {
        dx_sym = 1;
    } else {
        dx_sym = -1;
    }
    if (dy == 0) { /* horizontal line */
        if (x1 > x0)
            hline(x0, x1, y0, color);
        else
            hline(x1, x0, y0, color);
        return;
    }

    if (dy > 0) {
        dy_sym = 1;
    } else {
        dy_sym = -1;
    }

    dx = dx_sym * dx;
    dy = dy_sym * dy;

    dx_x2 = dx * 2;
    dy_x2 = dy * 2;

    if (dx >= dy) {
        di = dy_x2 - dx;
        while (x0 != x1) {
            pixel(x0, y0, color);
            x0 += dx_sym;
            if (di < 0) {
                di += dy_x2;
            } else {
                di += dy_x2 - dx_x2;
                y0 += dy_sym;
            }
        }
        pixel(x0, y0, color);
    } else {
        di = dx_x2 - dy;
        while (y0 != y1) {
            pixel(x0, y0, color);
            y0 += dy_sym;
            if (di < 0) {
                di += dx_x2;
            } else {
                di += dx_x2 - dy_x2;
                x0 += dx_sym;
            }
        }
        pixel(x0, y0, color);
    }
    return;
}

void MXC_TFT_Rectangle(int x0, int y0, int x1, int y1, int color)
{
    if (x1 > x0)
        hline(x0, x1, y0, color);
    else
        hline(x1, x0, y0, color);

    if (y1 > y0)
        vline(x0, y0, y1, color);
    else
        vline(x0, y1, y0, color);

    if (x1 > x0)
        hline(x0, x1, y1, color);
    else
        hline(x1, x0, y1, color);

    if (y1 > y0)
        vline(x1, y0, y1, color);
    else
        vline(x1, y1, y0, color);

    return;
}

/***************************************************************
 *          Printf Functions
 ***************************************************************/
void MXC_TFT_ResetCursor(void)
{
    return;
}

void MXC_TFT_SetFont(int font_id)
{
    if (font_id != 0)
        set_font((unsigned char *)font_id);
}

void MXC_TFT_Printf(const char *format, ...)
{
    return;
}

void MXC_TFT_ConfigPrintf(area_t *area)
{
    return;
}

void MXC_TFT_PrintFont(int x0, int y0, int id, text_t *str, area_t *area)
{
    int i;
    char value;

    if (id != 0)
        MXC_TFT_SetFont(id);

    locate(x0, y0);

    for (i = 0; i < str->len; i++) {
        value = str->data[i];

        if (value == '\n') { // new line
            char_x = 0;
            char_y = char_y + g_font[2];
            if (char_y >= height() - g_font[2]) {
                char_y = 0;
            }
        } else {
            tft_character(char_x, char_y, value);
        }
    }
}

void MXC_TFT_Print(int x0, int y0, text_t *str, area_t *area)
{
    MXC_TFT_PrintFont(x0, y0, (int)g_font, str, area);
}

void MXC_TFT_ClearArea(area_t *area, int color)
{
    MXC_TFT_FillRect(area, color);
}

void MXC_TFT_WriteReg(unsigned char command, unsigned char data)
{
    write_command(command);
    write_data(data);
}

void MXC_TFT_Stream(int x0, int y0, int width, int height)
{
    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180)
        window(x0, y0, height, width);
    else
        window(x0, y0, width, height);
    write_command(0x2C); // send pixel
}
