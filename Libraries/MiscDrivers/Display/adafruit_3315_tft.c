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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "mxc.h"
#include "adafruit_3315_tft.h"

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

#define COLUMN_ADDR 0x2A
#define PAGE_ADDR 0x2B
#define MEM_WRITE 0x2C
#define MEM_ACCESS_CNTL 0x36

static mxc_spi_regs_t *spi;
static int ssel;
static mxc_gpio_cfg_t *reset_pin;
static mxc_gpio_cfg_t *blen_pin;

static tft_rotation_t tft_rotation = ROTATE_0;
static unsigned int g_foreground_color = WHITE;
static unsigned int g_background_color = BLACK;
static unsigned char *g_font;

static unsigned int char_x;
static unsigned int char_y;

static void spi_transmit(uint8_t data, bool cmd)
{
    if (cmd)
        MXC_GPIO_OutClr(TFT_DC_PORT, TFT_DC_PIN);
    else
        MXC_GPIO_OutSet(TFT_DC_PORT, TFT_DC_PIN);

    spi->dma = MXC_F_SPI_DMA_TX_FIFO_EN;

    spi->ctrl1 = 1 << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;

    *spi->fifo8 = data;

    spi->ctrl0 |= MXC_F_SPI_CTRL0_START;

    // MAX78002 Evaluation Kit is designed for firmware control of device select.
    // Wait here until done as caller will negate select on return, possibly before FIFO is empty.
    while (!(spi->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}

    spi->intfl = spi->intfl;
}

static void spi_transmit_data_buf(void *src, int count)
{
    uint8_t *buf = (uint8_t *)src;

    MXC_GPIO_OutClr(TFT_SS_PORT, TFT_SS_PIN);

    MXC_GPIO_OutSet(TFT_DC_PORT, TFT_DC_PIN);

    spi->dma = MXC_F_SPI_DMA_TX_FIFO_EN;

    spi->ctrl1 = count << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;

    while (count--) {
        while ((spi->dma & MXC_F_SPI_DMA_TX_LVL) ==
               (MXC_SPI_FIFO_DEPTH << MXC_F_SPI_DMA_TX_LVL_POS)) {}

        *spi->fifo8 = *buf++;

        if (!(spi->ctrl0 & MXC_F_SPI_CTRL0_START))
            spi->ctrl0 |= MXC_F_SPI_CTRL0_START;
    }

    // MAX78002 Evaluation Kit is designed for firmware control of device select.
    // Wait here until done as caller will negate select on return, possibly before FIFO is empty.
    while (!(spi->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}

    spi->intfl = spi->intfl;

    MXC_GPIO_OutSet(TFT_SS_PORT, TFT_SS_PIN);
}

static void write_command(unsigned char command)
{
    MXC_GPIO_OutClr(TFT_SS_PORT, TFT_SS_PIN);

    spi_transmit(command, true);

    MXC_GPIO_OutSet(TFT_SS_PORT, TFT_SS_PIN);
}

static void write_data(unsigned char data)
{
    MXC_GPIO_OutClr(TFT_SS_PORT, TFT_SS_PIN);

    spi_transmit(data, false);

    MXC_GPIO_OutSet(TFT_SS_PORT, TFT_SS_PIN);
}

static void window(unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
    write_command(COLUMN_ADDR);
    write_data(x >> 8);
    write_data(x);
    write_data((x + w - 1) >> 8);
    write_data(x + w - 1);

    write_command(PAGE_ADDR);
    write_data(y >> 8);
    write_data(y);
    write_data((y + h - 1) >> 8);
    write_data(y + h - 1);
}

static void pixel(int x, int y, int color)
{
    write_command(COLUMN_ADDR);
    write_data(x >> 8);
    write_data(x);
    write_command(PAGE_ADDR);
    write_data(y >> 8);
    write_data(y);
    write_command(MEM_WRITE);
    write_data(color >> 8);
    write_data(color & 0xff);
}

static unsigned int width(void)
{
    return ((tft_rotation == ROTATE_0) || (tft_rotation == ROTATE_180)) ? DISPLAY_HEIGHT :
                                                                          DISPLAY_WIDTH;
}

static unsigned int height(void)
{
    return ((tft_rotation == ROTATE_0) || (tft_rotation == ROTATE_180)) ? DISPLAY_WIDTH :
                                                                          DISPLAY_HEIGHT;
}

static void window_max(void)
{
    window(0, 0, width(), height());
}

static void h_line(int x0, int x1, int y, int color)
{
    int w;
    int j;

    w = x1 - x0 + 1;
    window(x0, y, w, 1);
    write_command(MEM_WRITE);

    for (j = 0; j < w; j++) {
        write_data(color >> 8);
        write_data(color & 0xff);
    }

    window_max();
}

static void v_line(int x, int y0, int y1, int color)
{
    int h;

    h = y1 - y0 + 1;
    window(x, y0, 1, h);
    write_command(MEM_WRITE);

    for (int y = 0; y < h; y++) {
        write_data(color >> 8);
        write_data(color & 0xff);
    }

    window_max();
}

static void fill_rect(int x0, int y0, int x1, int y1, int color)
{
    int h = y1 - y0 + 1;
    int w = x1 - x0 + 1;
    uint32_t pixel = h * w;
    uint16_t buf[160];
    const uint32_t buf_len = sizeof(buf) / sizeof(*buf);
    const uint16_t c = (color << 8) | (color >> 8);

    for (uint32_t i = 0; i < buf_len; i++) buf[i] = c;

    window(x0, y0, w, h);

    write_command(MEM_WRITE);

    while (pixel >= buf_len) {
        pixel -= buf_len;
        spi_transmit_data_buf(buf, sizeof(buf));
    }

    if (pixel)
        spi_transmit_data_buf(buf, pixel * sizeof(*buf));

    window_max();
}

static void locate(int x, int y)
{
    char_x = x;
    char_y = y;
}

static void tft_character(int x, int y, int c)
{
    int hor, vert, offset, bpl, j, i, b;
    unsigned char *sym;
    unsigned char z, w;

    (void)x;
    (void)y;

    if ((c < ' ') || (c > '~'))
        return;

    // read font parameter from start of array
    offset = g_font[0]; // bytes / char
    hor = g_font[1]; // get hor size of font
    vert = g_font[2]; // get vert size of font
    bpl = g_font[3]; // bytes per line

    if ((char_x + hor) > width()) {
        char_x = 0;
        char_y = char_y + vert;
        if (char_y >= (height() - g_font[2]))
            char_y = 0;
    }

    window(char_x, char_y, hor, vert); // char box
    write_command(MEM_WRITE);

    sym = &g_font[((c - ' ') * offset) + 4]; // start of char bitmap
    w = sym[0]; // width of actual char
    for (j = 0; j < vert; j++) { // vertical line
        for (i = 0; i < hor; i++) { // horizontal line
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

    window_max();

    if ((w + 2) < hor) // x offset to next char
        char_x += w + 2;
    else
        char_x += hor;
}

static void set_font(unsigned char *f)
{
    g_font = f;
}

static void show_image(unsigned int x0, unsigned int y0, unsigned int w, unsigned int h,
                       unsigned char *bitmap)
{
    unsigned int j, i, n;
    int padd;
    uint16_t buf[160];
    const uint32_t buf_len = sizeof(buf) / sizeof(*buf);

    // the lines are padded to multiple of 4 bytes in a bitmap
    padd = -1;
    do {
        padd++;
    } while ((2 * (w + padd) % 4) != 0);

    window(x0, y0, w, h);

    bitmap += ((h - 1) * (w + padd)) * 2;

    write_command(MEM_WRITE);

    for (j = 0; j < h; j++) { // lines
        for (i = w; i >= buf_len; i -= buf_len) { // buffer length chunks of line
            for (n = 0; n < buf_len; n++) {
                buf[n] = (*bitmap << 8) | *(bitmap + 1);
                bitmap += 2;
            }
            spi_transmit_data_buf(buf, sizeof(buf));
        }

        if (i) { // remaining portion of line less than chunk size
            for (n = 0; n < i; n++) {
                buf[n] = (*bitmap << 8) | *(bitmap + 1);
                bitmap += 2;
            }
            spi_transmit_data_buf(buf, i * sizeof(*buf));
        }

        bitmap -= 4 * w;
        bitmap -= padd * 2;
    }

    window_max();
}

static void spi_init(void)
{
    int master = 1;
    int quadMode = 0;
    int numSlaves = 0;
    int ssPol = 0;
    int tft_hz = TFT_SPI_FREQ;
    mxc_spi_pins_t tft_pins = {
        .clock = true,
        .ss0 = (ssel == 0),
        .ss1 = (ssel == 1),
        .ss2 = (ssel == 2),
        .miso = true,
        .mosi = true,
        .sdio2 = false,
        .sdio3 = false,
    };

    MXC_SPI_Init(spi, master, quadMode, numSlaves, ssPol, tft_hz, tft_pins);

    // Set SPI pins to VDDIOH (3.3V) to be compatible with TFT display
    MXC_GPIO_SetVSSEL(TFT_SPI_PORT, MXC_GPIO_VSSEL_VDDIOH, TFT_SPI_PINS);
    MXC_SPI_SetDataSize(spi, 8);
    MXC_SPI_SetWidth(spi, SPI_WIDTH_STANDARD);
}

static void display_init(void)
{
    if (reset_pin) {
        MXC_GPIO_OutClr(reset_pin->port, reset_pin->mask);
        MXC_Delay(MXC_DELAY_USEC(50));
        MXC_GPIO_OutSet(reset_pin->port, reset_pin->mask);
        MXC_Delay(MXC_DELAY_MSEC(5));
    } else {
        /* Wait here before attempting to configure display controller as the
       TFT Wing employs an integrated reset device with a 200ms typ POR pulse. */
        MXC_Delay(MXC_DELAY_MSEC(500));
    }

    static const struct tft_cmd_s {
        uint8_t cmd;
        uint8_t msdelay;
        uint8_t argc;
        char *args;
    } * pcmd, cmds[] = {
        // FW reset
        { 0x01, 5, 0, NULL },
        // Display off
        { 0x28, 0, 0, NULL },
        // Panel specific configuration
        { 0xCF, 0, 3, "\x00\x83\x30" },
        { 0xED, 0, 4, "\x64\x03\x12\x81" },
        { 0xE8, 0, 3, "\x85\x01\x79" },
        { 0xCB, 0, 5, "\x39\x2C\x00\x34\x02" },
        { 0xF7, 0, 1, "\x20" },
        { 0xEA, 0, 2, "\x00\x00" },
        // Power control 1
        { 0xC0, 0, 1, "\x26" },
        // Power control 2
        { 0xC1, 0, 1, "\x11" },
        // VCOM control 1
        { 0xC5, 0, 2, "\x35\x3E" },
        // VCOM control 2
        { 0xC7, 0, 1, "\xBE" },
        // Memory access control
        { 0x36, 0, 1, "\x48" },
        // Column mode and pixel format; 16-bit pixels here
        { 0x3A, 0, 1, "\x55" },
        // Frame rate
        { 0xB1, 0, 2, "\x00\x1B" },
        // Gamma function disable
        { 0xF2, 0, 1, "\x08" },
        // Gamma set curve
        { 0x26, 0, 1, "\x01" },
        // Positive gamma correction
        { 0xE0, 0, 15, "\x1F\x1A\x18\x0A\x0F\x06\x45\x87\x32\x0A\x07\x02\x07\x05\x00" },
        // Negative gamma correction
        { 0xE1, 0, 15, "\x00\x25\x27\x05\x10\x09\x3A\x78\x4D\x05\x18\x0D\x38\x3A\x1F" },
        // Column address
        { 0x2A, 0, 4, "\x00\x00\x01\x3F" },
        // Page address
        { 0x2B, 0, 4, "\x00\x00\x00\xEF" },
        // Entry mode
        { 0xB7, 0, 1, "\x07" },
        // Wake from sleep
        { 0x11, 100, 0, NULL },
        // Display on
        { 0x29, 100, 0, NULL },
    };

    int ncmds = sizeof(cmds) / sizeof(*cmds);

    for (pcmd = cmds; ncmds; ncmds--, pcmd++) {
        write_command(pcmd->cmd);

        if (pcmd->argc)
            spi_transmit_data_buf(pcmd->args, pcmd->argc);

        if (pcmd->msdelay)
            MXC_Delay(MXC_DELAY_MSEC(pcmd->msdelay));
    }
}

int MXC_TFT_Init(mxc_spi_regs_t *tft_spi, int ss_idx, mxc_gpio_cfg_t *reset_ctrl,
                 mxc_gpio_cfg_t *bl_ctrl)
{
    spi = tft_spi;
    ssel = ss_idx;
    reset_pin = reset_ctrl;
    blen_pin = bl_ctrl;

    // TFT Data/Command pin
    mxc_gpio_cfg_t tft_dc_pin = { TFT_DC_PORT, TFT_DC_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                  MXC_GPIO_VSSEL_VDDIOH };
    MXC_GPIO_Config(&tft_dc_pin);

    // TFT select pin
    mxc_gpio_cfg_t tft_ss_pin = { TFT_SS_PORT, TFT_SS_PIN, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                  MXC_GPIO_VSSEL_VDDIOH };
    MXC_GPIO_Config(&tft_ss_pin);
    MXC_GPIO_OutSet(TFT_SS_PORT, TFT_SS_PIN);

    if (reset_pin)
        MXC_GPIO_Config(reset_pin);

    if (blen_pin)
        MXC_GPIO_Config(blen_pin);

    spi_init();

    MXC_TFT_Backlight(0);

    display_init();

    MXC_TFT_SetBackGroundColor(g_background_color);

    MXC_TFT_Backlight(1);

    return E_NO_ERROR;
}

void MXC_TFT_SetRotation(tft_rotation_t rotation)
{
    tft_rotation = rotation;

    __disable_irq();

    write_command(MEM_ACCESS_CNTL);
    write_data("\x48\x28\x88\xe8"[rotation & 0x03]);

    window_max();

    __enable_irq();
}

void MXC_TFT_SetBackGroundColor(unsigned int color)
{
    __disable_irq();

    fill_rect(0, 0, width() - 1, height() - 1, color);

    __enable_irq();

    g_background_color = color;
}

void MXC_TFT_SetForeGroundColor(unsigned int color)
{
    g_foreground_color = color;
}

int MXC_TFT_SetPalette(int img_id)
{
    (void)img_id;

    return E_NOT_SUPPORTED;
}

void MXC_TFT_Backlight(int on)
{
    if (blen_pin) {
        if (on)
            MXC_GPIO_OutSet(blen_pin->port, blen_pin->mask);
        else
            MXC_GPIO_OutClr(blen_pin->port, blen_pin->mask);
    }
}

void MXC_TFT_ClearScreen(void)
{
    MXC_TFT_SetBackGroundColor(g_background_color);
}

void MXC_TFT_FillRect(area_t *area, int color)
{
    int x0, y0, x1, y1;

    x0 = area->x;
    y0 = area->y;

    x1 = x0 + area->w;
    y1 = y0 + area->h;

    __disable_irq();

    fill_rect(x0, y0, x1, y1, color);

    __enable_irq();
}

void MXC_TFT_WritePixel(int pixelX, int pixelY, int width, int height, uint32_t color)
{
    int x1, y1;

    x1 = pixelX + width;
    y1 = pixelY + height;

    __disable_irq();

    fill_rect(pixelX, pixelY, x1, y1, color);

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

    __disable_irq();

    show_image((unsigned int)x0, (unsigned int)y0, w, h, bitmap_ptr);

    __enable_irq();
}

void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t *image, int width, int height)
{
    int j, i;

    __disable_irq();

    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180)
        window(x0, y0, height, width);
    else
        window(x0, y0, width, height);

    write_command(MEM_WRITE);

    for (j = 0; j < width; j++) { // lines
        for (i = 0; i < (width * height); i += width) { // one line
            write_data(*(image + ((i + j) * 2)));
            write_data(*(image + ((i + j) * 2) + 1));
        }
    }

    window_max();

    __enable_irq();
}

void MXC_TFT_PrintPalette(void)
{
    area_t area = { 10, 10, 2, 25 };

    for (int i = 0; i < 256; i++) {
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

    __disable_irq();

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

    __enable_irq();
}

void MXC_TFT_FillCircle(int x0, int y0, int r, int color)
{
    int x = -r, y = 0, err = 2 - 2 * r, e2;

    __disable_irq();

    do {
        v_line(x0 - x, y0 - y, y0 + y, color);
        v_line(x0 + x, y0 - y, y0 + y, color);
        e2 = err;
        if (e2 <= y) {
            err += ++y * 2 + 1;
            if (-x == y && e2 <= x)
                e2 = 0;
        }

        if (e2 > x)
            err += ++x * 2 + 1;
    } while (x <= 0);

    __enable_irq();
}

void MXC_TFT_Line(int x0, int y0, int x1, int y1, int color)
{
    int dx, dy;
    int dx_sym = 0, dy_sym = 0;
    int dx_x2 = 0, dy_x2 = 0;
    int di = 0;

    dx = x1 - x0;
    dy = y1 - y0;

    __disable_irq();

    if (dx == 0) { // vertical line
        if (y1 > y0)
            v_line(x0, y0, y1, color);
        else
            v_line(x0, y1, y0, color);

        __enable_irq();

        return;
    }

    if (dx > 0)
        dx_sym = 1;
    else
        dx_sym = -1;

    if (dy == 0) { // horizontal line
        if (x1 > x0)
            h_line(x0, x1, y0, color);
        else
            h_line(x1, x0, y0, color);

        __enable_irq();

        return;
    }

    if (dy > 0)
        dy_sym = 1;
    else
        dy_sym = -1;

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

    __enable_irq();
}

void MXC_TFT_Rectangle(int x0, int y0, int x1, int y1, int color)
{
    __disable_irq();

    if (x1 > x0)
        h_line(x0, x1, y0, color);
    else
        h_line(x1, x0, y0, color);

    if (y1 > y0)
        v_line(x0, y0, y1, color);
    else
        v_line(x0, y1, y0, color);

    if (x1 > x0)
        h_line(x0, x1, y1, color);
    else
        h_line(x1, x0, y1, color);

    if (y1 > y0)
        v_line(x1, y0, y1, color);
    else
        v_line(x1, y1, y0, color);

    __enable_irq();
}

void MXC_TFT_ResetCursor(void) {}

void MXC_TFT_SetFont(int font_id)
{
    if (font_id != 0)
        set_font((unsigned char *)font_id);
}

void MXC_TFT_Printf(const char *format, ...)
{
    (void)format;
}

void MXC_TFT_ConfigPrintf(area_t *area)
{
    (void)area;
}

void MXC_TFT_PrintFont(int x0, int y0, int id, text_t *str, area_t *area)
{
    int i;
    char value;

    (void)area;

    if (id)
        MXC_TFT_SetFont(id);

    locate(x0, y0);

    __disable_irq();

    for (i = 0; i < str->len; i++) {
        value = str->data[i];

        if (value == '\n') {
            char_x = 0;
            char_y = char_y + g_font[2];

            if (char_y >= height() - g_font[2]) {
                char_y = 0;
            }
        } else {
            tft_character(char_x, char_y, value);
        }
    }

    __enable_irq();
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
    __disable_irq();

    write_command(command);
    write_data(data);

    __enable_irq();
}
