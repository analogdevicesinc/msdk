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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "mxc.h"
#include "tft_st7789v.h"

/*
  The NewHaven 2.4" TFT includes Sitronix ST7789V 240x320 TFT LCD controller
  It supports 4-wire (CS/DC/SCLK/MISO/MOSI) 8-bit data serial interface
*/
static mxc_gpio_cfg_t *reset_pin;
static mxc_gpio_cfg_t *blen_pin;

static tft_rotation_t tft_rotation = ROTATE_0;
static unsigned int g_foreground_color = WHITE;
static unsigned int g_background_color = BLACK;
static unsigned char *g_font;

static unsigned int char_x;
static unsigned int char_y;

static void write_command(unsigned char command)
{
    TFT_SPI_Write(command, true);
}

static void write_data(unsigned char data)
{
    TFT_SPI_Write(data, false);
}

static void window(unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
    write_command(COLUMN_ADDR);
    write_data(x >> 8);
    write_data(x);
    write_data((x + w - 1) >> 8);
    write_data(x + w - 1);

    write_command(ROW_ADDR);
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
    write_command(ROW_ADDR);
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
        TFT_SPI_Transmit(buf, sizeof(buf));
    }

    if (pixel)
        TFT_SPI_Transmit(buf, pixel * sizeof(*buf));

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
            TFT_SPI_Transmit(buf, sizeof(buf));
        }

        if (i) { // remaining portion of line less than chunk size
            for (n = 0; n < i; n++) {
                buf[n] = (*bitmap << 8) | *(bitmap + 1);
                bitmap += 2;
            }
            TFT_SPI_Transmit(buf, i * sizeof(*buf));
        }

        bitmap -= 4 * w;
        bitmap -= padd * 2;
    }

    window_max();
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
       TFT employs an integrated reset device with a 200ms typ POR pulse. */
        MXC_Delay(MXC_DELAY_MSEC(500));
    }

    static const struct tft_cmd_s {
        uint8_t cmd;
        uint8_t msdelay;
        uint8_t argc;
        char *args;
    } * pcmd, cmds[] = {
        // FW reset
        { 0x01, 10, 0, NULL },
        // Display off
        { 0x28, 0, 0, NULL },
        // Exit Sleep mode
        { 0x11, 0, 0, NULL },
        // MADCTL: memory data access control, RGB=0 (RGB order), MV=0, MX=0, MY=0, Normal (0 degree rotation)
        { 0x36, 0, 1, "\x00" },
        // Column mode and pixel format; 16-bit pixels, 65K RGB color
        { 0x3A, 0, 1, "\x55" },
        // PORCTRK: Porch setting
        { 0xB2, 0, 5, "\x0C\x0C\x00\x33\x33" },
        // GCTRL: Gate Control
        { 0xB7, 0, 1, "\x35" },
        // VCOMS: VCOM setting
        { 0xBB, 0, 1, "\x2B" },
        // LCMCTRL: LCM Control
        { 0xC0, 0, 1, "\x2C" },
        // VDVVRHEN: VDV and VRH Command Enable
        { 0xC2, 0, 2, "\x01\xFF" },
        // VRHS: VRH set
        { 0xC3, 0, 1, "\x11" },
        // VDVS: VDV Set
        { 0xC4, 0, 1, "\x20" },
        // FRCTRL2: Frame Rate control in normal mode
        { 0xC6, 0, 1, "\x0F" },
        // PWCTRL1: Power Control 1
        { 0xD0, 0, 2, "\xA4\xA1" },
        // PVGAMCTRL: Positive Voltage Gamma control
        { 0xE0, 0, 14, "\xD0\xD0\x05\x0E\x15\x0D\x37\x43\x47\x09\x15\x12\x16\x19" },
        // NVGAMCTRL: Negative Voltage Gamma control
        { 0xE1, 0, 14, "\xD0\x00\x05\x0D\x0C\x06\x2D\x44\x40\x0E\x1C\x18\x16\x19" },
        // X address set
        { 0x2A, 0, 4, "\x00\x00\x00\xEF" },
        // Y address set
        { 0x2B, 0, 4, "\x00\x00\x01\x3F" },
        // Display on
        { 0x29, 10, 0, NULL }
    };

    int ncmds = sizeof(cmds) / sizeof(*cmds);

    for (pcmd = cmds; ncmds; ncmds--, pcmd++) {
        write_command(pcmd->cmd);

        if (pcmd->argc)
            TFT_SPI_Transmit(pcmd->args, pcmd->argc);

        if (pcmd->msdelay)
            MXC_Delay(MXC_DELAY_MSEC(pcmd->msdelay));
    }
}
int MXC_TFT_Init(mxc_gpio_cfg_t *reset_ctrl, mxc_gpio_cfg_t *bl_ctrl)
{
    reset_pin = reset_ctrl;
    blen_pin = bl_ctrl;

    if (reset_pin)
        MXC_GPIO_Config(reset_pin);

    if (blen_pin)
        MXC_GPIO_Config(blen_pin);

    TFT_SPI_Init();

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
    write_data("\x00\x60\xC0\xA0"[rotation & 0x03]);

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

void MXC_TFT_ShowImageCameraMono(int x0, int y0, uint8_t *image, int width, int height)
{
    int j, i;
    uint8_t mono;
    uint16_t rgb565;

    __disable_irq();

    if (tft_rotation == ROTATE_0 || tft_rotation == ROTATE_180)
        window(x0, y0, height, width);
    else
        window(x0, y0, width, height);

    write_command(MEM_WRITE);

    for (j = 0; j < width; j++) { // lines
        for (i = 0; i < (width * height); i += width) { // one line
            // Display is configured for RGB565, so for mono images convert with R=G=B
            mono = image[i + j];
            rgb565 = RGB(mono, mono, mono);
            write_data(rgb565 & 0xFF);
            write_data((rgb565 >> 8) & 0xFF);
        }
    }

    window_max();

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

    // Check if IRQs are currently enabled
    uint32_t irq_enabled = 0;
#ifndef __riscv
    /*
        On ARM M the 0th bit of the Priority Mask register indicates
        whether interrupts are enabled or not.

        0 = enabled
        1 = disabled
    */
    irq_enabled = !(__get_PRIMASK() & 0b1);
#else
    /*
        On RISC-V bit position 3 (Machine Interrupt Enable) of the
        mstatus register indicates whether interrupts are enabled.

        0 = disabled
        1 = enabled
    */
    irq_enabled = ((get_mstatus() & (1 << 3)) != 0);
#endif
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

    if (irq_enabled) { // Re-enable IRQs if they were previously
        __enable_irq();
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
    // Check if IRQs are currently enabled
    // __disable_irq always uses cpsid i, so check PRIMASK bit to save state
    uint32_t irq_enabled = 0;

#ifndef __riscv
    irq_enabled = !(__get_PRIMASK() & 0b1);
#else
    irq_enabled = ((get_mstatus() & (1 << 3)) != 0);
#endif
    __disable_irq();

    write_command(command);
    write_data(data);

    if (irq_enabled) { // Re-enable IRQs if they were previously
        __enable_irq();
    }
}
