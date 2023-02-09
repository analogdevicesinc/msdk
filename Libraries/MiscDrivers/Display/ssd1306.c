/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <string.h>
#include "ssd1306.h"

/************************************ DEFINES ********************************/
#define SSD1306_COMMAND (0x00)
#define SSD1306_DATA (0x40)

#define SSD1306_LOW_COLUMN_BASE_ADDR (0x00)
#define SSD1306_HIGH_COLUMN_BASE_ADDR (0x10)
#define SSD1306_PAGE_START_BASE_ADDR (0xB0)

#define SSD1306_SET_LOW_COLUMN(col) (SSD1306_LOW_COLUMN_BASE_ADDR | (col & 0xF))
#define SSD1306_SET_HIGH_COLUMN(col) (SSD1306_HIGH_COLUMN_BASE_ADDR | ((col >> 4) & 0xF))

#define SSD1306_SET_MEMORY_ADDR_MODE (0x20)
#define SSD1306_MEMORYADDR_MODE_HORIZONTAL (0x00)
#define SSD1306_MEMORYADDR_MODE_VERTICAL (0x01)
#define SSD1306_MEMORYADDR_MODE_PAGE (0x02)

#define SSD1306_SET_COLUMN_ADDR (0x21)
#define SSD1306_SET_PAGE_ADDR (0x22)

#define SSD1306_SET_DISPLAY_START_LINE(offset) (0x40 + (offset & 0x3F))

#define SSD1306_SET_CONTRAST (0x81)

#define SSD1306_SET_SEG_NOMAP (0xA0)
#define SSD1306_SET_SEG_REMAP (0xA1)

#define SSD1306_SET_ENTIRE_DISPLAY_NORMAL (0xA4)
#define SSD1306_SET_ENTIRE_DISPLAY_FORCE_ON (0xA5)

#define SSD1306_SET_NORMAL_PIXEL_DISPLAY (0xA6)
#define SSD1306_SET_INVERT_PIXEL_DISPLAY (0xA7)

#define SSD1306_SET_MULTIPLEX_RATIO (0xA8)

#define SSD1306_SET_DISPLAY_SLEEP (0xAE)
#define SSD1306_SET_DISPLAY_WAKE (0xAF)

#define SSD1306_SET_PAGE_START_ADDR(row) (SSD1306_PAGE_START_BASE_ADDR | row)

#define SSD1306_SET_COM_SCAN_DIR_UP (0xC0)
#define SSD1306_SET_COM_SCAN_DIR_DOWN (0xC8)

#define SSD1306_SET_DISPLAY_OFFSET (0xD3)

#define SSD1306_SET_DISPLAY_CLOCK_DIV_OSC_FREQ (0xD5)
#define SSD1306_DISPLAY_CLOCK_DIV_OSC_FREQ(div, freq) ((div & 0xF) | ((freq << 4) | 0xF0))

#define SSD1306_SET_PRE_CHARGE_PERIOD (0xD9)

#define SSD1306_SET_COM_PINS_HW (0xDA)
#define SSD1306_COM_CONFIG_SEQUENTIAL_LEFT (0x02)
#define SSD1306_COM_CONFIG_ALTERNATE_LEFT (0x12)
#define SSD1306_COM_CONFIG_SEQUENTIAL_RIGHT (0x22)
#define SSD1306_COM_CONFIG_ALTERNATE_RIGHT (0x32)

#define SSD1306_SET_VCOM_DESELECT (0xDB)

//
#define SSD1306_DCDC_CONFIG_PREFIX (0x8D)
#define SSD1306_DCDC_CONFIG_7p5v (0x14)
#define SSD1306_DCDC_CONFIG_6p0v (0x15)
#define SSD1306_DCDC_CONFIG_8p5v (0x94)
#define SSD1306_DCDC_CONFIG_9p0v (0x95)

#define SSD1306_VCOMH_DESELECT_0p65xVCC (0x00)
#define SSD1306_VCOMH_DESELECT_0p71xVCC (0x10)
#define SSD1306_VCOMH_DESELECT_0p77xVCC (0x20)
#define SSD1306_VCOMH_DESELECT_0p83xVCC (0x30)

#define SSD1306_CONTRAST_FACTORY_VALUE (0xBF) //magic # from factory

#define SSD1306_ACTIVATE_SCROLL (0x2F)
#define SSD1306_DEACTIVATE_SCROLL (0x2E)
#define SSD1306_SET_VERTICAL_SCROLL_AREA (0xA3)
#define SSD1306_RIGHT_HORIZONTAL_SCROLL (0x26)
#define SSD1306_LEFT_HORIZONTAL_SCROLL (0x27)
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL (0x29)
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL (0x2A)

/* **** Static Functions **** */

static int sendcommand(ssd1306_dev *dev, uint8_t payload)
{
    dev->comm_api.comm_buffer[0] = 0x00;
    dev->comm_api.comm_buffer[1] = payload;
    return dev->comm_api.write(dev->comm_api.comm_buffer, 2);
}

static int senddata(ssd1306_dev *dev, uint8_t *payload, uint32_t payloadlen)
{
    dev->comm_api.comm_buffer[0] = 0x40;
    memcpy(&dev->comm_api.comm_buffer[1], payload, payloadlen);
    return dev->comm_api.write(dev->comm_api.comm_buffer, payloadlen + 1);
}

/* **** Functions **** */
int ssd1306_configure(ssd1306_dev *dev, ssd1306_init_param_t *init_param,
                      display_comm_api *comm_api)
{
    int err = DISP_E_SUCCESS;

    if (init_param == NULL || comm_api == NULL || dev == NULL) {
        return DISP_E_BAD_PARAM;
    }
    dev->init_param = *init_param;
    dev->comm_api = *comm_api;
    return err;
}

int ssd1306_init(ssd1306_dev *dev)
{
    if (dev == NULL) {
        return DISP_E_NOT_CONFIGURED;
    }
    dev->comm_api.init();

    sendcommand(dev, SSD1306_SET_DISPLAY_WAKE);
    sendcommand(dev, SSD1306_SET_DISPLAY_SLEEP);

    sendcommand(dev, SSD1306_SET_DISPLAY_CLOCK_DIV_OSC_FREQ);
    sendcommand(dev, SSD1306_DISPLAY_CLOCK_DIV_OSC_FREQ(0x0, 0x8));

    sendcommand(dev, SSD1306_SET_MULTIPLEX_RATIO);
    sendcommand(dev, dev->init_param.row - 1);

    sendcommand(dev, SSD1306_SET_DISPLAY_OFFSET);
    sendcommand(dev, 0x00);

    sendcommand(dev, SSD1306_SET_DISPLAY_START_LINE(0x00));

    sendcommand(dev, SSD1306_DCDC_CONFIG_PREFIX);
    sendcommand(dev, SSD1306_DCDC_CONFIG_7p5v);

    if (dev->init_param.rotation == DISPLAY_NORMAL) {
        sendcommand(dev, SSD1306_SET_SEG_NOMAP);
        sendcommand(dev, SSD1306_SET_COM_SCAN_DIR_UP);
    } else {
        sendcommand(dev, SSD1306_SET_SEG_REMAP);
        sendcommand(dev, SSD1306_SET_COM_SCAN_DIR_DOWN);
    }

    sendcommand(dev, SSD1306_SET_COM_PINS_HW);
    sendcommand(dev, SSD1306_COM_CONFIG_SEQUENTIAL_LEFT);

    sendcommand(dev, SSD1306_SET_CONTRAST);
    sendcommand(dev, SSD1306_CONTRAST_FACTORY_VALUE);

    //Set precharge (low nibble) / discharge (high nibble) timing
    //precharge = 1 clock, discharge = 15 clocks
    sendcommand(dev, SSD1306_SET_PRE_CHARGE_PERIOD); //Set Pre-Charge period
    sendcommand(dev, 0xF1);

    sendcommand(dev, SSD1306_SET_VCOM_DESELECT);
    sendcommand(dev, SSD1306_VCOMH_DESELECT_0p83xVCC);

    sendcommand(dev, SSD1306_SET_ENTIRE_DISPLAY_NORMAL);

    if (dev->init_param.color_invert == 1) {
        sendcommand(dev, SSD1306_SET_INVERT_PIXEL_DISPLAY);
    } else {
        sendcommand(dev, SSD1306_SET_NORMAL_PIXEL_DISPLAY);
    }
    //
    sendcommand(dev, SSD1306_SET_DISPLAY_WAKE);

    return DISP_E_SUCCESS;
}

void ssd1306_set_cursor(ssd1306_dev *dev, uint8_t row, uint16_t col)
{
    sendcommand(dev, SSD1306_SET_PAGE_START_ADDR(row)); //set page address
    sendcommand(dev, SSD1306_SET_LOW_COLUMN(col)); //lower column address
    sendcommand(dev, SSD1306_SET_HIGH_COLUMN(col)); //upper column address
}

void ssd1306_send_pixels(ssd1306_dev *dev, uint8_t *payload, uint32_t payloadlen)
{
    senddata(dev, payload, payloadlen);
}

void ssd1306_flush_area(ssd1306_dev *dev, const display_area_t *area, const uint8_t *data)
{
    // Consist of pages! Each 8 bit!
    uint8_t page1 = area->y1 >> 3;
    uint8_t page2 = area->y2 >> 3;

    for (uint8_t page = page1; page <= page2; page++) {
        ssd1306_set_cursor(dev, page, area->x1);

        ssd1306_send_pixels(dev, (uint8_t *)data, area->x2 - area->x1 + 1);
        data += area->x2 - area->x1 + 1;
    }
}

void ssd1306_set_buffer_pixel_util(uint8_t *buf, uint16_t buf_w, uint32_t buf_max, uint16_t x,
                                   uint16_t y, uint8_t color, uint8_t is_opaque)
{
    SSD1306_SET_BUFFER_PIXEL_UTIL(buf, buf_w, buf_max, x, y, color, is_opaque);
}
