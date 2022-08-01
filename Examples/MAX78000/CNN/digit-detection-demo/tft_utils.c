/*******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 *******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "mxc_device.h"
#include "board.h"
#include "mxc_delay.h"
#include "rtc.h"
#include "uart.h"
#include "tft_utils.h"

#ifdef TFT_ENABLE
#ifdef BOARD_EVKIT_V1
static int font = urw_gothic_12_grey_bg_white;
#endif
#ifdef BOARD_FTHR_REVA
static int font = (int)&SansSerif16x16[0];
#endif

static text_t label_text[] = {
    // info
    {(char*)"One", 3},  {(char*)"Two", 3},  {(char*)"Three", 5}, {(char*)"Four", 4},
    {(char*)"Five", 4}, {(char*)"Six", 3},  {(char*)"Seven", 5}, {(char*)"Eight", 5},
    {(char*)"Nine", 4}, {(char*)"Zero", 4},
};
#endif

uint8_t signed_to_unsigned(int8_t val)
{
    uint8_t value;

    if (val < 0) {
        value = ~val + 1;
        return (128 - value);
    }

    return val + 128;
}

#ifdef TFT_ENABLE
void show_image(uint32_t* image, uint32_t xcord, uint32_t ycord, uint32_t scale, uint32_t w,
                uint32_t h)
{
    int r, g, b;
    uint32_t i, x, y;
    uint32_t color;

    x = xcord;
    y = ycord;

    for (i = 0; i < (w * h); i++) {
        b = signed_to_unsigned(((image[i] >> 16) & 0xFF));
        g = signed_to_unsigned(((image[i] >> 8) & 0xFF));
        r = signed_to_unsigned(((image[i] >> 0) & 0xFF));
        // b = (image[i] >> 16) & 0xFF;
        // g = (image[i] >>  8) & 0xFF;
        // r = (image[i] >>  0) & 0xFF;
#ifdef BOARD_EVKIT_V1
        color =
            (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
        // Convert to RGB565
        color = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
#endif

        MXC_TFT_WritePixel(x * scale, y * scale, scale, scale, color);

        x += 1;

        if (x >= (w + xcord)) {
            x = xcord;
            y += 1;
        }
    }
}
#endif

void TFT_Print(char* str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len  = length;

    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

void draw_obj_rect(float* xy, int class_idx, uint32_t w, uint32_t h, uint8_t scale)
{
#ifdef TFT_ENABLE
    int r = 0, g = 0, b = 0;
    uint32_t color;

    if (class_idx == 0) {
        r = 253;
        g = 172;
        b = 83;
    } else if (class_idx == 1) {
        r = 155;
        g = 183;
        b = 212;
    } else if (class_idx == 2) {
        r = 181;
        g = 90;
        b = 48;
    } else if (class_idx == 3) {
        r = 245;
        g = 223;
        b = 77;
    } else if (class_idx == 4) {
        r = 48;
        g = 120;
        b = 180;
    } else if (class_idx == 5) {
        r = 160;
        g = 218;
        b = 169;
    } else if (class_idx == 6) {
        r = 233;
        g = 137;
        b = 126;
    } else if (class_idx == 7) {
        r = 0;
        g = 182;
        b = 148;
    } else if (class_idx == 8) {
        r = 147;
        g = 105;
        b = 168;
    } else if (class_idx == 9) {
        r = 210;
        g = 56;
        b = 108;
    }

    int x1 = w * xy[0];
    int y1 = h * xy[1];
    int x2 = w * xy[2];
    int y2 = h * xy[3];
    int x, y;

#ifdef BOARD_EVKIT_V1
    color = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
    // Convert to RGB565
    color = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
#endif

    for (x = x1; x < x2; ++x) {
        MXC_TFT_WritePixel(x * scale, y1 * scale, scale, scale, color);
        MXC_TFT_WritePixel(x * scale, y2 * scale, scale, scale, color);
    }

    for (y = y1; y < y2; ++y) {
        MXC_TFT_WritePixel(x1 * scale, y * scale, scale, scale, color);
        MXC_TFT_WritePixel(x2 * scale, y * scale, scale, scale, color);
    }

    MXC_TFT_PrintFont(x1 * scale + THICKNESS, y1 * scale + THICKNESS, font, &label_text[class_idx],
                      NULL);
#endif
}
