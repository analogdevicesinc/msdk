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
#include "example_config.h"
#include "post_process.h"

#ifdef TFT_ENABLE
#ifdef BOARD_EVKIT_V1
static int font = urw_gothic_12_grey_bg_white;
#endif
#ifdef BOARD_FTHR_REVA
static int font = (int)&SansSerif16x16[0];
#endif

static text_t label_text[] = {
    // info
    { (char *)"One", 3 },  { (char *)"Two", 3 },  { (char *)"Three", 5 }, { (char *)"Four", 4 },
    { (char *)"Five", 4 }, { (char *)"Six", 3 },  { (char *)"Seven", 5 }, { (char *)"Eight", 5 },
    { (char *)"Nine", 4 }, { (char *)"Zero", 4 },
};
#endif

void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;

    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

void draw_obj_rect(float *xy, int class_idx, uint32_t w, uint32_t h, uint8_t scale)
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

    // sanity check
    if (x1 < 1)
        x1 = 1;
    if (y1 < 1)
        y1 = 1;
    if (x2 < 1)
        x2 = 1;
    if (y2 < 1)
        y2 = 1;

    if (x1 > w - 1)
        x1 = w - 1;
    if (y1 > h - 1)
        y1 = h - 1;
    if (x2 > w - 1)
        x2 = w - 1;
    if (y2 > h - 1)
        y2 = h - 1;

#ifdef BOARD_EVKIT_V1
    color = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
    // Convert to RGB565
    color = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
#endif

    for (x = x1; x < x2; ++x) {
        MXC_TFT_WritePixel(x * scale + TFT_X_OFFSET, y1 * scale, scale, scale, color);
        MXC_TFT_WritePixel(x * scale + TFT_X_OFFSET, y2 * scale, scale, scale, color);
    }

    for (y = y1; y < y2; ++y) {
        MXC_TFT_WritePixel(x1 * scale + TFT_X_OFFSET, y * scale, scale, scale, color);
        MXC_TFT_WritePixel(x2 * scale + TFT_X_OFFSET, y * scale, scale, scale, color);
    }

    MXC_TFT_PrintFont(x1 * scale + THICKNESS + TFT_X_OFFSET, y1 * scale + THICKNESS, font,
                      &label_text[class_idx], NULL);
#endif
}
int dma_channel;
int g_dma_channel_tft = 1;
static uint8_t *rx_data = NULL;

void setup_dma_tft(uint32_t *src_ptr, uint16_t byte_cnt)
{
    // TFT DMA
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }

    MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_channel_tft].dst = (uint32_t)rx_data; // Cast Pointer
    MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)src_ptr;
    MXC_DMA->ch[g_dma_channel_tft].cnt = byte_cnt;

    MXC_DMA->ch[g_dma_channel_tft].ctrl =
        ((0x1 << MXC_F_DMA_CTRL_CTZ_IE_POS) + (0x0 << MXC_F_DMA_CTRL_DIS_IE_POS) +
         (0x1 << MXC_F_DMA_CTRL_BURST_SIZE_POS) + (0x0 << MXC_F_DMA_CTRL_DSTINC_POS) +
         (0x1 << MXC_F_DMA_CTRL_DSTWD_POS) + (0x1 << MXC_F_DMA_CTRL_SRCINC_POS) +
         (0x1 << MXC_F_DMA_CTRL_SRCWD_POS) + (0x0 << MXC_F_DMA_CTRL_TO_CLKDIV_POS) +
         (0x0 << MXC_F_DMA_CTRL_TO_WAIT_POS) + (0x2F << MXC_F_DMA_CTRL_REQUEST_POS) + // SPI0 -> TFT
         (0x0 << MXC_F_DMA_CTRL_PRI_POS) + // High Priority
         (0x0 << MXC_F_DMA_CTRL_RLDEN_POS) // Disable Reload
        );

    MXC_SPI0->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);
    MXC_SETFIELD(MXC_SPI0->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR,
                 (byte_cnt) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    // Clear SPI master done flag
    MXC_SPI0->intfl = MXC_F_SPI_INTFL_MST_DONE;
    MXC_SETFIELD(MXC_SPI0->dma, MXC_F_SPI_DMA_TX_THD_VAL, 0x10 << MXC_F_SPI_DMA_TX_THD_VAL_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_DMA_TX_EN);
    MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);
}

void start_tft_dma(uint32_t *src_ptr, uint16_t byte_cnt)
{
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }

    if (MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_CTZ_IF) {
        MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF;
    }

    MXC_DMA->ch[g_dma_channel_tft].cnt = byte_cnt;
    MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)src_ptr;

    // Enable DMA channel
    MXC_DMA->ch[g_dma_channel_tft].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
    MXC_Delay(1); // to fix artifacts in the image
    // Start DMA
    MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
}

void tft_dma_display(int x, int y, int w, int h, uint32_t *data)
{
    // setup dma
    setup_dma_tft((uint32_t *)data, w * h * 2);

    // Send a line of captured image to TFT
    start_tft_dma((uint32_t *)data, w * h * 2);
}
