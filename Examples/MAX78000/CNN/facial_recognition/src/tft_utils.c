/******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
 ******************************************************************************
 *
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
#include "mxc.h"
#include "mxc_device.h"
#include "tft_utils.h"
#include "facedetection.h"

void draw_obj_rect(float *xy, uint32_t w, uint32_t h)
{
#ifdef TFT_ENABLE

    int x1 = h * (xy[1]) + X_START;
    int y1 = w * (xy[2]) + Y_START - 1;
    int x2 = h * (xy[3]) + X_START - 1;
    int y2 = w * (xy[0]) + Y_START;

    // Draw rectangle around detected face
    MXC_TFT_Rectangle(x1, y1, x2, y2, FRAME_ORANGE);
#endif
}

//--------------------------------------------------------------------
#ifdef TFT_ENABLE

int dma_channel;
int g_dma_channel_tft = 1;
uint8_t *rx_data = NULL;
void setup_dma_tft(uint32_t *src_ptr)
{
    printf("TFT DMA setup\n");
    // TFT DMA
    MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF; // Clear CTZ status flag
    MXC_DMA->ch[g_dma_channel_tft].dst = (uint32_t)rx_data; // Cast Pointer
    MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)src_ptr;
    MXC_DMA->ch[g_dma_channel_tft].cnt = IMAGE_XRES * IMAGE_YRES;

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
                 (IMAGE_XRES * IMAGE_YRES) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    // Clear SPI master done flag
    MXC_SPI0->intfl = MXC_F_SPI_INTFL_MST_DONE;
    MXC_SETFIELD(MXC_SPI0->dma, MXC_F_SPI_DMA_TX_THD_VAL, 0x10 << MXC_F_SPI_DMA_TX_THD_VAL_POS);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN);
    MXC_SPI0->dma |= (MXC_F_SPI_DMA_DMA_TX_EN);
    MXC_SPI0->ctrl0 |= (MXC_F_SPI_CTRL0_EN);
}

void start_tft_dma(uint32_t *src_ptr)
{
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }

    if (MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_CTZ_IF) {
        MXC_DMA->ch[g_dma_channel_tft].status = MXC_F_DMA_STATUS_CTZ_IF;
    }

    MXC_DMA->ch[g_dma_channel_tft].cnt = IMAGE_XRES * IMAGE_YRES;
    MXC_DMA->ch[g_dma_channel_tft].src = (uint32_t)src_ptr;

    // Enable DMA channel
    MXC_DMA->ch[g_dma_channel_tft].ctrl += (0x1 << MXC_F_DMA_CTRL_EN_POS);
    // Start DMA
    MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;
}

#endif
