/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "uart.h"
#include "led.h"
#include "board.h"

#include <stdlib.h>
#include <string.h>
#include "cnn.h"
#include "lp.h"
#include "camera.h"
#include "dma.h"
#include "dma_regs.h"
#include "camera_util.h"
#include "project_config.h"
#include "rtc.h"

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

int slaveAddress;
int id;
int dma_channel;

static uint32_t *data_addr0 = (uint32_t *)0x50400700;
static uint32_t *data_addr3 = (uint32_t *)0x50418700;
static uint32_t *data_addr6 = (uint32_t *)0x50810700;
static uint32_t *data_addr9 = (uint32_t *)0x50c08700;

static uint32_t *addr, offset0, offset1, subtract;

// *****************************************************************************

#ifdef TFT_ENABLE

static int g_dma_channel_tft = 1;
static uint8_t *rx_data = NULL;

static void setup_dma_tft(uint32_t *src_ptr)
{
    MXC_Delay(MSEC(1));
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

static void start_tft_dma(uint32_t *src_ptr)
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

int initialize_camera(void)
{
    int ret = 0;

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    camera_init(CAMERA_FREQ);
    printf("\n\nCamera Example\n");

    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    // Obtain the manufacturer ID of the camera.
    ret = camera_get_manufacture_id(&id);

    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    printf("Camera ID detected: %04x\n", id);

    printf("Init Camera\n");

    // Setup the camera image dimensions, pixel format and data acquiring details.
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, USE_DMA,
                       dma_channel);

    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

    // double camera PCLK speed
    camera_write_reg(0x11, 0x80);

    return ret;
}

static void load_row_cnn_init(int frame_no)
{
    if (frame_no == 0) {
        data_addr0 = (uint32_t *)0x50400f00;
        data_addr3 = (uint32_t *)0x50418f00;
        data_addr6 = (uint32_t *)0x50810f00;
        data_addr9 = (uint32_t *)0x50c08f00;
    } else {
        data_addr0 = (uint32_t *)0x50400f04;
        data_addr3 = (uint32_t *)0x50418f04;
        data_addr6 = (uint32_t *)0x50810f04;
        data_addr9 = (uint32_t *)0x50c08f04;
    }
    addr = data_addr9;
}

static void load_cnn(uint8_t *data, uint32_t row_number)
{
    union {
        uint32_t w;
        uint8_t b[4];
    } m;

    for (int row = 0; row < row_number; row++) {
        offset0 = 0x00002000;
        offset1 = 0x00002000;
        subtract = 0x00004000 - 2;

        switch (row & 3) {
        case 0:
            data_addr9 = addr;
            addr = data_addr0;
            break;

        case 1:
            data_addr0 = addr;
            addr = data_addr3;
            offset0 += 0x000FA000 - 0x00002000;
            subtract += 0x000FC000 - 0x00004000;
            break;

        case 2:
            data_addr3 = addr;
            addr = data_addr6;
            offset1 += 0x000FA000 - 0x00002000;
            subtract += 0x000FC000 - 0x00004000;
            break;

        default:
            data_addr6 = addr;
            addr = data_addr9;
            break;
        }

        // indexes of 240x240 image (row,j)
        for (int j = 0; j < IMAGE_XRES; j += 4) {
            // RGB565 to packed 24-bit RGB
            m.b[0] = (*data & 0xF8); // Red
            m.b[1] = (*data << 5) | ((*(data + 1) & 0xE0) >> 3); // Green
            m.b[2] = (*(data + 1) << 3); // Blue
            data += 2;
            m.b[3] = (*data & 0xF8); // Red

            *addr = m.w ^ 0x80808080U;
            addr += offset0;

            m.b[0] = (*data << 5) | ((*(data + 1) & 0xE0) >> 3); // Green
            m.b[1] = (*(data + 1) << 3); // Blue
            data += 2;
            m.b[2] = (*data & 0xF8); // Red
            m.b[3] = (*data << 5) | ((*(data + 1) & 0xE0) >> 3); // Green

            *addr = m.w ^ 0x80808080U;
            addr += offset1;

            m.b[0] = (*(data + 1) << 3); // Blue
            data += 2;
            m.b[1] = (*data & 0xF8); // Red
            m.b[2] = (*data << 5) | ((*(data + 1) & 0xE0) >> 3); // Green
            m.b[3] = (*(data + 1) << 3); // Blue
            data += 2;

            *addr = m.w ^ 0x80808080U;
            addr -= subtract;
        }
    }
}

/*
  Data Order:
  2 consecutive frames are loaded from the camera: (240,240,3)*2
  following indexes are based on camera pixel index

  0x50400f00:
  
  For the first frame:
      (0,1,0)|(0,0,2)|(0,0,1)|(0,0,0)              // 0
      // reserved space for the next frame           (1)                                 
      (0,5,0)|(0,4,2)|(0,4,1)|(0,4,0)              // 2
      // reserved space for the next frame           (3) 
      ...
      (236,237,0)|(236,236,2)|(236,236,1)|(236,236,0) // 7198
      // reserved space for the next frame              (7199)

  When the second frame is later loaded, the above mentioned spaces are filled
      // data is already filled from first frame     (0)
      (0,1,0)|(0,0,2)|(0,0,1)|(0,0,0)              // 1
      // data filled from first frame                (2)
      (0,5,0)|(0,4,2)|(0,4,1)|(0,4,0)              // 3
      ...
      // data filled from first frame                   (7198)
      (236,237,0)|(236,236,2)|(236,236,1)|(236,236,0) // 7199

 The same pattern of 3x3600x2 words is repeated, resulting in 4x3x3600*2 words:
 ....
    0x50c18f00: last bank
    ...
      (351,351,2)|(351,351,1)|(351,351,0)|(351,350,2)      // 7198 (from first frame)
      (351,351,2)|(351,351,1)|(351,351,0)|(351,350,2)      // 7199 (from second frame)

   */

// This function loads the sample data input -- replace with actual data
// Auto generated data sample codes :
// void load_input(void)
// {
//   // This function loads the sample data input -- replace with actual data
//   memcpy32((uint32_t *) 0x50400f00, input_0, 7200);
//   memcpy32((uint32_t *) 0x50408f00, input_4, 7200);
//   memcpy32((uint32_t *) 0x50410f00, input_8, 7200);
//   memcpy32((uint32_t *) 0x50418f00, input_12, 7200);
//   memcpy32((uint32_t *) 0x50800f00, input_16, 7200);
//   memcpy32((uint32_t *) 0x50808f00, input_20, 7200);
//   memcpy32((uint32_t *) 0x50810f00, input_24, 7200);
//   memcpy32((uint32_t *) 0x50818f00, input_28, 7200);
//   memcpy32((uint32_t *) 0x50c00f00, input_32, 7200);
//   memcpy32((uint32_t *) 0x50c08f00, input_36, 7200);
//   memcpy32((uint32_t *) 0x50c10f00, input_40, 7200);
//   memcpy32((uint32_t *) 0x50c18f00, input_44, 7200);
// }

extern unsigned int utils_get_time_ms(void);

void capture_and_display_camera(void)
{
    uint32_t imgLen;
    uint32_t w, h;
    uint8_t *raw;
    unsigned int start_time;

    start_time = utils_get_time_ms();

    // Capture the image
    camera_start_capture_image();
/* Sleep until camera interrupt */
//MXC_LP_EnterSleepMode();
#ifdef TFT_ENABLE
    MXC_TFT_Stream(X_START, Y_START, IMAGE_XRES, IMAGE_YRES);
#endif
    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);
#ifdef TFT_ENABLE
    setup_dma_tft((uint32_t *)raw);
#endif
    // Wait to complete image capture
    while (camera_is_image_rcv() == 0) {}
    printf("Image capture: %d ms\n", utils_get_time_ms() - start_time);

    start_time = utils_get_time_ms();
#ifdef TFT_ENABLE
    // Send a first half of captured image to TFT
    start_tft_dma((uint32_t *)raw);
    // Wait for DMA to finish
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }
    setup_dma_tft((uint32_t *)(raw + IMAGE_XRES * IMAGE_YRES));
    // Send a second half of captured image to TFT
    start_tft_dma((uint32_t *)(raw + IMAGE_XRES * IMAGE_YRES));
    // Wait for DMA to finish
    while ((MXC_DMA->ch[g_dma_channel_tft].status & MXC_F_DMA_STATUS_STATUS)) {
        ;
    }
    printf("Display image: %d ms\n", utils_get_time_ms() - start_time);
#endif
}

void load_input_camera(int frame_no)
{
    uint32_t imgLen;
    uint32_t w, h;
    uint8_t *raw;
    uint32_t start_time;

    start_time = utils_get_time_ms();

    // Initialize loading rows to CNN
    load_row_cnn_init(frame_no);
    // Load image by rows into CNN memory
    camera_get_image(&raw, &imgLen, &w, &h);
    load_cnn(raw, h);
    printf("CNN Load %d: %d ms\n", frame_no, utils_get_time_ms() - start_time);
}
