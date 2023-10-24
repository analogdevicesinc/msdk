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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "uart.h"
#include "led.h"
#include "board.h"

#include "camera.h"
#include "dma.h"
#include "camera_util.h"

#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#endif
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

uint8_t data565[IMAGE_XRES * 2];
// *****************************************************************************

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
#ifndef RGB565
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB888, FIFO_THREE_BYTE, STREAMING_DMA,
                       dma_channel); // RGB888 with 0 at MSB stream
#else
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, STREAMING_DMA,
                       dma_channel);
#endif

    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

    MXC_Delay(SEC(1));

#if defined(CAMERA_OV7692) && defined(STREAM_ENABLE)
    // set camera clock prescaler to prevent streaming overflow due to TFT display latency
    camera_write_reg(0x11, 0x1);
#endif

    // make the scale ratio of camera input size the same as output size, make is faster and regular
    camera_write_reg(0xc8, 0x1);
    camera_write_reg(0xc9, 0x60);
    camera_write_reg(0xca, 0x1);
    camera_write_reg(0xcb, 0x60);

    return ret;
}

void load_row_cnn_init(void)
{
    data_addr0 = (uint32_t *)0x50400700;
    data_addr3 = (uint32_t *)0x50418700;
    data_addr6 = (uint32_t *)0x50810700;
    data_addr9 = (uint32_t *)0x50c08700;
    addr = data_addr9;
}

void load_row_cnn(uint8_t *data, int row)
{
    union {
        uint32_t w;
        uint8_t b[4];
    } m;

    offset0 = 0x00002000;
    offset1 = 0x00002000;
    subtract = 0x00004000 - 1;

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

    // indexes of 352x352 image (row,j)
    for (int j = 0; j < IMAGE_XRES; j += 4) {
#if defined(PATTERN_GEN)
        // static test pattern
        m.b[0] = 133; // r0
        m.b[1] = j >> 1; // g0
        m.b[2] = row >> 1; // b0
        m.b[3] = 133; // r1

        *addr = m.w ^ 0x80808080U;
        addr += offset0;

        m.b[0] = (j + 1) >> 1; // g1
        m.b[1] = row >> 1; // b1
        m.b[2] = 133; // r2
        m.b[3] = (j + 2) >> 1; // g2

        *addr = m.w ^ 0x80808080U;
        addr += offset1;

        m.b[0] = row >> 1; // b2
        m.b[1] = 133; // r3
        m.b[2] = (j + 3) >> 1; // g3
        m.b[3] = row >> 1; // b3

        *addr = m.w ^ 0x80808080U;
        addr -= subtract;
#elif defined(RGB565)
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
#else
        // unpacked 24-bit RGB to packed 24-bit RGB
        m.b[0] = *data++; // r0
        m.b[1] = *data++; // g0
        m.b[2] = *data++; // b0
        data++; // skip MSB
        m.b[3] = *data++; // r1

        *addr = m.w ^ 0x80808080U;
        addr += offset0;

        m.b[0] = *data++; // g1
        m.b[1] = *data++; // b1
        data++; // skip MSB
        m.b[2] = *data++; // r2
        m.b[3] = *data++; // g2

        *addr = m.w ^ 0x80808080U;
        addr += offset1;

        m.b[0] = *data++; // b2
        data++; // skip MSB
        m.b[1] = *data++; // r3
        m.b[2] = *data++; // g3
        m.b[3] = *data++; // b3
        data++; // skip MSB

        *addr = m.w ^ 0x80808080U;
        addr -= subtract;
#endif
    }
}

/*
  Data Order:
  camera data: (352,352,3), following indexes are based on camera pixel index

  0x50400700:
      (0,1,0)|(0,0,2)|(0,0,1)|(0,0,0)              // 0
      (0,5,0)|(0,4,2)|(0,4,1)|(0,4,0)              // 1
      ...
      (0,349,0)|(0,348,2)|(0,348,1)|(0,348,0)      // 87

      (4,1,0)|(4,0,2)|(4,0,1)|(4,0,0)              // 88
      (4,5,0)|(4,4,2)|(4,4,1)|(4,4,0)
      ...
      (4,349,0)|(4,348,2)|(4,348,1)|(4,348,0)       // 175
      ...
      ...
      ...
      (348,1,0)|(348,0,2)|(348,0,1)|(348,0,0)              //
      (348,5,0)|(348,4,2)|(348,4,1)|(348,4,0)
      ...
      (348,349,0)|(348,348,2)|(348,348,1)|(348,348,0)       // 7743

  0x50408700:
      (0,2,1)|(0,2,0)|(0,1,2)|(0,1,1)              // 0
      (0,6,1)|(0,6,0)|(0,5,2)|(0,5,1)              // 1
      ...
      (0,350,1)|(0,350,0)|(0,349,2)|(0,349,1)      // 87

      (4,2,1)|(4,2,0)|(4,1,2)|(4,1,1)              // 88
      (4,6,1)|(4,6,0)|(4,5,2)|(4,5,1)
      ...
      (4,350,1)|(4,350,0)|(4,349,2)|(4,349,1)      // 175
      ...
      ...
      ...
      (348,2,1)|(348,2,0)|(348,1,2)|(348,1,1)              //
      (348,6,1)|(348,6,0)|(348,5,2)|(348,5,1)
      ...
      (348,350,1)|(348,350,0)|(348,349,2)|(348,349,1)      // 7743

  0x50410700:
      (0,3,2)|(0,3,1)|(0,3,0)|(0,2,2)              // 0
      (0,7,2)|(0,7,1)|(0,7,0)|(0,6,2)              // 1
      ...
      (0,351,2)|(0,351,1)|(0,351,0)|(0,350,2)      // 87

      ...
      ...
      ...
      (348,3,2)|(348,3,1)|(348,3,0)|(348,2,2)              //
      (348,7,2)|(348,7,1)|(348,7,0)|(348,6,2)
      ...
      (348,351,2)|(348,351,1)|(348,351,0)|(348,350,2)      // 7743


 The same pattern of 3x7744 words repeats another 3 times, with starting row index changed from 0 to 1, then 2 and then 3
 resulting in 4x3x7744 words:
 ....
    0x50c18700: last bank
    ...
      (351,351,2)|(351,351,1)|(351,351,0)|(351,350,2)      // 7743


   */

// This function loads the sample data input -- replace with actual data

/*
memcpy32((uint32_t *) 0x50400700, input_0, 7744);
memcpy32((uint32_t *) 0x50408700, input_4, 7744);
memcpy32((uint32_t *) 0x50410700, input_8, 7744);
memcpy32((uint32_t *) 0x50418700, input_12, 7744);
memcpy32((uint32_t *) 0x50800700, input_16, 7744);
memcpy32((uint32_t *) 0x50808700, input_20, 7744);
memcpy32((uint32_t *) 0x50810700, input_24, 7744);
memcpy32((uint32_t *) 0x50818700, input_28, 7744);
memcpy32((uint32_t *) 0x50c00700, input_32, 7744);
memcpy32((uint32_t *) 0x50c08700, input_36, 7744);
memcpy32((uint32_t *) 0x50c10700, input_40, 7744);
memcpy32((uint32_t *) 0x50c18700, input_44, 7744);
*/

// STREAM mode
uint8_t *data = NULL;
stream_stat_t *stat;
void load_input_camera(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    // Initialize loading rows to CNN
    load_row_cnn_init();

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    // Get image line by line
    for (int row = 0; row < h; row++) {
        // Wait until camera streaming buffer is full
        while ((data = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        };

        LED_Toggle(LED2);

        load_row_cnn(data, row);

        LED_Toggle(LED2);

        // Release stream buffer
        release_camera_stream_buffer();
    }

    stat = get_camera_stream_statistic();
    if (stat->overflow_count > 0) {
        printf("OVERFLOW Loading= %d\n", stat->overflow_count);
        LED_On(LED2); // Turn on red LED if overflow detected

        while (1) {}
    }
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

void display_camera(void)
{
    uint32_t imgLen;
    uint32_t w, h;

    uint8_t *raw;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    // Send image line by line TFT
    // Only display TFT width
    if (w > TFT_W) {
        w = TFT_W;
    }

    // Get image line by line
    for (int row = 0; row < h; row++) {
        // Wait until camera streaming buffer is full
        while ((data = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        };

        LED_Toggle(LED2);

        // convert RGB888 to RGB565
        if (row < TFT_H) {
#ifndef RGB565
            uint16_t rgb;
            uint8_t r, g, b;
            int j = 0;
#ifdef BOARD_FTHR_REVA

            for (int k = 0; k < 4 * w; k += 4) {
#endif
#ifdef BOARD_EVKIT_V1

                for (int k = 4 * w - 1; k > 0; k -= 4) { // reverse order to display
#endif
                    r = data[k];
                    g = data[k + 1];
                    b = data[k + 2];
                    //skip k+3
                    rgb = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
                    data565[j++] = (rgb >> 8) & 0xFF;
                    data565[j++] = rgb & 0xFF;
                }

#ifdef BOARD_EVKIT_V1
                MXC_TFT_ShowImageCameraRGB565(0, Y_START + row, data565, w, 1);
#else
        tft_dma_display(0, Y_START + row, TFT_W, 1, (uint32_t *)data565);
#endif

#else //#ifndef RGB565

#ifdef BOARD_EVKIT_V1
            int j = 0;
            for (int k = 2 * w - 1; k > 0; k -= 2) { // reverse order to display

                data565[j++] = data[k + 1];
                data565[j++] = data[k];
            }

            MXC_TFT_ShowImageCameraRGB565(0, Y_START + row, data565, w, 1);

#else
            tft_dma_display(0, Y_START + row, TFT_W, 1, (uint32_t *)data);
#endif

#endif //#ifndef RGB565
            }

            LED_Toggle(LED2);
            // Release stream buffer
            release_camera_stream_buffer();
        }

        stat = get_camera_stream_statistic();
        if (stat->overflow_count > 0) {
            printf("OVERFLOW DISP = %d\n", stat->overflow_count);
            LED_On(LED2); // Turn on red LED if overflow detected

            while (1) {}
        }
    }

    static uint32_t sum = 0;
    void dump_cnn(void)
    {
        uint32_t *data_addr[12] = { (uint32_t *)0x50400700, (uint32_t *)0x50408700,
                                    (uint32_t *)0x50410700, (uint32_t *)0x50418700,
                                    (uint32_t *)0x50800700, (uint32_t *)0x50808700,
                                    (uint32_t *)0x50810700, (uint32_t *)0x50818700,
                                    (uint32_t *)0x50c00700, (uint32_t *)0x50c08700,
                                    (uint32_t *)0x50c10700, (uint32_t *)0x50c18700 };

        printf("\nDUMPING CNN, press PB0 \n");

        while (!PB_Get(0)) {}

        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 7744; j += 16) {
                printf("\n%08X: ", data_addr[i]);

                for (int k = 0; k < 16; k++) {
                    printf("%08X ", *data_addr[i]);
                    sum += *data_addr[i];
                    data_addr[i]++;
                }
            }

            printf("\n");
        }

        printf("SUM: %08X \n", sum);

        while (1) {}
    }

    void dump_inference(void)
    {
        uint32_t *data_addr[16] = {
            (uint32_t *)0x50400000, (uint32_t *)0x50408000, (uint32_t *)0x50410000,
            (uint32_t *)0x50418000, (uint32_t *)0x50800000, (uint32_t *)0x50808000,
            (uint32_t *)0x50810000, (uint32_t *)0x50818000, (uint32_t *)0x50c00000,
            (uint32_t *)0x50c08000, (uint32_t *)0x50c10000, (uint32_t *)0x50c18000,
            (uint32_t *)0x51000000, (uint32_t *)0x51008000, (uint32_t *)0x51010000,
            (uint32_t *)0x51018000,
        };

        printf("\nDUMPING INFERENCE, press PB0 \n");

        while (!PB_Get(0)) {}

        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 7744; j += 16) {
                printf("\n%08X: ", data_addr[i]);

                for (int k = 0; k < 16; k++) {
                    printf("%08X ", *data_addr[i]);
                    sum += *data_addr[i];
                    data_addr[i]++;
                }
            }

            printf("\n");
        }

        printf("SUM: %08X \n", sum);

        while (1) {}
    }
