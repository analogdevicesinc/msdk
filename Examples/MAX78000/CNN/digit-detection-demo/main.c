/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
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
 ******************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "gcfr_regs.h"
#include "fcr_regs.h"
#include "icc.h"
#include "dma.h"
#include "led.h"
#include "tmr.h"
#include "pb.h"
#include "cnn.h"
#include "weights.h"
#include "sampledata.h"
#include "mxc_delay.h"
#include "camera.h"
#include "spi.h"
#include "mxc.h"
#include "mxc_device.h"
#include "board.h"
#include "rtc.h"
#include "uart.h"
#include "tft_utils.h"
#include "post_process.h"
#ifdef BOARD_EVKIT_V1
#include "bitmap.h"
#include "tft_ssd2119.h"
#endif
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#include "example_config.h"

#ifdef BOARD_EVKIT_V1
int font = urw_gothic_12_grey_bg_white;
#endif
#ifdef BOARD_FTHR_REVA
int font = (int)&Liberation_Sans16x16[0];
#endif
volatile uint32_t cnn_time; // Stopwatch

// 3-channel 74x74 data input (16428 bytes total / 5476 bytes per channel):
// HWC 74x74, channels 0 to 2
#ifdef USE_SAMPLEDATA //Sample DATA
static uint32_t input_buffer[] = SAMPLE_INPUT_0;
uint32_t *input = input_buffer;
#endif

#if defined(RGB565) && defined(BOARD_EVKIT_V1)
uint8_t data565[CAMERA_SIZE_X * 2];
#endif

extern int g_dma_channel_tft;
void fail(void)
{
    printf("\n*** FAIL ***\n\n");

    while (1) {}
}

void load_input_display_RGB888(void)
{
#ifdef USE_SAMPLEDATA
    // This function loads the sample data input -- replace with actual data
    memcpy32((uint32_t *)0x50402000, input, IMAGE_SIZE_X * IMAGE_SIZE_Y);

#ifdef TFT_ENABLE
    uint8_t r, g, b;
    uint32_t x, y;
    uint32_t color;
    uint8_t *buffer = (uint8_t *)input;
    for (y = 0; y < IMAGE_SIZE_Y; y++) {
        for (x = 0; x < IMAGE_SIZE_X; x++) {
            r = *buffer++;
            g = *buffer++;
            b = *buffer++;
            buffer++; // skip msb=0x00

            // display on TFT
#ifdef BOARD_EVKIT_V1
            color = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) |
                     (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
            // Convert to RGB565
            color = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
#endif
            MXC_TFT_WritePixel(x * IMG_SCALE + TFT_X_OFFSET, y * IMG_SCALE, IMG_SCALE, IMG_SCALE,
                               color);
        }
    }

#endif

#else // Camera
    uint8_t *frame_buffer;
    uint8_t *buffer;
    uint32_t imgLen;
    uint32_t w, h, x, y;
    uint8_t r, g, b;

    uint32_t *cnn_mem = (uint32_t *)0x50402000;

    camera_start_capture_image();

    while (!camera_is_image_rcv()) {}

    camera_get_image(&frame_buffer, &imgLen, &w, &h);
    buffer = frame_buffer;

    printf("Width:%d Height:%d\n", w, h);

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            r = *buffer++;
            g = *buffer++;
            b = *buffer++;
            buffer++; // skip msb=0x00
            // change the range from [0,255] to [-128,127] and store in buffer for CNN
            *cnn_mem++ = ((b << 16) | (g << 8) | r) ^ 0x00808080;

            // display on TFT
#ifdef TFT_ENABLE
            uint32_t color;
#ifdef BOARD_EVKIT_V1
            color = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) |
                     (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
            // Convert to RGB565
            color = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
#endif
            MXC_TFT_WritePixel(x * IMG_SCALE + TFT_X_OFFSET, y * IMG_SCALE, IMG_SCALE, IMG_SCALE,
                               color);
#endif
        }
    }
#endif //#ifdef USE_SAMPLEDATA
}

#ifdef RGB565
void load_input_RGB565(void)
{
    static stream_stat_t *stat;
    uint8_t *frame_buffer = NULL;
    uint8_t *buffer;
    uint32_t imgLen;
    uint32_t w, h, x, y;

    uint32_t *cnn_mem = (uint32_t *)0x50402000;

    union {
        uint32_t w;
        uint8_t b[4];
    } m;

    camera_start_capture_image();

    camera_get_image(&buffer, &imgLen, &w, &h);

    for (y = 0; y < h; y++) {
        // Wait until camera streaming buffer is full
        while ((frame_buffer = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        };

        if ((y % IMG_SCALE) != 0) { // down-sample
            release_camera_stream_buffer();
            continue;
        }

        for (x = 0; x < w; x += IMG_SCALE) { // down-sample camera images

            // RGB565 to packed 24-bit RGB
            m.b[0] = (*frame_buffer & 0xF8); // Red
            m.b[1] = (*frame_buffer << 5) | ((*(frame_buffer + 1) & 0xE0) >> 3); // Green
            m.b[2] = (*(frame_buffer + 1) << 3); // Blue

            frame_buffer += 2 * IMG_SCALE;
            *cnn_mem++ = m.w ^ 0x00808080U;
        }
        // Release stream buffer
        release_camera_stream_buffer();
    }

    stat = get_camera_stream_statistic();
    if (stat->overflow_count > 0) {
        printf("OVERFLOW CNN = %d\n", stat->overflow_count);
        LED_On(LED2); // Turn on red LED if overflow detected
        while (1) {}
    }
}

void display_camera_RGB565(void)
{
    static stream_stat_t *stat;

    uint8_t *frame_buffer = NULL;
    uint8_t *buffer;
    uint32_t imgLen;
    uint32_t w, h, y;

    // display
    camera_start_capture_image();

    camera_get_image(&buffer, &imgLen, &w, &h);

    printf("W:%d H:%d L:%d \n", w, h, imgLen);

#ifdef BOARD_FTHR_REVA
    // Initialize FTHR TFT for DMA streaming
    MXC_TFT_Stream(TFT_X_OFFSET, 0, w, h);
#endif

    for (y = 0; y < h; y++) {
        // Wait until camera streaming buffer is full
        while ((frame_buffer = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        };

#ifdef BOARD_EVKIT_V1
        int j = 0;
        for (int k = 2 * w - 1; k > 0; k -= 2) { // reverse order to display

            data565[j++] = frame_buffer[k + 1];
            data565[j++] = frame_buffer[k];
        }

        MXC_TFT_ShowImageCameraRGB565(TFT_X_OFFSET, y, data565, w, 1);
#endif
#ifdef BOARD_FTHR_REVA
        tft_dma_display(TFT_X_OFFSET, y, w, 1, (uint32_t *)frame_buffer);
#endif

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
#endif

void cnn_wait(void)
{
    while ((*((volatile uint32_t *)0x50100000) & (1 << 12)) != 1 << 12) {}

    CNN_COMPLETE; // Signal that processing is complete
    cnn_time = MXC_TMR_SW_Stop(MXC_TMR0);
}

uint32_t utils_get_time_ms(void)
{
    uint32_t sec, ssec;
    double subsec;
    uint32_t ms;
    MXC_RTC_GetSubSeconds(&ssec);
    subsec = ssec / 4096.0;
    MXC_RTC_GetSeconds(&sec);

    ms = (sec * 1000) + (int)(subsec * 1000);

    return ms;
}

int main(void)
{
#ifdef TFT_ENABLE
    char buff[TFT_BUFF_SIZE];
#endif
    static uint32_t t1, t2, t3, t4, t5, t6;

#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
    /* Enable camera power */
    Camera_Power(POWER_ON);
    printf("\n\nDigit Detection Feather Demo\n");
#else
    printf("\n\nDigit Detection Evkit Demo\n");
#endif

    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("Waiting...\n");

    // Initialize RTC
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed

    // Configure P2.5, turn on the CNN Boost
    mxc_gpio_cfg_t gpio_out;
    gpio_out.port = MXC_GPIO2;
    gpio_out.mask = MXC_GPIO_PIN_5;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    gpio_out.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_out.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_out);
    MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

#ifdef TFT_ENABLE
    // Initialize TFT display.
    printf("Init LCD...");

#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
#endif

#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_270);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif

    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 80, 30, font, snprintf(buff, sizeof(buff), "ANALOG DEVICES             "));
    TFT_Print(buff, 55, 50, font, snprintf(buff, sizeof(buff), "Digit Detection Demo      "));
    TFT_Print(buff, 120, 90, font, snprintf(buff, sizeof(buff), "Ver. 1.1.0                   "));
    MXC_Delay(SEC(2));

#ifdef BOARD_EVKIT_V1
    MXC_TFT_SetBackGroundColor(255);
#endif

#endif //#ifdef TFT_ENABLE

    int xx = IMAGE_SIZE_X;
    int yy = IMAGE_SIZE_Y;
    printf("x %d  y %d\n", xx, yy);

#if !defined(USE_SAMPLEDATA)
    int dma_channel;
    // Initialize camera.
    printf("Init Camera...\n");

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    camera_init(CAMERA_FREQ);

#ifndef RGB565
    int ret = camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, USE_DMA,
                           dma_channel);
#else
    int ret = camera_setup(CAMERA_SIZE_X, CAMERA_SIZE_Y, PIXFORMAT_RGB565, FIFO_FOUR_BYTE,
                           STREAMING_DMA, dma_channel);
    // set camera clock prescaler to prevent streaming overflow due to TFT display latency
#ifdef BOARD_EVKIT_V1
    camera_write_reg(0x11, 0x3);
#endif

#ifdef BOARD_FTHR_REVA
    camera_write_reg(0x11, 0x0);
#endif

#endif //#ifndef RGB565

    if (ret != STATUS_OK) {
        printf("\tError returned from setting up camera. Error %d\n", ret);
        return -1;
    }

#else //#if !defined(USE_SAMPLEDATA)
    printf("Using Sample Data!\n");
#endif

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: 50 MHz div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels

#ifdef TFT_ENABLE
    MXC_TFT_ClearScreen();
#endif

    while (1) {
        t1 = utils_get_time_ms();

        // Reload bias after wakeup
        cnn_init(); // Bring state machine into consistent state
        cnn_load_bias();
        cnn_configure(); // Configure state machine

#if defined(USE_SAMPLEDATA) || !defined(RGB565)
        // capture one frame, load to CNN and display on TFT pixel by pixel
        load_input_display_RGB888();
#else
        // capture image row by row, convert and load it to CNN
        load_input_RGB565(); // Load data input
#endif

        t2 = utils_get_time_ms();

        LED_On(LED1);

        cnn_start(); // Start CNN processing

#if defined(TFT_ENABLE) && defined(RGB565) && !defined(USE_SAMPLEDATA)
        display_camera_RGB565();
#endif

        t3 = utils_get_time_ms();

        while (cnn_time == 0) {
            __WFI(); // Wait for CNN
        }

        t4 = utils_get_time_ms();

        LED_Off(LED1);
        get_priors();

        t5 = utils_get_time_ms();

        localize_objects();

        t6 = utils_get_time_ms();

        printf("CNN time: %d us\n\n", cnn_time);
#ifdef TFT_ENABLE
        TFT_Print(buff, 10 + TFT_X_OFFSET, 215, font,
                  snprintf(buff, sizeof(buff), "CNN Time: %.3f ms   ", (double)cnn_time / 1000));
#endif

        // print timing data
        printf("load:%d TFT:%d cnn_wait:%d cnn_unload:%d localize:%d Total:%dms\n", t2 - t1,
               t3 - t2, t4 - t3, t5 - t4, t6 - t5, t6 - t1);
        MXC_Delay(SEC(1));
    }

    return 0;
}
