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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "fcr_regs.h"
#include "icc.h"
#include "led.h"
#include "tmr.h"
#include "dma.h"
#include "pb.h"
#include "cnn.h"
#include "weights.h"
#include "sampledata.h"
#include "mxc_delay.h"
#include "camera.h"
#ifdef BOARD_EVKIT_V1
#include "bitmap.h"
#include "tft_ssd2119.h"
#endif
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

// Comment out USE_SAMPLEDATA to use Camera module
//#define USE_SAMPLEDATA

#define ASCII_ART

#define IMAGE_SIZE_X (64 * 2)
#define IMAGE_SIZE_Y (64 * 2)

#define TFT_X_START 100
#define TFT_Y_START 50

#define CAMERA_FREQ (5 * 1000 * 1000)

#define TFT_BUFF_SIZE 30 // TFT buffer size

#ifdef BOARD_EVKIT_V1
int image_bitmap_1 = ADI_256_bmp;
int image_bitmap_2 = logo_white_bg_darkgrey_bmp;
int font_1 = urw_gothic_12_white_bg_grey;
int font_2 = urw_gothic_13_white_bg_grey;
#endif
#ifdef BOARD_FTHR_REVA
int image_bitmap_1 = (int)&img_1_rgb565[0];
int image_bitmap_2 = (int)&logo_rgb565[0];
int font_1 = (int)&SansSerif16x16[0];
int font_2 = (int)&SansSerif16x16[0];
#endif

const char classes[CNN_NUM_OUTPUTS][10] = { "Cat", "Dog" };

// Classification layer:
static int32_t ml_data[CNN_NUM_OUTPUTS];
static q15_t ml_softmax[CNN_NUM_OUTPUTS];

volatile uint32_t cnn_time; // Stopwatch

// RGB565 buffer for TFT
uint8_t data565[IMAGE_SIZE_X * 2];

#ifdef USE_SAMPLEDATA
// Data input: HWC 3x128x128 (49152 bytes total / 16384 bytes per channel):
static const uint32_t input_0[] = SAMPLE_INPUT_0; // input data from header file
#else
static uint32_t input_0[IMAGE_SIZE_X * IMAGE_SIZE_Y]; // buffer for camera image
#endif

/* **************************************************************************** */
#ifdef ASCII_ART

//char * brightness = "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\"^`'. "; // standard
char *brightness = "@%#*+=-:. "; // simple
#define RATIO 2 // ratio of scaling down the image to display in ascii
void asciiart(uint8_t *img)
{
    int skip_x, skip_y;
    uint8_t r, g, b, Y;
    uint8_t *srcPtr = img;
    int l = strlen(brightness) - 1;

    skip_x = RATIO;
    skip_y = RATIO;
    for (int i = 0; i < IMAGE_SIZE_Y; i++) {
        for (int j = 0; j < IMAGE_SIZE_X; j++) {
            // 0x00bbggrr, convert to [0,255] range
            r = *srcPtr++ ^ 0x80;
            g = *(srcPtr++) ^ 0x80;
            b = *(srcPtr++) ^ 0x80;

            srcPtr++; //skip msb=0x00

            // Y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
            Y = (3 * r + b + 4 * g) >> 3; // simple luminance conversion
            if ((skip_x == RATIO) && (skip_y == RATIO))
                printf("%c", brightness[l - (Y * l / 255)]);

            skip_x++;
            if (skip_x > RATIO)
                skip_x = 1;
        }
        skip_y++;
        if (skip_y > RATIO) {
            printf("\n");
            skip_y = 1;
        }
    }
}

#endif

/* **************************************************************************** */

void TFT_Print(char *str, int x, int y, int font, int length)
{
#ifdef TFT_ENABLE
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;
    MXC_TFT_PrintFont(x, y, font, &text, NULL);
#endif
}

/* **************************************************************************** */
void fail(void)
{
    printf("\n*** FAIL ***\n\n");

    while (1) {}
}

/* **************************************************************************** */
void cnn_load_input(void)
{
    int i;
    const uint32_t *in0 = input_0;

    for (i = 0; i < 16384; i++) {
        // Remove the following line if there is no risk that the source would overrun the FIFO:
        while (((*((volatile uint32_t *)0x50000004) & 1)) != 0) {}
        // Wait for FIFO 0
        *((volatile uint32_t *)0x50000008) = *in0++; // Write FIFO 0
    }
}

/* **************************************************************************** */
#if defined USE_SAMPLEDATA && defined TFT_ENABLE
void display_sampledata(void)
{
#ifdef TFT_ENABLE
    uint32_t w;
    uint8_t r, g, b;
    uint16_t rgb;

    int j = 0;
    uint32_t temp;

    int cnt = 0;
    {
    }
    w = IMAGE_SIZE_X;

    // Get image line by line
    for (int row = 0; row < IMAGE_SIZE_Y; row++) {
        //LED_Toggle(LED2);
#ifdef BOARD_EVKIT_V1
        j = IMAGE_SIZE_X * 2 - 2; // mirror on display
#else
        j = 0;
#endif

        for (int k = 0; k < 4 * w; k += 4) {
            // sample data is already in [-128,127] range, make it [0,255] for display
            temp = input_0[cnt] ^ 0x00808080;

            // data format: 0x00bbggrr
            r = temp & 0xFF;
            g = (temp >> 8) & 0xFF;
            b = (temp >> 16) & 0xFF;
            cnt++;

            // convert to RGB656 for display
            rgb = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
            data565[j] = (rgb >> 8) & 0xFF;
            data565[j + 1] = rgb & 0xFF;

#ifdef BOARD_EVKIT_V1
            j -= 2; // mirror on display
#else
            j += 2;
#endif
        }

        MXC_TFT_ShowImageCameraRGB565(TFT_X_START, TFT_Y_START + row, data565, w, 1);

        LED_Toggle(LED2);
    }
#endif
}

#elif !defined USE_SAMPLEDATA
void capture_process_camera(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    int cnt = 0;

    uint8_t r, g, b;
    uint16_t rgb;
    int j = 0;

    uint8_t *data = NULL;
    stream_stat_t *stat;

    camera_start_capture_image();

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    // Get image line by line
    for (int row = 0; row < h; row++) {
        // Wait until camera streaming buffer is full
        while ((data = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        }

        //LED_Toggle(LED2);
#ifdef BOARD_EVKIT_V1
        j = IMAGE_SIZE_X * 2 - 2; // mirror on display
#else
        j = 0;
#endif
        for (int k = 0; k < 4 * w; k += 4) {
            // data format: 0x00bbggrr
            r = data[k];
            g = data[k + 1];
            b = data[k + 2];
            //skip k+3

            // change the range from [0,255] to [-128,127] and store in buffer for CNN
            input_0[cnt++] = ((b << 16) | (g << 8) | r) ^ 0x00808080;

            // convert to RGB656 for display
            rgb = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
            data565[j] = (rgb >> 8) & 0xFF;
            data565[j + 1] = rgb & 0xFF;
#ifdef BOARD_EVKIT_V1
            j -= 2; // mirror on display
#else
            j += 2;
#endif
        }
#ifdef TFT_ENABLE
        MXC_TFT_ShowImageCameraRGB565(TFT_X_START, TFT_Y_START + row, data565, w, 1);
#endif

        //LED_Toggle(LED2);
        // Release stream buffer
        release_camera_stream_buffer();
    }

    //camera_sleep(1);
    stat = get_camera_stream_statistic();

    if (stat->overflow_count > 0) {
        printf("OVERFLOW DISP = %d\n", stat->overflow_count);
        LED_On(LED2); // Turn on red LED if overflow detected
        while (1) {}
    }
}
#endif

/* **************************************************************************** */
int main(void)
{
    int i;
    int digs, tens;
    int ret = 0;
    int result[CNN_NUM_OUTPUTS]; // = {0};
    int dma_channel;

    char buff[TFT_BUFF_SIZE];

#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
    /* Enable camera power */
    Camera_Power(POWER_ON);
    //MXC_Delay(300000);
    printf("\n\nCats-vs-Dogs Feather Demo\n");
#else
    printf("\n\nCats-vs-Dogs Evkit Demo\n");
#endif

    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Switch to 100 MHz clock */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    /* Enable peripheral, enable CNN interrupt, turn on CNN clock */
    /* CNN clock: 50 MHz div 1 */
    cnn_enable((0 << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS), MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    /* Configure P2.5, turn on the CNN Boost */
    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5);

    /* Bring CNN state machine into consistent state */
    cnn_init();
    /* Load CNN kernels */
    cnn_load_weights();
    /* Load CNN bias */
    cnn_load_bias();
    /* Configure CNN state machine */
    cnn_configure();

#ifdef TFT_ENABLE
    /* Initialize TFT display */
    printf("Init LCD.\n");
#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
    MXC_TFT_ClearScreen();
    MXC_TFT_ShowImage(0, 0, image_bitmap_1);
#endif
#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_270);

    MXC_TFT_ShowImage(0, 0, image_bitmap_1);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif
    MXC_Delay(1000000);
#endif

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    // Initialize camera.
    printf("Init Camera.\n");
    camera_init(CAMERA_FREQ);

    ret = camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, STREAMING_DMA,
                       dma_channel);
    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

#ifdef BOARD_EVKIT_V1
    camera_write_reg(0x11, 0x1); // set camera clock prescaller to prevent streaming overflow
#else
    camera_write_reg(0x11, 0x3); // set camera clock prescaller to prevent streaming overflow
#endif

#ifdef TFT_ENABLE
    MXC_TFT_SetPalette(image_bitmap_2);
    MXC_TFT_SetBackGroundColor(4);
    //MXC_TFT_ShowImage(1, 1, image_bitmap_2);
    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 55, 50, font_2, sprintf(buff, "ANALOG DEVICES"));
    TFT_Print(buff, 55, 90, font_1, sprintf(buff, "Cats-vs-Dogs Demo"));
    TFT_Print(buff, 30, 130, font_2, sprintf(buff, "PRESS PB1(SW1) TO START!"));
#endif

    printf("********** Press PB1(SW1) to capture an image **********\r\n");
    while (!PB_Get(0)) {}

#ifdef TFT_ENABLE
    MXC_TFT_ClearScreen();
#endif

    // Enable CNN clock
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN);

    while (1) {
        LED_Off(LED1);
        LED_Off(LED2);
#ifdef USE_SAMPLEDATA
#ifdef TFT_ENABLE
        display_sampledata();
#endif
#else
        capture_process_camera();
#endif

        cnn_start();
        cnn_load_input();

        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0
        while (cnn_time == 0) {
            __WFI(); // Wait for CNN interrupt
        }

        // Unload CNN data
        cnn_unload((uint32_t *)ml_data);
        cnn_stop();

        // Softmax
        softmax_q17p14_q15((const q31_t *)ml_data, CNN_NUM_OUTPUTS, ml_softmax);

        printf("Time for CNN: %d us\n\n", cnn_time);

        printf("Classification results:\n");

        for (i = 0; i < CNN_NUM_OUTPUTS; i++) {
            digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
            tens = digs % 10;
            digs = digs / 10;
            result[i] = digs;
            printf("[%7d] -> Class %d %8s: %d.%d%%\r\n", ml_data[i], i, classes[i], result[i],
                   tens);
        }

        printf("\n");

#ifdef TFT_ENABLE

        area_t area;
        area.x = 0;
        area.y = 0;
        area.w = 320;
        area.h = TFT_Y_START - 1;
        MXC_TFT_ClearArea(&area, 4);

        memset(buff, 32, TFT_BUFF_SIZE);
#endif
        if (result[0] == result[1]) {
            TFT_Print(buff, TFT_X_START + 10, TFT_Y_START - 30, font_1, sprintf(buff, "Unknown"));
            LED_On(LED1);
            LED_On(LED2);

        } else if (ml_data[0] > ml_data[1]) {
            TFT_Print(buff, TFT_X_START + 10, TFT_Y_START - 30, font_1,
                      sprintf(buff, "%s (%d%%)", classes[0], result[0]));
            LED_On(LED1);
            LED_Off(LED2);
        } else {
            TFT_Print(buff, TFT_X_START + 10, TFT_Y_START - 30, font_1,
                      sprintf(buff, "%s (%d%%)", classes[1], result[1]));
            LED_Off(LED1);
            LED_On(LED2);
        }

        memset(buff, 32, TFT_BUFF_SIZE);
        TFT_Print(buff, TFT_X_START + 30, TFT_Y_START + IMAGE_SIZE_Y + 10, font_1,
                  sprintf(buff, "%dms", cnn_time / 1000));
        TFT_Print(buff, 20, TFT_Y_START + IMAGE_SIZE_Y + 35, font_2,
                  sprintf(buff, "PRESS PB1(SW1) TO CAPTURE"));

#ifdef ASCII_ART
        asciiart((uint8_t *)input_0);
        printf("********** Press PB1(SW1) to capture an image **********\r\n");
        while (!PB_Get(0)) {}
#endif
    }

    return 0;
}
