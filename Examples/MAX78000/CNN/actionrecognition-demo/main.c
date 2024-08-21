/******************************************************************************
 *
 * Copyright (C) 2019-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"
#include "rtc.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#include "camera.h"
#include "camera_util.h"
#include "project_config.h"

#ifdef BOARD_FTHR_REVA
int font = (int)&Liberation_Sans16x16[0];
int font_little = (int)&Liberation_Sans12x12[0];
#endif

static int32_t ml_data[CNN_NUM_OUTPUTS];
#define TFT_BUFF_SIZE 20 // TFT buffer size

volatile uint32_t cnn_time; // Stopwatch

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1) {}
}

// define 5 strings for the 5 classes, max 6 characters each
char *classes[6] = { "pullup", "pushup", "situp", "squat", "other" };

int cnn_display_decision(void)
{
    int32_t max = 0;
    int32_t max_index = 0;
    int32_t i;

    for (i = 0; i < CNN_NUM_OUTPUTS; i++) {
        printf("\n%s: %d\n", classes[i], ml_data[i]);
        if (ml_data[i] > max) {
            max = ml_data[i];
            max_index = i;
        }
    }
    printf("\nDecision: %d\n", max_index);
    return max_index;
}

unsigned int utils_get_time_ms(void)
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

#ifdef TFT_ENABLE
void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;

    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}
#endif

int main(void)
{
#ifdef TFT_ENABLE
    char buff[TFT_BUFF_SIZE];
#endif

#ifdef BOARD_FTHR_REVA
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(MSEC(200));
    /* Enable camera power */
    Camera_Power(POWER_ON);
    printf("\n\nTCN Action Demo - MAX78000 Feather\n");
#else
    printf("\n\nTCN Action Demo - MAX78000 EVKIT\n");
#endif

    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    // DO NOT DELETE THIS LINE:
    printf("Waiting ...\n");
    MXC_Delay(SEC(1)); // Let debugger interrupt if needed

    printf("\n\nInitializing Camera\n");
    initialize_camera();

#ifdef TFT_ENABLE
    // Initialize TFT display.
    printf("Init TFT\n");

#ifdef BOARD_FTHR_REVA

    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_270);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif

    MXC_TFT_ClearScreen();
    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 55, 30, font, snprintf(buff, sizeof(buff), "ANALOG"));
    TFT_Print(buff, 55, 55, font, snprintf(buff, sizeof(buff), "DEVICES"));
    TFT_Print(buff, 55, 80, font, snprintf(buff, sizeof(buff), "TCN Demo"));
    TFT_Print(buff, 55, 105, font, snprintf(buff, sizeof(buff), "Ver. 1.0.0"));
#endif

    MXC_Delay(SEC(1));

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: APB (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5); // Turn on the boost circuit

    if (cnn_init() != CNN_OK)
        fail();
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine

    unsigned int start_time;
    unsigned int unload_time;

    static q15_t ml_softmax[CNN_NUM_OUTPUTS];

    int i;
    int digs;
    int tens[CNN_NUM_OUTPUTS];
    int result[CNN_NUM_OUTPUTS]; // = {0};

#ifdef TFT_ENABLE
    TFT_Print(buff, 242, 20, font, snprintf(buff, sizeof(buff), classes[0]));
    TFT_Print(buff, 242, 60, font, snprintf(buff, sizeof(buff), classes[1]));
    TFT_Print(buff, 242, 100, font, snprintf(buff, sizeof(buff), classes[2]));
    TFT_Print(buff, 242, 140, font, snprintf(buff, sizeof(buff), classes[3]));
    TFT_Print(buff, 242, 180, font, snprintf(buff, sizeof(buff), classes[4]));

    area_t area_black = { 242, 40, 78, 15 };
#endif

    while (1) {
        printf("Loop starts \n");

        LED_Toggle(LED1);

        start_time = utils_get_time_ms();

        // capture image & display on TFT (frame 0)
        capture_and_display_camera();

        // make sure the two frame captures are at least 85 ms apart
        while (utils_get_time_ms() - start_time < 85) {
            MXC_Delay(MSEC(1));
        }

        // capture image & display on TFT (frame 1)
        capture_and_display_camera();
        // load image into CNN as frame 1
        load_input_camera(1);
        // inference
        cnn_start(); // Start CNN processing
        printf("Start CNN\n");

        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0
        while (cnn_time == 0) {
            __WFI(); // Wait for CNN
        }

#ifdef CNN_INFERENCE_TIMER
        printf("CNN time: %u us\n", cnn_time);
#endif

        // Unload CNN and display results
        printf("Unload CNN\n\n");
        unload_time = utils_get_time_ms();
        cnn_unload((uint32_t *)ml_data);

        softmax_q17p14_q15((const q31_t *)ml_data, CNN_NUM_OUTPUTS, ml_softmax);
        for (i = 0; i < CNN_NUM_OUTPUTS; i++) {
            digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
            tens[i] = digs % 10;
            digs = digs / 10;
            result[i] = digs;
            printf("Class %d %8s: %d.%d%%\r\n", i, classes[i], result[i], tens[i]);
        }

#ifdef TFT_ENABLE
        // display results as bar graphs
        area_black.y = 40;
        area_black.w = 78;
        MXC_TFT_FillRect(&area_black, BLACK);
        area_black.w = result[0] / 1.3;
        MXC_TFT_FillRect(&area_black, GREEN);

        area_black.y = 80;
        area_black.w = 78;
        MXC_TFT_FillRect(&area_black, BLACK);
        area_black.w = result[1] / 1.3;
        MXC_TFT_FillRect(&area_black, GREEN);

        area_black.y = 120;
        area_black.w = 78;
        MXC_TFT_FillRect(&area_black, BLACK);
        area_black.w = result[2] / 1.3;
        MXC_TFT_FillRect(&area_black, GREEN);

        area_black.y = 160;
        area_black.w = 78;
        MXC_TFT_FillRect(&area_black, BLACK);
        area_black.w = result[3] / 1.3;
        MXC_TFT_FillRect(&area_black, GREEN);

        area_black.y = 200;
        area_black.w = 78;
        MXC_TFT_FillRect(&area_black, BLACK);
        area_black.w = result[4] / 1.3;
        MXC_TFT_FillRect(&area_black, GREEN);
#endif

        printf("Unload/Decision time:%d ms\n", utils_get_time_ms() - unload_time);

        // load image into CNN as frame 0 (for next inference)
        load_input_camera(0);

        // make sure the loop takes at least 200 ms
        while (utils_get_time_ms() - start_time < 200) {
            MXC_Delay(MSEC(1));
        }

        printf("Total time:%d ms\n", utils_get_time_ms() - start_time);
        printf("---\n\n");
    }

    cnn_disable(); // Shut down CNN clock, disable peripheral

    return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 244,319,168 ops (243,294,688 macc; 1,006,368 comp; 18,112 add; 0 mul; 0 bitwise)
    Layer 0: 22,348,800 ops (22,118,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 132,940,800 ops (132,710,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 33,465,600 ops (33,177,600 macc; 288,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (res1_out): 33,235,200 ops (33,177,600 macc; 57,600 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (conv2): 8,366,400 ops (8,294,400 macc; 72,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5 (conv2_1): 936,000 ops (921,600 macc; 14,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (conv2_p): 8,366,400 ops (8,294,400 macc; 72,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7 (res2_out): 14,400 ops (0 macc; 0 comp; 14,400 add; 0 mul; 0 bitwise)
    Layer 8 (conv3): 1,822,016 ops (1,806,336 macc; 15,680 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (conv3_1): 203,840 ops (200,704 macc; 3,136 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (conv3_p): 1,822,016 ops (1,806,336 macc; 15,680 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (res3_out): 3,136 ops (0 macc; 0 comp; 3,136 add; 0 mul; 0 bitwise)
    Layer 12 (conv4): 334,656 ops (331,776 macc; 2,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13 (conv4_1): 37,440 ops (36,864 macc; 576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14 (conv4_p): 334,656 ops (331,776 macc; 2,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15 (res4_out): 576 ops (0 macc; 0 comp; 576 add; 0 mul; 0 bitwise)
    Layer 16 (buffer_shift): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (conv5): 18,464 ops (18,432 macc; 32 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18 (tcn0): 40,352 ops (39,936 macc; 416 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19 (tcn1): 27,936 ops (27,648 macc; 288 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (tcn2): 480 ops (480 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 378,272 bytes out of 442,368 bytes total (85.5%)
  Bias memory:   933 bytes out of 2,048 bytes total (45.6%)
*/
