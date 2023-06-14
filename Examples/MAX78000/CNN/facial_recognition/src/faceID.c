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

#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc.h"
#include "utils.h"
#include "camera.h"
#include "faceID.h"
#include "facedetection.h"
#include "utils.h"
#include "embedding_process.h"
#include "MAXCAM_Debug.h"
#include "cnn_1.h"
#include "cnn_2.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#include "led.h"
#include "uart.h"
#include "math.h"
#include "post_process.h"
#define S_MODULE_NAME "faceid"

#define PRINT_TIME 1

extern area_t area;
/************************************ VARIABLES ******************************/
extern volatile uint32_t cnn_time; // Stopwatch

static void run_cnn_2(void);

#ifdef TFT_ENABLE
#ifdef BOARD_FTHR_REVA
static int font = (int)&SansSerif16x16[0];
#endif
#endif //#ifdef TFT_ENABLE

static int8_t prev_decision = -2;
static int8_t decision = -2;
extern uint8_t box[4]; // x1, y1, x2, y2

int face_id(void)
{
#if (PRINT_TIME == 1)
    /* Get current time */
    uint32_t process_time = utils_get_time_ms();
    uint32_t total_time = utils_get_time_ms();
    uint32_t weight_load_time = 0;
#endif

    /* Check for received image */
    if (camera_is_image_rcv()) {
#if (PRINT_TIME == 1)
        process_time = utils_get_time_ms();
#endif

        // Enable CNN peripheral, enable CNN interrupt, turn on CNN clock
        // CNN clock: 50 MHz div 1
        cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
        weight_load_time = utils_get_time_ms();
        /* Configure CNN 2 to recognize a face */
        cnn_2_init(); // Bring state machine into consistent state
        cnn_2_load_weights_from_SD(); // Load CNN kernels from SD card
        PR_DEBUG("CNN weight load time: %dms", utils_get_time_ms() - weight_load_time);
        cnn_2_load_bias(); // Reload CNN bias
        cnn_2_configure(); // Configure state machine

        /* Run CNN */
        run_cnn_2();

#if (PRINT_TIME == 1)
        PR_INFO("Process Time Total : %dms", utils_get_time_ms() - process_time);
#endif

#if (PRINT_TIME == 1)
        PR_INFO("Capture Time : %dms", process_time - total_time);
        PR_INFO("Total Time : %dms", utils_get_time_ms() - total_time);
        total_time = utils_get_time_ms();
#endif
    }

    return 0;
}

static void run_cnn_2(void)
{
    uint32_t imgLen;
    uint32_t w, h;
    static uint32_t noface_count = 0;
    /* Get current time */
    uint32_t pass_time = 0;
    uint8_t *raw;
    float y_prime;
    float x_prime;
    uint8_t x_loc;
    uint8_t y_loc;
    uint8_t x1 = MAX(box[1], 0);
    uint8_t y1 = MAX(box[0], 0);
    uint8_t box_width = box[2] - box[0];
    uint8_t box_height = box[3] - box[1];

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    pass_time = utils_get_time_ms();
    cnn_start();

    //PR_INFO("CNN initialization time : %d", utils_get_time_ms() - pass_time);

    uint8_t *data = raw;

    pass_time = utils_get_time_ms();

    PR_INFO("x1: %d, y1: %d", x1, y1);

    // Resize image inside facedetection box to 160x120 and load to CNN memory
    for (int i = 0; i < HEIGHT_ID; i++) {
        y_prime = ((float)(i) / HEIGHT_ID) * box_height;
        y_loc = (uint8_t)(MIN(round(y_prime), box_height - 1)); // + y1;
        data = raw + y1 * IMAGE_W * BYTE_PER_PIXEL + y_loc * BYTE_PER_PIXEL;
        data += x1 * BYTE_PER_PIXEL;
        for (int j = 0; j < WIDTH_ID; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;
            x_prime = ((float)(j) / WIDTH_ID) * box_width;
            x_loc = (uint8_t)(MIN(round(x_prime), box_width - 1));

            ub = (uint8_t)(data[x_loc * BYTE_PER_PIXEL * IMAGE_W + 1] << 3);
            ug = (uint8_t)((data[x_loc * BYTE_PER_PIXEL * IMAGE_W] << 5) |
                           ((data[x_loc * BYTE_PER_PIXEL * IMAGE_W + 1] & 0xE0) >> 3));
            ur = (uint8_t)(data[x_loc * BYTE_PER_PIXEL * IMAGE_W] & 0xF8);

            b = ub - 128;
            g = ug - 128;
            r = ur - 128;

            number = 0x00FFFFFF & ((((uint8_t)b) << 16) | (((uint8_t)g) << 8) | ((uint8_t)r));
            // Loading data into the CNN fifo
            // Remove the following line if there is no risk that the source would overrun the FIFO:
            while (((*((volatile uint32_t *)0x50000004) & 1)) != 0)
                ; // Wait for FIFO 0

            *((volatile uint32_t *)0x50000008) = number; // Write FIFO 0
        }
    }

    int cnn_load_time = utils_get_time_ms() - pass_time;
    PR_DEBUG("CNN load data time : %d", cnn_load_time);

    pass_time = utils_get_time_ms();

    // Disable Deep Sleep mode
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    // CNN interrupt wakes up CPU from sleep mode
    while (cnn_time == 0) {
        asm volatile("wfi"); // Sleep and wait for CNN interrupt
    }

    PR_INFO("CNN wait time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    cnn_2_unload((uint32_t *)(raw));

    // Power off CNN after unloading result to clear all CNN registers.
    // It's needed to load and run other CNN model
    cnn_disable();

    PR_INFO("CNN unload time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    int pResult = calculate_minDistance((uint8_t *)(raw));
    PR_DEBUG("Embedding time : %d", utils_get_time_ms() - pass_time);
    PR_DEBUG("Result = %d \n", pResult);

    if (pResult == 0) {
        char *name;

        uint8_t *counter;
        uint8_t counter_len;
        get_min_dist_counter(&counter, &counter_len);

        name = "";
        prev_decision = decision;
        decision = -5;

        PR_INFO("counter_len: %d,  %d,%d,%d\n", counter_len, counter[0], counter[1], counter[2]);

        for (uint8_t id = 0; id < counter_len; ++id) {
            if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.8)) { // >80%  detection
                name = get_subject(id);
                decision = id;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                LED_Off(RED_LED);
                LED_On(GREEN_LED);
                LED_Off(BLUE_LED);
                break;
            } else if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.4)) { // >%40 adjust
                name = "Adjust Face";
                decision = -2;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                LED_Off(RED_LED);
                LED_Off(GREEN_LED);
                LED_On(BLUE_LED);
                break;
            } else if (counter[id] > closest_sub_buffer_size * 0.2) { //>>20% unknown
                name = "Unknown";
                decision = -1;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                LED_On(RED_LED);
                LED_Off(GREEN_LED);
                LED_On(BLUE_LED);
                break;
            } else if (counter[id] > closest_sub_buffer_size * 0.1) { //>> 10% transition
                name = "";
                decision = -3;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                LED_On(RED_LED);
                LED_Off(GREEN_LED);
                LED_Off(BLUE_LED);

            } else {
                noface_count++;

                if (noface_count > 10) {
                    name = "No subject";
                    decision = -4;
                    noface_count--;
                    PR_INFO("Detection: %s: %d", name, counter[id]);
                    LED_On(RED_LED);
                    LED_Off(GREEN_LED);
                    LED_Off(BLUE_LED);
                }
            }
        }

        PR_DEBUG("Decision: %d Name:%s\n", decision, name);

#ifdef TFT_ENABLE
        text_t printResult;
        if (decision != prev_decision) {
            printResult.data = name;
            printResult.len = strlen(name);
            MXC_TFT_ClearArea(&area, 4);
            MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
        }

#endif
    }
}
