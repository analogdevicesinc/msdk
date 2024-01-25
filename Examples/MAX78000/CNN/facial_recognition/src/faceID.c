/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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
#include <string.h>

#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc.h"
#include "utils.h"
#include "camera.h"
#include "faceID.h"
#include "facedetection.h"
#include "embeddings.h"
#include "MAXCAM_Debug.h"
#include "cnn_1.h"
#include "cnn_2.h"
#include "cnn_3.h"
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
extern volatile char names[1024][7];
extern volatile uint32_t cnn_time; // Stopwatch

static void run_cnn_2(void);

#ifdef TFT_ENABLE
#ifdef BOARD_FTHR_REVA
static int font = (int)&Liberation_Sans16x16[0];
#endif
#endif //#ifdef TFT_ENABLE

extern uint8_t box[4]; // x1, y1, x2, y2
volatile int32_t output_buffer[16];
char *name;
static int32_t ml_3_data32[(CNN_3_NUM_OUTPUTS + 3) / 4];

int face_id(void)
{
#if (PRINT_TIME == 1)
    uint32_t process_time;
#endif

    /* Check for received image */
    if (camera_is_image_rcv()) {
#if (PRINT_TIME == 1)
        process_time =
            utils_get_time_ms(); // Mark the start of process_time.  Var will be re-used to re-calculate itself.
#endif

        // Enable CNN peripheral, enable CNN interrupt, turn on CNN clock
        // CNN clock: 50 MHz div 1
        cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
        /* Configure CNN 2 to recognize a face */
        cnn_2_init(); // Bring state machine into consistent state
        uint32_t weight_load_time = utils_get_time_ms();
        cnn_2_load_weights_from_SD(); // Load CNN kernels from SD card
        PR_DEBUG("CNN weight load time: %dms", utils_get_time_ms() - weight_load_time);
        cnn_2_load_bias(); // Reload CNN bias
        PR_DEBUG("CNN bias load");
        cnn_2_configure(); // Configure state machine
        PR_DEBUG("CNN configure");

        /* Run CNN */
        run_cnn_2();
        PR_DEBUG("CNN RUN");

#if (PRINT_TIME == 1)
        process_time = utils_get_time_ms() - process_time;
        PR_INFO("FaceID Processing Time : %dms", process_time);
#endif
    }

    return 0;
}

static void run_cnn_2(void)
{
    uint32_t imgLen;
    uint32_t w, h;
    /* Get current time */
    uint32_t pass_time = 0;
    uint8_t *raw;
    float y_prime;
    float x_prime;
    uint8_t x_loc;
    uint8_t y_loc;
    int adj_boxes[4];
    int diff = 0;

    float sum = 0;
    float b;

    uint8_t box_width = box[2] - box[0];
    uint8_t box_height = box[3] - box[1];

    if (box_width > box_height) {
        diff = box_width - box_height;
        PR_DEBUG("width is bigger diff: %d", diff);
        PR_DEBUG("x1: %d, y1: %d, x2: %d, y2: %d", box[0], box[1], box[2], box[3]);
        if (diff % 2 == 0) {
            adj_boxes[1] = (int)box[1] - diff / 2;
            adj_boxes[3] = (int)box[3] + diff / 2;
        } else {
            adj_boxes[1] = (int)box[1] - diff / 2;
            adj_boxes[3] = (int)box[3] + diff / 2 + 1;
        }
        adj_boxes[0] = (int)box[0];
        adj_boxes[2] = (int)box[2];
        PR_DEBUG("ADJUSTED x1: %d, y1: %d, x2: %d, y2: %d", adj_boxes[0], adj_boxes[1],
                 adj_boxes[2], adj_boxes[3]);
    } else {
        diff = box_height - box_width;
        PR_DEBUG("height is bigger diff: %d", diff);
        PR_DEBUG("x1: %d, y1: %d, x2: %d, y2: %d", box[0], box[1], box[2], box[3]);
        if (diff % 2 == 0) {
            adj_boxes[0] = (int)box[0] - diff / 2;
            adj_boxes[2] = (int)box[2] + diff / 2;
        } else {
            adj_boxes[0] = (int)box[0] - diff / 2;
            adj_boxes[2] = (int)box[2] + diff / 2 + 1;
        }
        adj_boxes[1] = (int)box[1];
        adj_boxes[3] = (int)box[3];
        PR_DEBUG("ADJUSTED x1: %d, y1: %d, x2: %d, y2: %d", adj_boxes[0], adj_boxes[1],
                 adj_boxes[2], adj_boxes[3]);
    }
    int x1 = adj_boxes[1]; //rotated box
    int y1 = adj_boxes[0]; //rotated box
    box_height = adj_boxes[3] - adj_boxes[1];
    box_width = adj_boxes[2] - adj_boxes[0];

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    pass_time = utils_get_time_ms();
    cnn_start();

    uint8_t *data = raw;

    PR_DEBUG("x1: %d, y1: %d", x1, y1);

    // Resize image inside facedetection box to 160x120 and load to CNN memory
    for (int i = 0; i < HEIGHT_ID; i++) {
        y_prime = ((float)(i) / HEIGHT_ID) * box_height;
        y_loc = (uint8_t)(MIN(round(y_prime), box_height - 1));
        data = raw + y1 * IMAGE_W * BYTE_PER_PIXEL + y_loc * BYTE_PER_PIXEL;
        data += x1 * BYTE_PER_PIXEL;
        for (int j = 0; j < WIDTH_ID; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;
            x_prime = ((float)(j) / WIDTH_ID) * box_width;
            x_loc = (uint8_t)(MIN(round(x_prime), box_width - 1));
            if ((y1 + x_loc < 0) || (x1 + y_loc < 0) || (y1 + x_loc >= WIDTH_DET) ||
                (x1 + y_loc >= HEIGHT_DET)) {
                b = 0;
                g = 0;
                r = 0;
            } else {
                ub = (uint8_t)(data[x_loc * BYTE_PER_PIXEL * HEIGHT_DET + 1] << 3);
                ug = (uint8_t)((data[x_loc * BYTE_PER_PIXEL * HEIGHT_DET] << 5) |
                               ((data[x_loc * BYTE_PER_PIXEL * HEIGHT_DET + 1] & 0xE0) >> 3));
                ur = (uint8_t)(data[x_loc * BYTE_PER_PIXEL * HEIGHT_DET] & 0xF8);

                b = ub - 128;
                g = ug - 128;
                r = ur - 128;
            }

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

    PR_DEBUG("CNN wait time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    cnn_2_unload((uint32_t *)(output_buffer));

    cnn_disable();

    uint32_t value;
    int8_t pr_value;
    float n1, n2, n3, n4;
    //Calculate vector sum
    for (int i = 0; i < 16; i++) {
        value = output_buffer[i];
        pr_value = (int8_t)(value & 0xff);
        sum += pr_value * pr_value;
        pr_value = (int8_t)((value >> 8) & 0xff);
        sum += pr_value * pr_value;
        pr_value = (int8_t)((value >> 16) & 0xff);
        sum += pr_value * pr_value;
        pr_value = (int8_t)((value >> 24) & 0xff);
        sum += pr_value * pr_value;
    }
    b = 1 / sqrt(sum);

    for (int i = 0; i < 16; i++) {
        value = output_buffer[i];
        pr_value = (int8_t)(value & 0xff);
        n1 = 128 * pr_value * b;
        if (n1 < 0) {
            n1 = 256 + n1;
        }

        pr_value = (int8_t)((value >> 8) & 0xff);
        n2 = 128 * pr_value * b;
        if (n2 < 0) {
            n2 = 256 + n2;
        }

        pr_value = (int8_t)((value >> 16) & 0xff);
        n3 = 128 * pr_value * b;
        if (n3 < 0) {
            n3 = 256 + n3;
        }
        pr_value = (int8_t)((value >> 24) & 0xff);
        n4 = 128 * pr_value * b;
        if (n4 < 0) {
            n4 = 256 + n4;
        }
#ifdef UNNORMALIZE_RECORD
        norm_output_buffer[i] = (((uint8_t)n4) << 24) | (((uint8_t)n3) << 16) |
                                (((uint8_t)n2) << 8) | ((uint8_t)n1);
#else
        output_buffer[i] = (((uint8_t)n4) << 24) | (((uint8_t)n3) << 16) | (((uint8_t)n2) << 8) |
                           ((uint8_t)n1);
#endif
    }
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_3_init(); // Bring CNN state machine into consistent state
    cnn_3_load_weights(); // Load CNN kernels
    cnn_3_load_bias(); // Load CNN bias
    cnn_3_configure();
//load input
#ifdef UNNORMALIZE_RECORD
    uint32_t *out_point = (uint32_t *)norm_output_buffer; // For unnormalized emb. experiment
#else
    uint32_t *out_point = (uint32_t *)output_buffer;
#endif

    memcpy32((uint32_t *)0x50400000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50408000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50410000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50418000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50800000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50808000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50810000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50818000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50c00000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50c08000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50c10000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x50c18000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x51000000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x51008000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x51010000, out_point, 1);
    out_point++;
    memcpy32((uint32_t *)0x51018000, out_point, 1);
    out_point++;
    cnn_3_start(); // Start CNN_3
    while (cnn_time == 0)
        ;
    //    MXC_LP_EnterSleepMode(); // Wait for CNN_3, but not sleep, ISR can't catch it otherwise
    cnn_3_unload((uint32_t *)ml_3_data32);
    cnn_disable();

    PR_DEBUG("CNN unload time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    int32_t *ml_point = ml_3_data32;
    int8_t max_emb = 0;
    int32_t max_emb_index = 0;
    char *name;

    for (int i = 0; i < DEFAULT_EMBS_NUM; i++) {
        value = *ml_point;
        pr_value = value & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = i * 4;
        }
        pr_value = (value >> 8) & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = (i * 4) + 1;
        }
        pr_value = (value >> 16) & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = (i * 4) + 2;
        }
        pr_value = (value >> 24) & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = (i * 4) + 3;
        }
        ml_point++;
    }
    PR_DEBUG("FaceID inference time: %d ms\n", utils_get_time_ms() - pass_time);
    PR_DEBUG("CNN_3 max value: %d \n", max_emb);
    PR_DEBUG("CNN_3 max value index: %d \n", max_emb_index);
    if (max_emb > Threshold) {
        PR_INFO("FaceID result: subject id: %d \n", max_emb_index);
        name = (char *)names[max_emb_index];
        PR_INFO("FaceID result: subject name: %s \n", name);
    } else {
        PR_INFO("FaceID result: Unknown subject");
        name = "Unknown";
    }

#ifdef TFT_ENABLE
    text_t printResult;
    printResult.data = name;
    printResult.len = strlen(name);
    //Rotate before print text
    MXC_TFT_SetRotation(ROTATE_180);
    MXC_TFT_ClearArea(&area, 4);
    MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
    MXC_TFT_SetRotation(ROTATE_270);

#endif
}
