/******************************************************************************
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <math.h>

#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc.h"
#include "utils.h"
#include "camera.h"
#include "faceID.h"
#include "facedetection.h"
#include "utils.h"
#include "MAXCAM_Debug.h"
#include "cnn_1.h"
#include "cnn_2.h"
#include "cnn_3.h"
#include "led.h"
#include "lp.h"
#include "uart.h"
#include "math.h"
#include "post_process.h"
#include "record.h"
#include "embeddings.h"

#define S_MODULE_NAME "faceid"

#define PRINT_TIME 1

extern uint32_t ticks_1;
extern uint32_t ticks_2;
extern mxc_wut_cfg_t cfg;
extern area_t area;
extern volatile uint32_t db_flash_emb_count;
extern volatile char
    names[1024][7]; // 1024 names of 7 bytes each, as we support 1024 people in the database
/************************************ VARIABLES ******************************/
extern volatile uint32_t cnn_time; // Stopwatch

#ifndef USE_BOX_ONLY
static void process_img(int x_offset, int y_offset);
#endif

static void run_cnn_2(int x_offset, int y_offset);
#ifdef LP_MODE_ENABLE
static void ARM_low_power(int lp_mode);
#endif

#ifdef TFT_ENABLE
static int font = (int)&Liberation_Sans16x16[0];
#endif //#ifdef TFT_ENABLE

static int box_x_offset = 0;
static int box_y_offset = 0;

uint8_t box[4]; // x1, y1, x2, y2
volatile int32_t output_buffer[16];

#ifdef UNNORMALIZE_RECORD
volatile int32_t norm_output_buffer[16]; // For unnormilized emb. record
#endif

static int32_t ml_3_data32[(CNN_3_NUM_OUTPUTS + 3) / 4];

uint8_t box_width;
uint8_t box_height;
/********************************* Functions **************************/

#ifndef USE_BOX_ONLY
void calculate_box_offset(void)
{
    int x_mid, y_mid, temp;
    uint8_t x1, y1, x2, y2;

    x1 = box[0];
    y1 = box[1];
    x2 = box[2];
    y2 = box[3];

    box_width = x2 - x1;
    box_height = y2 - y1;

    PR_INFO("width:%d height:%d\n", box_width, box_height);

    // Box coordinates sanity check
    if ((box_width == 0) || (box_height == 0)) {
        box_x_offset = 0;
        box_y_offset = 0;
        return;
    }

    // Calculate box middle point
    x_mid = x1 + box_width / 2;
    y_mid = y1 + box_height / 2;

    PR_INFO("x_mid:%d y_mid:%d\n", x_mid, y_mid);

    // Calculate difference between X coordinate of box middle point and captured image middle point
    temp = x_mid - WIDTH_DET / 2;
    // Limit box movement
    if (temp > MAX_X_OFFSET)
        box_x_offset = MAX_X_OFFSET;
    else if (temp < -MAX_X_OFFSET)
        box_x_offset = -MAX_X_OFFSET;
    else
        box_x_offset = temp;

    // Calculate difference between Y coordinate of box middle point and captured image middle point
    temp = y_mid - HEIGHT_DET / 2;

    // Limit box movement
    if (temp > MAX_Y_OFFSET)
        box_y_offset = MAX_Y_OFFSET;
    else if (temp < -MAX_Y_OFFSET)
        box_y_offset = -MAX_Y_OFFSET;
    else
        box_y_offset = temp;

    PR_INFO("x_offset:%d y_offset:%d\n", box_x_offset, box_y_offset);
}

void get_box(void)
{
    PR_INFO("x1:%d y1:%d x2:%d y2:%d\n", box[0], box[1], box[2], box[3]);
    calculate_box_offset();
}
#endif

int face_id(void)
{
#if (PRINT_TIME == 1)
    /* Get current time */
    uint32_t process_time = utils_get_time_ms();
    uint32_t total_time = utils_get_time_ms();
#endif

    /* Check for received image */
    if (camera_is_image_rcv()) {
#if (PRINT_TIME == 1)
        process_time = utils_get_time_ms();
#endif

#ifndef USE_BOX_ONLY
        get_box(); // when USE_BOX_ONLY not used

        process_img(box_x_offset, box_y_offset); // when USE_BOX_ONLY not used
#endif

        /* Run CNN */
        run_cnn_2(box_x_offset, box_y_offset);

#if (PRINT_TIME == 1)
        PR_INFO("Process Time Total : %dms", utils_get_time_ms() - process_time);
#endif

#ifdef LP_MODE_ENABLE
        cfg.cmp_cnt = ticks_1;
        /* Config WakeUp Timer */
        MXC_WUT_Config(&cfg);
        //Enable WUT
        MXC_WUT_Enable();

        //LED_On(0); // green LED on
        /* Configure low power mode */
        ARM_low_power(LP_MODE);
        //LED_Off(0); // green LED off

        // Camera startup delay (~100ms) after resuming XVCLK clock generated
        // by Pulse Train which is off during UPM/Standby mode
        if (LP_MODE > 2) {
            cfg.cmp_cnt = ticks_2;
            /* Config WakeUp Timer */
            MXC_WUT_Config(&cfg);
            //Enable WUT
            MXC_WUT_Enable();
            MXC_LP_EnterLowPowerMode();
        }

#endif

#if (PRINT_TIME == 1)
        PR_INFO("Capture Time : %dms", process_time - total_time);
        PR_INFO("Total Time : %dms", utils_get_time_ms() - total_time);
        total_time = utils_get_time_ms();
#endif
    }

    return 0;
}

#ifndef USE_BOX_ONLY
static void process_img(int x_offset, int y_offset)
{
#ifdef TFT_ENABLE
    int x1, x2, y1, y2;
    uint32_t pass_time = 0;
#endif
    uint32_t imgLen;
    uint32_t w, h;
    uint8_t *raw;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

#ifdef TFT_ENABLE
    pass_time = utils_get_time_ms();

    int ret;
    int lum;
    text_t printResult;
    // Read luminance level from camera
    ret = camera_get_luminance_level(&lum);

    if (ret != STATUS_OK) {
        PR_ERR("Camera Error %d", ret);
    } else {
        //PR_DEBUG("Lum = %d", lum);

        // Warn if luminance level is low
        if (lum < LOW_LIGHT_THRESHOLD) {
            //PR_WARN("Low Light!");
            printResult.data = " LOW LIGHT ";
            printResult.len = strlen(printResult.data);
            //area_t area      = {100, 290, 200, 30};
            MXC_TFT_ClearArea(&area, 4);
            MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
        }
    }

    PR_INFO("Screen print time : %d", utils_get_time_ms() - pass_time);
#endif //#ifdef TFT_ENABLE
}
#endif // #ifndef USE_BOX_ONLY

static void run_cnn_2(int x_offset, int y_offset)
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

    cnn_2_load_bias(); // Load bias data of CNN_2

    // Bring CNN_2 state machine of FaceID model into consistent state
    *((volatile uint32_t *)0x51000000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x51000008) = 0x00000048; // Layer count
    *((volatile uint32_t *)0x52000000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x52000008) = 0x00000048; // Layer count
    *((volatile uint32_t *)0x53000000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x53000008) = 0x00000048; // Layer count
    *((volatile uint32_t *)0x54000000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x54000008) = 0x00000048; // Layer count
    // Enable FIFO control
    *((volatile uint32_t *)0x50000000) = 0x00001108; // FIFO control

    // CNN clock: PLL (200 MHz) div 1
    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;

    // Start CNN_2
    cnn_2_start();

    uint8_t *data = raw;

// Load data to CNN_2 FIFO
#ifdef USE_BOX_ONLY
    PR_INFO("x1: %d, y1: %d", x1, y1);
    for (int i = 0; i < HEIGHT_ID; i++) {
        y_prime = ((float)(i) / HEIGHT_ID) * box_height;
        y_loc = (int)(MIN(round(y_prime), box_height - 1));
        data = raw + y1 * HEIGHT_DET * BYTE_PER_PIXEL + y_loc * BYTE_PER_PIXEL;
        data += x1 * BYTE_PER_PIXEL;
        for (int j = 0; j < WIDTH_ID; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;
            x_prime = ((float)(j) / WIDTH_ID) * box_width;
            x_loc = (int)(MIN(round(x_prime), box_width - 1));
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
#else
    for (int i = y_offset; i < HEIGHT_ID + y_offset; i++) {
        data = raw + ((WIDTH_DET - (WIDTH_ID)) / 2) * HEIGHT_DET * BYTE_PER_PIXEL;
        data += (((HEIGHT_DET - (HEIGHT_ID)) / 2) + i) * BYTE_PER_PIXEL;

        for (int j = x_offset; j < WIDTH_ID + x_offset; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;

            ub = (uint8_t)(data[j * BYTE_PER_PIXEL * HEIGHT_DET + 1] << 3);
            ug = (uint8_t)((data[j * BYTE_PER_PIXEL * HEIGHT_DET] << 5) |
                           ((data[j * BYTE_PER_PIXEL * HEIGHT_DET + 1] & 0xE0) >> 3));
            ur = (uint8_t)(data[j * BYTE_PER_PIXEL * HEIGHT_DET] & 0xF8);

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
#endif

    while (cnn_time == 0) MXC_LP_EnterSleepMode(); // Wait for CNN

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;

    cnn_2_unload((uint32_t *)output_buffer);

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

    //Dot product cnn state machine configuration
    *((volatile uint32_t *)0x51000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x51000008) = 0x00004949; // Layer count
    *((volatile uint32_t *)0x52000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x52000008) = 0x00004949; // Layer count
    *((volatile uint32_t *)0x53000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x53000008) = 0x00004949; // Layer count
    *((volatile uint32_t *)0x54000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x54000008) = 0x00004949; // Layer count
    // Disable FIFO control
    *((volatile uint32_t *)0x50000000) = 0x00000000;

#ifdef UNNORMALIZE_RECORD
    uint32_t *out_point = (uint32_t *)norm_output_buffer; // For unnormalized emb. experiment
#else
    uint32_t *out_point = (uint32_t *)output_buffer;
#endif

    memcpy32((uint32_t *)0x51800000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x51820000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x51840000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x51860000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x52800000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x52820000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x52840000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x52860000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x53800000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x53820000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x53840000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x53860000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x54800000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x54820000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x54840000, out_point, 1);
    out_point++;

    memcpy32((uint32_t *)0x54860000, out_point, 1);

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;

    cnn_3_start(); // Start CNN_3

    while (cnn_time == 0)
        ;
    //    MXC_LP_EnterSleepMode(); // Wait for CNN_3, but not sleep, ISR can't catch it otherwise

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;

    cnn_3_unload((uint32_t *)ml_3_data32);

    int32_t *ml_point = ml_3_data32;
    int8_t max_emb = 0;
    int32_t max_emb_index = 0;
    char *name;

    for (int i = 0; i < (DEFAULT_EMBS_NUM + db_flash_emb_count + 3) / 4; i++) {
        value = *ml_point;
        pr_value = value & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = i * 4;
        }
        PR_DEBUG("pr_value: %d, index: %d\n, name: %s", pr_value, i * 4, names[i * 4]);
        pr_value = (value >> 8) & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = (i * 4) + 1;
        }
        PR_DEBUG("pr_value: %d, index: %d\n, name: %s", pr_value, i * 4, names[(i * 4) + 1]);
        pr_value = (value >> 16) & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = (i * 4) + 2;
            PR_DEBUG("pr_value: %d\n", pr_value);
        }
        PR_DEBUG("pr_value: %d, index: %d\n, name: %s", pr_value, i * 4, names[(i * 4) + 2]);
        pr_value = (value >> 24) & 0xff;
        if ((int8_t)pr_value > max_emb) {
            max_emb = (int8_t)pr_value;
            max_emb_index = (i * 4) + 3;
            PR_DEBUG("pr_value: %d\n", pr_value);
        }
        PR_DEBUG("pr_value: %d, index: %d\n, name: %s", pr_value, i * 4, names[(i * 4) + 3]);
        ml_point++;
    }
    PR_DEBUG("FaceID inference time: %d ms\n", utils_get_time_ms() - pass_time);
    PR_DEBUG("CNN_3 max value: %d \n", max_emb);
    PR_DEBUG("CNN_3 max value index: %d \n", max_emb_index);
    if (max_emb > Threshold) {
        name = (char *)names[max_emb_index];
        PR_DEBUG("subject name: %s \n", name);
    } else {
        PR_DEBUG("Unknown subject \n");
        name = "Unknown";
    }

#ifdef TFT_ENABLE
    text_t printResult;
    printResult.data = name;
    printResult.len = strlen(name);
    MXC_TFT_ClearArea(&area, 4);
    MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
#endif
}

#ifdef LP_MODE_ENABLE
static void ARM_low_power(int lp_mode)
{
    switch (lp_mode) {
    case 0:
        PR_DEBUG("Active\n");
        break;

    case 1:
        PR_DEBUG("Enter SLEEP\n");
        MXC_LP_EnterSleepMode();
        PR_DEBUG("Exit SLEEP\n");
        break;

    case 2:
        PR_DEBUG("Enter LPM\n");
        MXC_LP_EnterLowPowerMode();
        PR_DEBUG("Exit LPM\n");
        break;

    case 3:
        PR_DEBUG("Enter UPM\n");
        MXC_LP_EnterMicroPowerMode();
        PR_DEBUG("Exit UPM\n");
        break;

    case 4:
        PR_DEBUG("Enter STANDBY\n");
        MXC_LP_EnterStandbyMode();
        PR_DEBUG("Exit STANDBY\n");
        break;

    case 5:
        PR_DEBUG("Enter BACKUP\n");
        MXC_LP_EnterBackupMode();
        PR_DEBUG("Exit BACKUP\n");
        break;

    case 6:
        PR_DEBUG("Enter POWERDOWN, disable WUT\n");
        MXC_WUT_Disable();
        MXC_Delay(SEC(2));
        MXC_LP_EnterPowerDownMode();
        PR_DEBUG("Exit SHUTDOWN\n");
        break;

    default:
        PR_DEBUG("Enter SLEEP\n");
        MXC_LP_EnterSleepMode();
        PR_DEBUG("Exit SLEEP\n");
        break;
    }
}
#endif
