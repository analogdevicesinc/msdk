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
#include "facedetection.h"
#include "post_process.h"
#include "utils.h"
#include "MAXCAM_Debug.h"
#include "cnn_1.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#include "led.h"
#include "lp.h"

#define S_MODULE_NAME "facedetection"
/************************************ VARIABLES ******************************/
volatile uint32_t cnn_time; // Stopwatch

static void run_cnn_1(int x_offset, int y_offset);

int face_detection(void)
{
    // Capture the image
    camera_start_capture_image();
    /* Sleep until camera interrupt */
    MXC_LP_EnterSleepMode();

#define PRINT_TIME 1
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

#ifdef IMAGE_TO_UART
        break;
#endif

        // Enable CNN peripheral, enable CNN interrupt, turn on CNN clock
        // CNN clock: 50 MHz div 1
        cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
        /* Configure CNN 1 to detect a face */
        cnn_1_init(); // Bring CNN state machine into consistent state
        cnn_1_load_weights(); // Load CNN kernels
        cnn_1_load_bias(); // Load CNN bias
        cnn_1_configure(); // Configure CNN state machine

        /* Run CNN on time on original and shifted images */
        run_cnn_1(0, 0);

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

static void run_cnn_1(int x_offset, int y_offset)
{
    uint32_t imgLen;
    uint32_t w, h;
    /* Get current time */
    uint32_t pass_time = 0;
    uint8_t *raw;
    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);
#ifdef TFT_ENABLE
#ifdef BOARD_FTHR_REVA
    MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, raw, w, h);
#endif
#endif

    cnn_start();

    //PR_INFO("CNN initialization time : %d", utils_get_time_ms() - pass_time);

    uint8_t *data = raw;

    pass_time = utils_get_time_ms();

    for (int i = y_offset; i < HEIGHT_DET + y_offset; i++) {
        data = raw + ((IMAGE_H - (WIDTH_DET)) / 2) * IMAGE_W * BYTE_PER_PIXEL;
        data += (((IMAGE_W - (HEIGHT_DET)) / 2) + i) * BYTE_PER_PIXEL;
        for (int j = x_offset; j < WIDTH_DET + x_offset; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;

            ub = (uint8_t)(data[j * BYTE_PER_PIXEL * IMAGE_W + 1] << 3);
            ug = (uint8_t)((data[j * BYTE_PER_PIXEL * IMAGE_W] << 5) |
                           ((data[j * BYTE_PER_PIXEL * IMAGE_W + 1] & 0xE0) >> 3));
            ur = (uint8_t)(data[j * BYTE_PER_PIXEL * IMAGE_W] & 0xF8);
            b = ub - 128;
            g = ug - 128;
            r = ur - 128;

            // Loading data into the CNN fifo
            while (((*((volatile uint32_t *)0x50000004) & 1)) != 0)
                ; // Wait for FIFO 0

            number = 0x00FFFFFF & ((((uint8_t)b) << 16) | (((uint8_t)g) << 8) | ((uint8_t)r));

            *((volatile uint32_t *)0x50000008) = number; // Write FIFO 0
        }
    }

    int cnn_load_time = utils_get_time_ms() - pass_time;
    PR_DEBUG("CNN load data time : %dms", cnn_load_time);

    pass_time = utils_get_time_ms();

    // Disable Deep Sleep mode
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    // CNN interrupt wakes up CPU from sleep mode
    while (cnn_time == 0) {
        asm volatile("wfi"); // Sleep and wait for CNN interrupt
    }

    PR_DEBUG("CNN wait time : %dms", utils_get_time_ms() - pass_time);

    get_priors();
    localize_objects();
    // Power off CNN after unloading result to clear all CNN registers.
    // It's needed to load and run other CNN model
    cnn_disable();
}
