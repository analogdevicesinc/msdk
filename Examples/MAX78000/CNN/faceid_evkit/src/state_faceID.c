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
#include <string.h>

#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc.h"
#include "keypad.h"
#include "state.h"
#include "utils.h"
#include "camera.h"
#include "faceID.h"
#include "utils.h"
#include "embedding_process.h"
#include "MAXCAM_Debug.h"
#include "cnn.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#include "tsc2046.h"
#include "bitmap.h"
#endif
#include "led.h"
#include "lp.h"

#define S_MODULE_NAME "state_faceid"

extern uint32_t ticks_1;
extern uint32_t ticks_2;
extern mxc_wut_cfg_t cfg;
/********************************** Type Defines  *****************************/
typedef void (*ScreenFunc)(void);

/************************************ VARIABLES ******************************/
volatile uint32_t cnn_time; // Stopwatch

static void process_img(void);
static void run_cnn(int x_offset, int y_offset);
static int init(void);
static int key_process(int key);
#ifdef LP_MODE_ENABLE
static void ARM_low_power(int lp_mode);
#endif

#ifdef TFT_ENABLE
static text_t screen_msg[] = {
    // info
    { (char *)"FACEID DEMO", strlen("FACEID DEMO") },
    { (char *)"Process Time:", strlen("Process Time:") },
};
#ifdef BOARD_EVKIT_V1
static int bitmap = logo_white_bg_darkgrey_bmp;
static int font = urw_gothic_12_grey_bg_white;
#endif
#ifdef BOARD_FTHR_REVA
static int bitmap = (int)&logo_rgb565[0];
static int font = (int)&SansSerif16x16[0];
#endif
#endif //#ifdef TFT_ENABLE

static int8_t prev_decision = -2;
static int8_t decision = -2;

static State g_state = { "faceID", init, key_process, NULL, 0 };

/********************************* Static Functions **************************/
#ifdef TFT_ENABLE
static void screen_faceID(void)
{
    MXC_TFT_SetPalette(bitmap);
    MXC_TFT_SetBackGroundColor(4);
    // MXC_TFT_ShowImage(3, 5, bitmap);
#ifdef BOARD_EVKIT_V1
    MXC_TFT_ShowImage(BACK_X, BACK_Y, left_arrow_bmp); // back button icon
#endif
    MXC_TFT_PrintFont(98, 5, font, &screen_msg[0], NULL); // FACEID DEMO
    MXC_TFT_PrintFont(12, 240, font, &screen_msg[1], NULL); // Process Time:
    // texts
#ifdef TS_ENABLE
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(BACK_X, BACK_Y, BACK_X + 48, BACK_Y + 39, KEY_1); // Back
#endif
}
#endif //#ifdef TFT_ENABLE

static int init(void)
{
    uint32_t run_count = 0;
#ifdef TFT_ENABLE
    screen_faceID();
#endif

    camera_start_capture_image();

#define PRINT_TIME 1
#if (PRINT_TIME == 1)
    /* Get current time */
    uint32_t process_time = utils_get_time_ms();
    uint32_t total_time = utils_get_time_ms();
#endif

    while (1) { //Capture image and run CNN
#ifdef TS_ENABLE
        /* Check pressed touch screen key */
        int key = MXC_TS_GetKey();

        if (key > 0) {
            key_process(key);
        }

        if (state_get_current() != &g_state) {
            break;
        }

#endif

        /* Check for received image */
        if (camera_is_image_rcv()) {
#if (PRINT_TIME == 1)
            process_time = utils_get_time_ms();
#endif
#ifdef TS_ENABLE
            /* Disable TouchScreen to avoid collision with TFT */
            MXC_TS_Stop();
#endif
            process_img();

#ifdef IMAGE_TO_UART
            break;
#endif

            MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

#ifdef LP_MODE_ENABLE
            /* Reinit CNN and reload weigths after UPM or Standby because CNN is powered off */
            if (LP_MODE > 2) {
                cnn_init(); // Bring state machine into consistent state
                cnn_load_weights(); // Reload CNN kernels
                cnn_load_bias(); // Reload CNN bias
                cnn_configure(); // Configure state machine
            }
#endif
            /* Run CNN three times on original and shifted images */
            run_cnn(0, 0);

            if ((run_count % 2) == 0) {
                run_cnn(-10, -10);
                run_cnn(10, 10);
            } else {
                run_cnn(-10, 10);
                run_cnn(10, -10);
            }

#ifdef TS_ENABLE
            /* Enable TouchScreen */
            MXC_TS_Start();
#endif
            run_count++;

#if (PRINT_TIME == 1)

            printf("\n\n\n");
            PR_INFO("Process Time Total : %dms", utils_get_time_ms() - process_time);
#endif

#ifdef LP_MODE_ENABLE
            cfg.cmp_cnt = ticks_1;
            /* Config WakeUp Timer */
            MXC_WUT_Config(&cfg);
            //Enable WUT
            MXC_WUT_Enable();

            LED_On(0); // green LED on
            /* Configure low power mode */
            ARM_low_power(LP_MODE);
            LED_Off(0); // green LED off

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

            camera_start_capture_image();
            /* Sleep until camera interrupt */
            MXC_LP_EnterSleepMode();
        }
    }

    return 0;
}

static int key_process(int key)
{
    switch (key) {
    case KEY_1:
        state_set_current(get_home_state());
        break;

    default:
        break;
    }

    return 0;
}

static void process_img(void)
{
    uint32_t pass_time = 0;
    uint32_t imgLen;
    uint32_t w, h;
    uint16_t *image;
    uint8_t *raw;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

#ifdef IMAGE_TO_UART
    // Send the image through the UART to the console.
    // "grab_image" python program will read from the console and write to an image file.
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
#endif

    pass_time = utils_get_time_ms();

    image = (uint16_t *)raw; // 2bytes per pixel RGB565

    // left line
    image += ((IMAGE_H - (WIDTH + 2 * THICKNESS)) / 2) * IMAGE_W;

    for (int i = 0; i < THICKNESS; i++) {
        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);

        for (int j = 0; j < HEIGHT + 2 * THICKNESS; j++) {
            *(image++) = FRAME_COLOR; //color
        }

        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);
    }

    //right line
    image = ((uint16_t *)raw) +
            (((IMAGE_H - (WIDTH + 2 * THICKNESS)) / 2) + WIDTH + THICKNESS) * IMAGE_W;

    for (int i = 0; i < THICKNESS; i++) {
        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);

        for (int j = 0; j < HEIGHT + 2 * THICKNESS; j++) {
            *(image++) = FRAME_COLOR; //color
        }

        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);
    }

    //top + bottom lines
    image = ((uint16_t *)raw) + ((IMAGE_H - (WIDTH + 2 * THICKNESS)) / 2) * IMAGE_W;

    for (int i = 0; i < WIDTH + 2 * THICKNESS; i++) {
        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);

        for (int j = 0; j < THICKNESS; j++) {
            *(image++) = FRAME_COLOR; //color
        }

        image += HEIGHT;

        for (int j = 0; j < THICKNESS; j++) {
            *(image++) = FRAME_COLOR; //color
        }

        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);
    }

    PR_INFO("Frame drawing time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

#ifdef TFT_ENABLE
#ifdef BOARD_EVKIT_V1
    MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, raw, h, w);
#endif
#ifdef BOARD_FTHR_REVA
    MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, raw, w, h);
#endif

    int ret;
    int lum;
    text_t printResult;
    // Read luminance level from camera
    ret = camera_get_luminance_level(&lum);

    if (ret != STATUS_OK) {
        PR_ERR("Camera Error %d", ret);
    } else {
        PR_DEBUG("Lum = %d", lum);

        // Warn if luminance level is low
        if (lum < LOW_LIGHT_THRESHOLD) {
            PR_WARN("Low Light!");
            printResult.data = " LOW LIGHT ";
            printResult.len = strlen(printResult.data);
            area_t area = { 50, 290, 180, 30 };
            MXC_TFT_ClearArea(&area, 4);
            MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
        }
    }

#endif //#ifdef TFT_ENABLE

    PR_INFO("Screen print time : %d", utils_get_time_ms() - pass_time);
}

static void run_cnn(int x_offset, int y_offset)
{
    uint32_t imgLen;
    uint32_t w, h;
    static uint32_t noface_count = 0;
    /* Get current time */
    uint32_t pass_time = 0;
    uint8_t *raw;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    pass_time = utils_get_time_ms();

    // Enable CNN clock
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN);

    cnn_start();

    PR_INFO("CNN initialization time : %d", utils_get_time_ms() - pass_time);

    uint8_t *data = raw;

    pass_time = utils_get_time_ms();

    //LED_On(1); // red LED
    for (int i = y_offset; i < HEIGHT + y_offset; i++) {
        data = raw + ((IMAGE_H - (WIDTH)) / 2) * IMAGE_W * BYTE_PER_PIXEL;
        data += (((IMAGE_W - (HEIGHT)) / 2) + i) * BYTE_PER_PIXEL;

        for (int j = x_offset; j < WIDTH + x_offset; j++) {
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
            while (((*((volatile uint32_t *)0x50000004) & 1)) != 0) {}
            // Wait for FIFO 0

            number = 0x00FFFFFF & ((((uint8_t)b) << 16) | (((uint8_t)g) << 8) | ((uint8_t)r));

            *((volatile uint32_t *)0x50000008) = number; // Write FIFO 0
        }
    }

    //LED_Off(1);

    int cnn_load_time = utils_get_time_ms() - pass_time;

    PR_DEBUG("CNN load data time : %d", cnn_load_time);

#ifdef TFT_ENABLE
    text_t printResult;
    text_t cnn_load_time_string;
    char string_time[7];

    snprintf(string_time, sizeof(string_time), "%dms", cnn_load_time);
    cnn_load_time_string.data = string_time;
    cnn_load_time_string.len = strlen(string_time);
    area_t area = { 150, 240, 50, 30 };
    MXC_TFT_ClearArea(&area, 4);
    MXC_TFT_PrintFont(150, 240, font, &cnn_load_time_string, NULL); // RunCNN
#endif

    pass_time = utils_get_time_ms();

    // Disable Deep Sleep mode
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    // CNN interrupt wakes up CPU from sleep mode
    while (cnn_time == 0) {
        asm volatile("wfi"); // Sleep and wait for CNN interrupt
    }

    PR_INFO("CNN wait time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    cnn_unload((uint32_t *)(raw));

    cnn_stop();

    // Disable CNN clock to save power
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);

    PR_INFO("CNN unload time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    int pResult = calculate_minDistance((uint8_t *)(raw));

    PR_INFO("Embedding time : %d", utils_get_time_ms() - pass_time);
    PR_INFO("Result = %d \n", pResult);

    if (pResult == 0) {
        char *name;

        uint8_t *counter;
        uint8_t counter_len;
        get_min_dist_counter(&counter, &counter_len);

        name = "";
        prev_decision = decision;
        decision = -5;

        PR_INFO("counter_len: %d,  %d,%d,%d\n", counter_len, counter[0], counter[1], counter[2]);
#if 1

        for (uint8_t id = 0; id < counter_len; ++id) {
            if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.8)) { // >80%  detection
                name = get_subject(id);
                decision = id;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                break;
            } else if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.4)) { // >%40 adjust
                name = "Adjust Face";
                decision = -2;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                break;
            } else if (counter[id] > closest_sub_buffer_size * 0.2) { //>>20% unknown
                name = "Unknown";
                decision = -1;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                break;
            } else if (counter[id] > closest_sub_buffer_size * 0.1) { //>> 10% transition
                name = "";
                decision = -3;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
            } else {
                noface_count++;

                if (noface_count > 10) {
                    name = "No face";
                    decision = -4;
                    noface_count--;
                    PR_INFO("Detection: %s: %d", name, counter[id]);
                }
            }
        }

#else

        for (uint8_t id = 0; id < counter_len; ++id) {
            if (counter[id] >= (closest_sub_buffer_size - 4)) {
                name = get_subject(id);
                decision = id;
                break;
            } else if (counter[id] >= (closest_sub_buffer_size / 2 + 1)) {
                name = "Adjust Face";
                decision = -2;
                break;
            } else if (counter[id] > 4) {
                name = "Unknown";
                decision = -1;
                break;
            }
        }

#endif

        PR_DEBUG("Decision: %d Name:%s \n", decision, name);

#ifdef TFT_ENABLE

        if (decision != prev_decision) {
            area_t area = { 50, 290, 180, 30 };
            printResult.data = name;
            printResult.len = strlen(name);
            MXC_TFT_ClearArea(&area, 4);
            MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
        }

#endif
    }
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

/********************************* Public Functions **************************/
State *get_faceID_state(void)
{
    return &g_state;
}
