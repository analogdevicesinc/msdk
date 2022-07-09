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
#include "utils.h"
#include "camera.h"
#include "utils.h"
#include "embedding_process.h"
#include "MAXCAM_Debug.h"
#include "faceID.h"
#include "sema_regs.h"
#include "led.h"
#include "tmr.h"
#include "cnn.h"
#include "mxc_delay.h"

#define S_MODULE_NAME   "state_faceid"

typedef struct {
    char*    data;
    int len;
} text_t;

/* **** Globals **** */
volatile uint32_t cnn_time; // Stopwatch
/********************************** Type Defines  *****************************/
typedef void (*ScreenFunc)(void);
/************************************ VARIABLES ******************************/
static void process_img(void);
static void run_cnn(int x_offset, int y_offset);

static int8_t prev_decision = -2;
static int8_t decision = -2;

extern volatile uint32_t *arm_mail_box;
extern volatile uint32_t *riscv_mail_box;

int start_faceid(void)
{
    uint32_t run_count = 0;

    camera_start_capture_image();

#define PRINT_TIME 1
#if (PRINT_TIME==1)
    /* Get current time */
    uint32_t process_time = utils_get_time_ms();
    uint32_t total_time = utils_get_time_ms();
#endif

    while (1) { //Capture image and run CNN

        /* Check if camera image is ready to process */
        if (camera_is_image_rcv()) {

#if (PRINT_TIME==1)
            process_time = utils_get_time_ms();
#endif
            process_img();

#ifdef  IMAGE_TO_UART
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
            }
            else {
                run_cnn(-10, 10);
                run_cnn(10, -10);
            }

            run_count++;

            riscv_mail_box[0] = RESULT_READY;
            // Signal the Cortex-M4 to wake up and display on TFT
            MXC_SEMA->irq0 = MXC_F_SEMA_IRQ0_EN | MXC_F_SEMA_IRQ0_CM4_IRQ;

#if (PRINT_TIME==1)
            //printf("\n\n\n");
            PR_INFO("Process Time Total : %dms", utils_get_time_ms() - process_time);
#endif

            //PR_DEBUG("RISC-V: sleep");
            asm volatile("wfi"); // Wait for wakeup timer interrupt
            //PR_DEBUG("RISC-V: wake up");

            // Camera startup delay (~100ms) after resuming XVCLK clock generated
            // by Pulse Train which is off during UPM/Standby mode
            if (LP_MODE > 2) {
                MXC_Delay(100000);
            }

            camera_start_capture_image();

#if (PRINT_TIME==1)
            PR_INFO("Capture Time : %dms", process_time - total_time);
            PR_INFO("Total Time : %dms", utils_get_time_ms() - total_time);
            total_time = utils_get_time_ms();
#endif

            //PR_DEBUG("RISC-V: sleep");
            asm volatile("wfi"); // Wait for camera or semaphore interrupt
            //PR_DEBUG("RISC-V: wake up");

        }

        if (arm_mail_box[0]) {
            PR_DEBUG("Stop FaceId");
            /* clear ARM mailbox */
            arm_mail_box[0] = 0;
            break;
        }

    }

    return 0;
}

static void process_img(void)
{
    uint32_t pass_time = 0;
    uint32_t  imgLen;
    uint32_t  w, h;
    int ret, lum;
    uint16_t*  image;
    uint8_t*   raw;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

#ifdef  IMAGE_TO_UART
    // Send the image through the UART to the console.
    // "grab_image" python program will read from the console and write to an image file.
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
#endif

    pass_time = utils_get_time_ms();

    image = (uint16_t*)raw; // 2bytes per pixel RGB565

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
    image = ((uint16_t*)raw) + (((IMAGE_H - (WIDTH + 2 * THICKNESS)) / 2) + WIDTH + THICKNESS) * IMAGE_W;

    for (int i = 0; i < THICKNESS; i++) {
        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);

        for (int j = 0; j < HEIGHT + 2 * THICKNESS; j++) {
            *(image++) = FRAME_COLOR; //color
        }

        image += ((IMAGE_W - (HEIGHT + 2 * THICKNESS)) / 2);
    }

    //top + bottom lines
    image = ((uint16_t*)raw) + ((IMAGE_H - (WIDTH + 2 * THICKNESS)) / 2) * IMAGE_W;

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

    // Post image info to RISC-V mailbox
    riscv_mail_box[1] = (uint32_t)raw;
    //PR_DEBUG("Image ptr: %x",riscv_mail_box[1]);
    riscv_mail_box[2] = h;
    //PR_DEBUG("Image heigth: %x",riscv_mail_box[2]);
    riscv_mail_box[3] = w;
    //PR_DEBUG("Image width: %x\n",riscv_mail_box[3]);
    riscv_mail_box[0] = IMAGE_READY;
    // Signal the Cortex-M4 to wake up and display on TFT
    MXC_SEMA->irq0 = MXC_F_SEMA_IRQ0_EN | MXC_F_SEMA_IRQ0_CM4_IRQ;

    // Read luminance level from camera
    ret = camera_get_luminance_level(&lum);

    if (ret != STATUS_OK) {
        PR_ERR("Camera Error %d", ret);
    }
    else {
        PR_DEBUG("Lum = %d", lum);

        // Warn if luminance level is low
        if (lum < LOW_LIGHT_THRESHOLD) {
            PR_WARN("Low Light!");
        }
    }

    PR_INFO("Screen print time : %d", utils_get_time_ms() - pass_time);
}

static void run_cnn(int x_offset, int y_offset)
{
    uint32_t  imgLen;
    uint32_t  w, h;
    static uint32_t noface_count = 0;
    /* Get current time */
    uint32_t pass_time = 0;
    uint8_t*   raw, *ptr;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    pass_time = utils_get_time_ms();

    // Enable CNN clock
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN);

    cnn_start();

    PR_INFO("CNN initialization time : %d", utils_get_time_ms() - pass_time);

    uint8_t* data = raw;

    pass_time = utils_get_time_ms();

    LED_On(1); // red LED

    for (int i = y_offset; i < HEIGHT + y_offset; i++) {
        data =  raw + ((IMAGE_H - (WIDTH)) / 2) * IMAGE_W * BYTE_PER_PIXEL;
        data += (((IMAGE_W - (HEIGHT)) / 2) + i) * BYTE_PER_PIXEL;

        for (int j = x_offset; j < WIDTH + x_offset; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;

            ub = (uint8_t)(data[j * BYTE_PER_PIXEL * IMAGE_W + 1] << 3);
            ug = (uint8_t)((data[j * BYTE_PER_PIXEL * IMAGE_W] << 5) | ((data[j * BYTE_PER_PIXEL * IMAGE_W + 1] & 0xE0) >> 3));
            ur = (uint8_t)(data[j * BYTE_PER_PIXEL * IMAGE_W] & 0xF8);

            b = ub - 128;
            g = ug - 128;
            r = ur - 128;

#ifndef FAST_FIFO
            // Loading data into the CNN fifo
            while (((*((volatile uint32_t*) 0x50000004) & 1)) != 0);  // Wait for FIFO 0

            number = 0x00FFFFFF & ((((uint8_t)b) << 16) | (((uint8_t)g) << 8) | ((uint8_t)r));
            *((volatile uint32_t*) 0x50000008) = number;  // Write FIFO 0
#else

            // Loading data into the CNN fifo
            while (((*((volatile uint32_t*) 0x400c0404) & 2)) != 0);  // Wait for FIFO 0

            number = 0x00FFFFFF & ((((uint8_t)b) << 16) | (((uint8_t)g) << 8) | ((uint8_t)r));
            *((volatile uint32_t*) 0x400c0410) = number;  // Write FIFO 0
#endif
        }
    }

    LED_Off(1);

    int  cnn_load_time = utils_get_time_ms() - pass_time;

    PR_DEBUG("CNN load data time : %d", cnn_load_time);

    text_t cnn_load_time_string;
    char string_time[7];

    sprintf(string_time, "%dms", cnn_load_time);
    cnn_load_time_string.data = string_time;
    cnn_load_time_string.len = strlen(string_time);

    pass_time = utils_get_time_ms();

    // CNN interrupt wakes up CPU from sleep mode    
    while (cnn_time == 0) {
        asm volatile("wfi"); // Sleep and wait for CNN interrupt
    }

    PR_INFO("CNN wait time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    cnn_unload((uint32_t*)(raw));

    cnn_stop();

    // Disable CNN clock to save power
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);

    PR_INFO("CNN unload time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    int pResult = calculate_minDistance((uint8_t*)(raw));

    PR_INFO("Embedding time : %d", utils_get_time_ms() - pass_time);
    PR_INFO("Result = %d \n", pResult);

    if (pResult == 0) {
        text_t printResult;
        char* name;

        uint8_t* counter;
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
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                break;
            }
            else if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.4)) { // >%40 adjust
                name = "Adjust Face";
                decision = -2;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                break;
            }
            else if (counter[id] > closest_sub_buffer_size * 0.2) {  //>>20% unknown
                name = "Unknown";
                decision = -1;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
                break;
            }
            else if (counter[id] > closest_sub_buffer_size * 0.1) { //>> 10% transition
                name = "";
                decision = -3;
                noface_count = 0;
                PR_DEBUG("Status: %s \n", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
            }
            else {
                noface_count ++;

                if (noface_count > 10) {
                    name = "No face";
                    decision = -4;
                    noface_count --;
                    PR_INFO("Detection: %s: %d", name, counter[id]);
                }
            }
        }

        PR_DEBUG("Decision: %d Name:%s \n", decision, name);
        printResult.data = name;
        printResult.len = strlen(name);

        PR_DEBUG("%s \n", name);

        // Post CNN result to RISC-V mailbox
        riscv_mail_box[1] = (uint32_t)cnn_load_time_string.len;
        //PR_DEBUG("Time size: %d",cnn_load_time_string.len);

        ptr = (uint8_t*)&riscv_mail_box[2];
        memcpy(ptr, cnn_load_time_string.data, cnn_load_time_string.len);
        //PR_DEBUG("Time ptr: %x", ptr );

        riscv_mail_box[6] = (uint32_t)printResult.len;
        //PR_DEBUG("Result size: %d",printResult.len);

        ptr = (uint8_t*)&riscv_mail_box[7];
        memcpy(ptr, printResult.data, printResult.len);
        //PR_DEBUG("Result ptr: %x", ptr );
    }
}
