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
#include "lp.h"
#include "uart.h"
#include "math.h"
#include "post_process.h"
#define S_MODULE_NAME "faceid"

#define PRINT_TIME 1

extern uint32_t ticks_1;
extern uint32_t ticks_2;
extern mxc_wut_cfg_t cfg;
extern area_t area;
/************************************ VARIABLES ******************************/
extern volatile uint32_t cnn_time; // Stopwatch

static void process_img(int x_offset, int y_offset);
static void run_cnn_2(int x_offset, int y_offset);
#ifdef LP_MODE_ENABLE
static void ARM_low_power(int lp_mode);
#endif

#ifdef TFT_ENABLE
#ifdef BOARD_FTHR_REVA
static int font   = (int)&SansSerif16x16[0];
#endif
#endif //#ifdef TFT_ENABLE

static int box_x_offset = 0;
static int box_y_offset = 0;

static int8_t prev_decision = -2;
static int8_t decision      = -2;
uint8_t box[4]; // x1, y1, x2, y2

/********************************* Functions **************************/
void calculate_box_offset(void)
{
int x_mid, y_mid, temp;
uint8_t x1, y1, x2, y2;
uint8_t box_width;
uint8_t box_height;
	
	x1 = box[0];
    y1 = box[1];
    x2 = box[2];
	y2 = box[3];
	
	box_width = x2 - x1;
	box_height = y2 - y1;
	
	PR_DEBUG("width:%d height:%d\n", box_width, box_height);
	
	// Box coordinates sanity check
	if((box_width == 0) || (box_height == 0))
	{
		box_x_offset = 0;
		box_y_offset = 0;
		return;
	}
	
	// Calculate box middle point
	x_mid = x1 + box_width/2;
	y_mid = y1 + box_height/2;
	
	PR_DEBUG("x_mid:%d y_mid:%d\n", x_mid, y_mid);
	
	// Calculate difference between X coordinate of box middle point and captured image middle point
	temp = x_mid - IMAGE_H/2;
	// Limit box movement
	if(temp > MAX_X_OFFSET)
		box_x_offset = MAX_X_OFFSET;
	else if(temp < -MAX_X_OFFSET)
		box_x_offset = -MAX_X_OFFSET;
	else
		box_x_offset = temp;

    // Calculate difference between Y coordinate of box middle point and captured image middle point
    temp = y_mid - IMAGE_W/2;
	
	// Limit box movement
	if(temp > MAX_Y_OFFSET)
		box_y_offset = MAX_Y_OFFSET;
	else if(temp < -MAX_Y_OFFSET)
		box_y_offset = -MAX_Y_OFFSET;
	else
		box_y_offset = temp;
	
	PR_DEBUG("x_offset:%d y_offset:%d\n", box_x_offset, box_y_offset);
}

void get_box(void)
{
   	PR_DEBUG("x1:%d y1:%d x2:%d y2:%d\n", box[0], box[1], box[2], box[3]);
	calculate_box_offset();
}


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
			get_box();
            process_img(box_x_offset, box_y_offset);

            // Enable CNN peripheral, enable CNN interrupt, turn on CNN clock
			// CNN clock: 50 MHz div 1
			cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
            weight_load_time = utils_get_time_ms();
			/* Configure CNN 2 to recognize a face */
			cnn_2_init();         // Bring state machine into consistent state
            cnn_2_load_weights_from_SD(); // Load CNN kernels from SD card
			PR_DEBUG("CNN weight load time: %dms", utils_get_time_ms() - weight_load_time);
            cnn_2_load_bias();    // Reload CNN bias
            cnn_2_configure();    // Configure state machine
			
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

static void process_img(int x_offset, int y_offset)
{
#ifdef TFT_ENABLE
//int x1, x2, y1, y2;
uint32_t pass_time = 0;
#endif
uint32_t imgLen;
uint32_t w, h;
uint8_t* raw;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

#ifdef IMAGE_TO_UART
    // Send the image through the UART to the console.
    // "grab_image" python program will read from the console and write to an image file.
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
#endif

#ifdef TFT_ENABLE
    pass_time = utils_get_time_ms();

#ifdef BOARD_FTHR_REVA
    MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, raw, w, h);
	//x1 = X_START + MAX_X_OFFSET + x_offset;
	//y1 = Y_START + MAX_Y_OFFSET + y_offset;
	//x2 = x1 + WIDTH_ID;
	//y2 = y1 + HEIGHT_ID;
	// Draw rectangle around face
	//MXC_TFT_Rectangle(x1, y1, x2, y2, FRAME_ORANGE);
#endif

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
            printResult.len  = strlen(printResult.data);
            area_t area      = {50, 290, 180, 30};
            MXC_TFT_ClearArea(&area, 4);
            MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &printResult, NULL);
        }
    }

    PR_INFO("Screen print time : %d", utils_get_time_ms() - pass_time);
#endif //#ifdef TFT_ENABLE
}

static void run_cnn_2(int x_offset, int y_offset)
{
    uint32_t imgLen;
    uint32_t w, h;
    static uint32_t noface_count = 0;
    /* Get current time */
    uint32_t pass_time = 0;
    uint8_t* raw;
    float y_prime;
    float x_prime;
    uint8_t x_loc;
    uint8_t y_loc;
    uint8_t x1 =  MAX(box[1], 0); 
    uint8_t y1 =  MAX(box[0], 0);
    uint8_t box_width = box[2] - box[0];
    uint8_t box_height = box[3] - box[1];

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    pass_time = utils_get_time_ms();
    cnn_start();

    //PR_INFO("CNN initialization time : %d", utils_get_time_ms() - pass_time);

    uint8_t* data = raw;

    pass_time = utils_get_time_ms();

    //LED_On(1); // red LED
    #ifdef USE_BOX_ONLY
    PR_INFO("x1: %d, y1: %d", x1, y1);
    for (int i = 0; i <  HEIGHT_ID ; i++) {
        y_prime = ((float)(i) / HEIGHT_ID) * box_height;
        y_loc =  (uint8_t) (MIN(round(y_prime), box_height-1)); // + y1;        
        data = raw + y1 * IMAGE_W * BYTE_PER_PIXEL + y_loc  * BYTE_PER_PIXEL;
        data += x1 * BYTE_PER_PIXEL;
        for (int j = 0; j < WIDTH_ID ; j++) {
            uint8_t ur, ug, ub;
            int8_t r, g, b;
            uint32_t number;
            x_prime = ((float)(j) / WIDTH_ID) * box_width;
            x_loc = (uint8_t) (MIN(round(x_prime), box_width-1));
            
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
			while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0

            *((volatile uint32_t*)0x50000008) = number; // Write FIFO 0
        }
    }
    #else
	for (int i = y_offset; i <  HEIGHT_ID + y_offset; i++) {
        data = raw + ((IMAGE_H - (WIDTH_ID)) / 2) * IMAGE_W * BYTE_PER_PIXEL;
        data += (((IMAGE_W - (HEIGHT_ID)) / 2) + i) * BYTE_PER_PIXEL;

        for (int j = x_offset; j < WIDTH_ID + x_offset; j++) {
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

            number = 0x00FFFFFF & ((((uint8_t)b) << 16) | (((uint8_t)g) << 8) | ((uint8_t)r));
            // Loading data into the CNN fifo
			// Remove the following line if there is no risk that the source would overrun the FIFO:
			while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0

            *((volatile uint32_t*)0x50000008) = number; // Write FIFO 0
        }
    }
    #endif

    //LED_Off(1);

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

    cnn_2_unload((uint32_t*)(raw));

	// Power off CNN after unloading result to clear all CNN registers.
	// It's needed to load and run other CNN model
	cnn_disable();

    PR_INFO("CNN unload time : %d", utils_get_time_ms() - pass_time);

    pass_time = utils_get_time_ms();

    int pResult = calculate_minDistance((uint8_t*)(raw));
    PR_DEBUG("Embedding time : %d", utils_get_time_ms() - pass_time);
    PR_DEBUG("Result = %d \n", pResult);

    if (pResult == 0) {
        char* name;

        uint8_t* counter;
        uint8_t counter_len;
        get_min_dist_counter(&counter, &counter_len);

        name          = "";
        prev_decision = decision;
        decision      = -5;

        PR_INFO("counter_len: %d,  %d,%d,%d\n", counter_len, counter[0], counter[1], counter[2]);

        for (uint8_t id = 0; id < counter_len; ++id) {
            if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.8)) { // >80%  detection
                name         = get_subject(id);
                decision     = id;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
				LED_Off(RED_LED);
				LED_On(GREEN_LED);
				LED_Off(BLUE_LED);	
                break;
            } else if (counter[id] >= (uint8_t)(closest_sub_buffer_size * 0.4)) { // >%40 adjust
                name         = "Adjust Face";
                decision     = -2;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
				LED_Off(RED_LED);
				LED_Off(GREEN_LED);
				LED_On(BLUE_LED);
                break;
            } else if (counter[id] > closest_sub_buffer_size * 0.2) { //>>20% unknown
                name         = "Unknown";
                decision     = -1;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
				LED_On(RED_LED);
				LED_Off(GREEN_LED);
				LED_On(BLUE_LED);
                break;
            } else if (counter[id] > closest_sub_buffer_size * 0.1) { //>> 10% transition
                name         = "";
                decision     = -3;
                noface_count = 0;
                PR_DEBUG("Status: %s", name);
                PR_INFO("Detection: %s: %d", name, counter[id]);
				LED_On(RED_LED);
				LED_Off(GREEN_LED);
				LED_Off(BLUE_LED);

            } else {
                noface_count++;

                if (noface_count > 10) {
                    name     = "No face";
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
            printResult.len  = strlen(name);
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
