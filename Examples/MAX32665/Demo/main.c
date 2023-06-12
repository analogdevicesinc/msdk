/**
 * @file    main.c
 * @brief   Hello_World-lvgl Demo
 * @details Bouncing text and blinking virtual LEDs demo.
 *          
 */

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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "lv_conf.h"
#include "lvgl.h"
#include "sharp_mip.h"
#include "tmr.h"
#include "led.h"
#include "pb.h"
#include "rtc.h"

/***** Definitions *****/
#define DISPLAY_HOR_RES (128)
#define DISPLAY_VER_RES (128)

// LVGL Definitions
#define DRAW_BUF_SIZE BUF_SIZE(DISPLAY_HOR_RES, DISPLAY_VER_RES)
#define TEXT_BOUNCE_DELAY 10

// LVGL DISPLAY DRIVER VARIABLES
static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;
static lv_disp_t *disp;
static lv_color_t disp_buf1[DRAW_BUF_SIZE];

extern sharp_mip_dev ls013b7dh03_controller;

// LVGL SCREEN VARIABLES
lv_obj_t *label1;
lv_obj_t *label2;
lv_obj_t *label3;
lv_obj_t *label4;
lv_obj_t *img1;

// Tick Timer Parameters
#define TICK_TIMER MXC_TMR0 // Can be MXC_TMR0 through MXC_TMR5
#define TICK_TIMER_IRQn TMR0_IRQn
mxc_tmr_cfg_t lvgl_tmr;

// RTC Definitions
#define SECS_PER_MIN 60
#define SECS_PER_HR (60 * SECS_PER_MIN)
#define SECS_PER_DAY (24 * SECS_PER_HR)

//============================================================================
static void set_px_cb(struct _lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x,
                      lv_coord_t y, lv_color_t color, lv_opa_t opa)
{
    sharp_mip_set_buffer_pixel_util(&ls013b7dh03_controller, buf, buf_w, x, y, color.full,
                                    (LV_COLOR_SCREEN_TRANSP != opa));
}

static void flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    sharp_mip_flush_area(&ls013b7dh03_controller, (display_area_t *)area, (uint8_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

static void rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->x1 = 0;
    area->x2 = DISPLAY_HOR_RES - 1;
}

//============================================================================
void lvgl_setup()
{
    /* LittlevGL setup */
    lv_init();

    /* Initialize `disp_buf` with the buffer(s). */
    lv_disp_draw_buf_init(&disp_buf, disp_buf1, NULL, DRAW_BUF_SIZE);

    /* Display driver setup */
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf = &disp_buf;
    disp_drv.flush_cb = flush_cb;
    disp_drv.set_px_cb = set_px_cb;
    disp_drv.rounder_cb = rounder_cb;
    disp_drv.hor_res = DISPLAY_HOR_RES;
    disp_drv.ver_res = DISPLAY_VER_RES;
    disp = lv_disp_drv_register(&disp_drv);

    lv_theme_t *th = lv_theme_mono_init(disp, FALSE, LV_FONT_DEFAULT);
    lv_disp_set_theme(disp, th);
}

void LV_Tick_TimerHandler()
{
    // Clear interrupt
    MXC_TMR_ClearFlags(TICK_TIMER);
    lv_tick_inc(1);
    lv_task_handler();
}

void LV_Tick_Timer_Init()
{
    // 200Hz (Can be between 1ms to 10ms)
    uint32_t periodTicks = PeripheralClock / 200;

    /*
    Steps for configuring a timer for Continuous mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */
    MXC_TMR_Shutdown(TICK_TIMER);

    lvgl_tmr.pres = TMR_PRES_4;
    lvgl_tmr.mode = TMR_MODE_CONTINUOUS;
    lvgl_tmr.cmp_cnt = periodTicks;
    lvgl_tmr.pol = 0;

    MXC_TMR_Init(TICK_TIMER, &lvgl_tmr);

    MXC_NVIC_SetVector(TICK_TIMER_IRQn, LV_Tick_TimerHandler);
    NVIC_EnableIRQ(TICK_TIMER_IRQn);

    MXC_TMR_Start(TICK_TIMER);
}

//============================================================================
int main(void)
{
    int i;
    int pb_state = 0;
    int prev_pb_state = 0;
    uint32_t usn_timeout = 0;
    int usn_toggle = 0;
    uint8_t usn[MXC_SYS_USN_LEN];
    uint8_t checksum[MXC_SYS_USN_CHECKSUM_LEN];
    int error;
    int day, hr, min;
    uint32_t sec, rtc_readout;

    printf("**** MAX32665 EV Kit Demo ****\n");

    error = MXC_SYS_GetUSN(usn, checksum);
    if (error != E_NO_ERROR) {
        // Error reading USN
        return error;
    }

    // Initialize the Display, LVGL, and LV Tick
    sharp_mip_init(&ls013b7dh03_controller);

    lvgl_setup();

    // Print Logo
    LV_IMG_DECLARE(adi_logo);
    img1 = lv_img_create(lv_scr_act());
    lv_img_set_src(img1, &adi_logo);
    lv_obj_align(img1, LV_ALIGN_CENTER, -32, -45);

    label1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label1, "Analog\nDevices");
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 18, -45);

    // Set up label to print count
    label2 = lv_label_create(lv_scr_act());
    lv_label_set_text(label2, "");
    lv_obj_set_style_text_align(label2, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(label2, LV_ALIGN_LEFT_MID, 0, -10);

    // Set up labels to print revision and USN
    label3 = lv_label_create(lv_scr_act());
    lv_label_set_text(label3, "");
    lv_obj_set_style_text_align(label3, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(label3, LV_ALIGN_LEFT_MID, 0, 10);

    label4 = lv_label_create(lv_scr_act());
    lv_label_set_text(label4, "");
    lv_obj_set_style_text_align(label4, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(label4, LV_ALIGN_LEFT_MID, 0, 35);

    // lv_tick needed to refresh display at ~200Hz or ~5ms.
    LV_Tick_Timer_Init();

    error = MXC_RTC_Init(0, 0);
    if (error != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        return error;
    }

    error = MXC_RTC_Start();
    if (error != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        return error;
    }

    while (1) {
        do {
            error = MXC_RTC_GetSeconds(&rtc_readout);
        } while (error != E_NO_ERROR);
        sec = rtc_readout;

        day = sec / SECS_PER_DAY;
        sec -= day * SECS_PER_DAY;

        hr = sec / SECS_PER_HR;
        sec -= hr * SECS_PER_HR;

        min = sec / SECS_PER_MIN;
        sec -= min * SECS_PER_MIN;

        printf("\n(ddd:hh:mm:ss): %03d:%02d:%02d:%02d\n\n", day, hr, min, sec);
        lv_label_set_text_fmt(label2, " (ddd:hh:mm:ss)\n  %03d:%02d:%02d:%02d", day, hr, min, sec);

        // Print USN
        pb_state = (PB_Get(1) << 1) | (PB_Get(0));
        // Checking previous button state checks whether the buttons were de-pressed.
        if ((pb_state == 0b011) && prev_pb_state != 0b011) {
            // This flag lets the user clear the USN/Rev info if the two buttons
            // are pressed again before the 5 second timeout ends.
            usn_toggle ^= 1;

            if (usn_toggle) {
                usn_timeout = sec + 5;

                lv_label_set_text_fmt(label3, "Rev: %x", MXC_GCR->revision);

                lv_label_set_text_fmt(
                    label4,
                    "USN: %02x%02x%02x%02x%02x\n      -%02x%02x%02x%02x\n      -%02x%02x%02x%02x",
                    usn[0], usn[1], usn[2], usn[3], usn[4], usn[5], usn[6], usn[7], usn[8], usn[9],
                    usn[10], usn[11], usn[12]);

                printf("\nRev: %x\n", MXC_GCR->revision);
                printf("USN: ");
                for (i = 0; i < MXC_SYS_USN_LEN; i++) {
                    printf("%02x", usn[i]);
                }
                printf("\n\n");
            } else {
                // Clear the USN/Rev if the two buttons are pressed before the timeout.
                lv_label_set_text(label3, "");
                lv_label_set_text(label4, "");
                usn_timeout = 0;
            }
        }

        // Automatically clear the USN/Rev info if the timeout ends.
        if (sec >= usn_timeout && usn_timeout != 0) {
            lv_label_set_text(label3, "");
            lv_label_set_text(label4, "");
            usn_timeout = 0;
            // Reset the toggle flag to prepare for next time the two buttons are pressed.
            usn_toggle = 0;
        }

        LED_Toggle(0);
        LED_Toggle(1);

        MXC_Delay(250000);

        // Toggle LED0 every 250ms instead of 500ms if PB0 is pressed.
        if (pb_state & 0b001) {
            LED_Toggle(0);
        }

        // Toggle LED1 every 250ms instead of 500ms if PB1 is pressed.
        if (pb_state & 0b010) {
            LED_Toggle(1);
        }

        MXC_Delay(250000);

        LED_Toggle(0);
        LED_Toggle(1);

        MXC_Delay(250000);

        // Toggle LED0 every 250ms instead of 500ms if PB0 is pressed.
        if (pb_state & 0b001) {
            LED_Toggle(0);
        }

        // Toggle LED1 every 250ms instead of 500ms if PB1 is pressed.
        if (pb_state & 0b010) {
            LED_Toggle(1);
        }

        MXC_Delay(250000);

        // Keeps track of previous button state to check if buttons were de-pressed.
        prev_pb_state = pb_state;
    }
}
