/**
 * @file    main.c
 * @brief   Display Demo
 * @details Bouncing text and blinking virtual LEDs demo.
 *          
 */

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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "lv_conf.h"
#include "lvgl/lvgl.h"
#include "ssd1306.h"

/***** Definitions *****/
#define DISPLAY_HOR_RES (128)
#define DISPLAY_VER_RES (32)

// LVGL Definitions
#define DRAW_BUF_SIZE (DISPLAY_HOR_RES * DISPLAY_VER_RES / 8)
#define TEXT_BOUNCE_DELAY 10

// LVGL DISPLAY DRIVER VARIABLES
static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;
static lv_disp_t *disp;
static lv_color_t disp_buf1[DRAW_BUF_SIZE];

// LVGL SCREEN VARIABLES
static lv_obj_t *label1;
static lv_obj_t *img1;
static lv_obj_t *ui_Screen1_Spinner1;
static lv_obj_t *ui_Screen1_Spinner2;

extern ssd1306_dev cfal12832c_controller;

//============================================================================
static void set_px_cb(struct _lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x,
                      lv_coord_t y, lv_color_t color, lv_opa_t opa)
{
    ssd1306_set_buffer_pixel_util(buf, buf_w, DRAW_BUF_SIZE, x, y, color.full,
                                  (LV_COLOR_SCREEN_TRANSP != opa));
}

static void flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    ssd1306_flush_area(&cfal12832c_controller, (display_area_t *)area, (uint8_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

static void rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->y1 = (area->y1 & (~0x7));
    area->y2 = (area->y2 & (~0x7)) + 7;
}
//============================================================================
void lvgl_setup()
{
    /* LittlevGL setup */
    lv_init();

    /* Initialize `disp_buf` with the buffer(s). */
    lv_disp_draw_buf_init(&disp_buf, disp_buf1, NULL, DRAW_BUF_SIZE * 8);

    /* Display driver setup */
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf = &disp_buf;
    disp_drv.flush_cb = flush_cb;
    disp_drv.set_px_cb = set_px_cb;
    disp_drv.rounder_cb = rounder_cb;
    disp_drv.hor_res = DISPLAY_HOR_RES;
    disp_drv.ver_res = DISPLAY_VER_RES;
    disp = lv_disp_drv_register(&disp_drv);
}

//============================================================================
void test_screen_1(void)
{
    ui_Screen1_Spinner1 = lv_spinner_create(lv_scr_act(), 1000, 90);
    lv_obj_set_size(ui_Screen1_Spinner1, 20, 20);
    lv_obj_align(ui_Screen1_Spinner1, LV_ALIGN_CENTER, 50, 0);
    lv_obj_set_style_arc_opa(ui_Screen1_Spinner1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Screen1_Spinner1, 4, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Screen1_Spinner2 = lv_spinner_create(lv_scr_act(), 2000, 90);
    lv_obj_set_size(ui_Screen1_Spinner2, 20, 20);
    lv_obj_align(ui_Screen1_Spinner2, LV_ALIGN_CENTER, -50, 0);
    lv_obj_set_style_arc_opa(ui_Screen1_Spinner2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Screen1_Spinner2, 4, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    LV_IMG_DECLARE(adi_logo);
    img1 = lv_img_create(lv_scr_act());
    lv_img_set_src(img1, &adi_logo);
    lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);

    label1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label1, "Analog\nDevices");
    lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);
}

//============================================================================
int main(void)
{
    uint32_t last_tick, last_roll_tick, last_bounce_tick;
    int x, y, incx;

    printf("OLED demo\n");
    printf("This example uses LVGL graphics library to manage display\n");
    printf("For more demos please check: https://github.com/lvgl/lvgl\n");
    printf("Note: This example only works on MAX32672 FTHR board\n");

    ssd1306_init(&cfal12832c_controller);

    lvgl_setup();

    test_screen_1();

    /* Bounce routine */
    last_tick = last_roll_tick = last_bounce_tick = lv_tick_get();
    x = y = 0;
    incx = 1;

    while (1) {
        MXC_Delay(MXC_DELAY_MSEC(1));
        lv_tick_inc(1);

        /* Create the bouncing text effect */
        if ((last_bounce_tick + TEXT_BOUNCE_DELAY) < lv_tick_get()) {
            last_bounce_tick = lv_tick_get();
            if (x >= 80) {
                incx = -1;
            } else if (x <= -80) {
                incx = 1;
            }
            x += incx;

            lv_obj_set_pos(img1, x, y);
        }

        if ((last_tick + 10) < lv_tick_get()) {
            /* The timing is not critical but should be between 1..10 ms */
            lv_task_handler();
            last_tick = lv_tick_get();
        }
    }

    return E_NO_ERROR;
}
