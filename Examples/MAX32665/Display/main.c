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
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "spi.h"
#include "led.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2c.h"
#include "dma.h"
#include "lv_conf.h"
#include "lvgl/lvgl.h"

#include "sharp_mip.h"
#include "test_screen.h"
#include "resources/ecg_data.h"

/***** Definitions *****/
#define DISPLAY_HOR_RES         (128)
#define DISPLAY_VER_RES         (128)

// LVGL Definitions
#define DRAW_BUF_SIZE 			BUF_SIZE(DISPLAY_HOR_RES, DISPLAY_VER_RES)
#define TEXT_BOUNCE_DELAY 		10

// LVGL DISPLAY DRIVER VARIABLES
static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;
static lv_disp_t *disp;
static lv_color_t disp_buf1[DRAW_BUF_SIZE];

extern sharp_mip_dev ls013b7dh03_controller;

//============================================================================
static void set_px_cb(	struct _lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        				lv_color_t color, lv_opa_t opa)
{
	sharp_mip_set_buffer_pixel_util(&ls013b7dh03_controller, buf, buf_w, x, y, color.full, (LV_COLOR_SCREEN_TRANSP != opa));
}

static void flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
	sharp_mip_flush_area(&ls013b7dh03_controller, (display_area_t*)area, (uint8_t*) color_p);
    lv_disp_flush_ready(disp_drv);
}


static void rounder_cb(struct _lv_disp_drv_t * disp_drv, lv_area_t * area) {
	area->x1 = 0;
	area->x2 = DISPLAY_HOR_RES - 1;
}

//============================================================================
void lvgl_setup() {
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

	lv_theme_t * th = lv_theme_mono_init(disp, FALSE, LV_FONT_DEFAULT);
	lv_disp_set_theme(disp, th);
}
//============================================================================
int main(void)
{
	uint32_t last_tick;

	printf("Display demo\n");

	sharp_mip_init(&ls013b7dh03_controller);

	lvgl_setup();

	test_screen();

	last_tick = lv_tick_get();

	int count = 0;
	int ecg_data_size = sizeof(ecg_sample) / sizeof(ecg_sample[0]);
    while (1) {

        MXC_Delay(MXC_DELAY_MSEC(1));
        lv_tick_inc(1);

        lv_chart_set_next_value(chart, ser, ecg_sample[count++ % ecg_data_size]);

        if ((last_tick + 10) < lv_tick_get()) {
            /* The timing is not critical but should be between 1..10 ms */
            lv_task_handler();
            last_tick = lv_tick_get();
        }
    }

	printf("Done.\n");

	return E_NO_ERROR;
}
