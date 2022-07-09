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
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "spi.h"
#include "led.h"
#include "board.h"

#include "st7735s_drv.h"
#include "disp_cfaf128128b1.h"

#include "lvgl.h"

/***** Preprocessors *****/
#define SPI_SPEED       6000000  /* Bit Rate */

/* Time (ms) between moving text location
 * Lower bound set by display update over SPI and 1ms SysTick rate
 */
#define TEXT_BOUNCE_DELAY 10

/***** Definitions *****/
#define MY_HOR_RES 128
#define MY_VER_RES 128
#define DRAW_BUF_SIZE (MY_HOR_RES*MY_VER_RES)/10
#define LINEBUF_SIZE (3*MY_HOR_RES)

/***** Globals *****/
volatile uint32_t demo_ticks;
lv_disp_draw_buf_t disp_buf;
lv_disp_drv_t disp_drv;
lv_disp_t *disp;
lv_color_t disp_buf1[DRAW_BUF_SIZE];
lv_color_t disp_buf2[DRAW_BUF_SIZE];
uint8_t linebuf[LINEBUF_SIZE];

mxc_spi_req_t req;

/***** Functions *****/
void SPI_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(MXC_SPI0);
}

void SysTick_Handler(void)
{
  /* 1ms tick LVGL tick */
  lv_tick_inc(1);

  /* Used during initialization and for demo timing */
  demo_ticks++;
}

/* The standard mxc_delay() is not used, as it will re-program the SysTick timer
 *  which is also used by the lvgl demo. Instead, we provide a 1ms-resolution 
 * pause function which blocks until the specified time has elapsed.
 */
void pause_ms(uint32_t x)
{
  /* Determine remaining time */
  x = demo_ticks + x;
  
  /* Counter wrap check */
  if (x < demo_ticks) {
    /* Wait until overflow */
    while (demo_ticks > x);
  }

  /* Burn CPU cycles */
  while (x > demo_ticks);
  
  return;
}

/* 
 * Interface-specific transmit of command and data to the controller
 *
 * Args: 
 * Returns: 
 */
int spi_tx(uint8_t *cmd, unsigned int cmd_len, uint8_t *data, unsigned int data_len)
{
  uint8_t spibuf[(1+LINEBUF_SIZE)*2], *bptr;
  unsigned int txlen;

  if (((cmd_len > 0) && (cmd == NULL)) ||
      ((data_len > 0) && (data == NULL)) ||
      ((cmd_len + data_len) > (1+LINEBUF_SIZE))) {
    return E_BAD_PARAM;
  }
  
  bptr = spibuf;
  txlen = 0;

  /* The txlen++ is _not_ an error. Since the data is 9 bits, it is held in two bytes */
  while (cmd_len--) {
    *bptr++ = *cmd++;
    *bptr++ = 0; /* CMD */
    txlen++;
  }
  while (data_len--) {
    *bptr++ = *data++;
    *bptr++ = 1; /* DATA */
    txlen++;
  }

  req.spi = MXC_SPI0;
  req.txData = (uint8_t *) spibuf;
  req.rxData = NULL;
  req.txLen = txlen;
  req.rxLen = 0;
  req.ssIdx = 0;
  req.ssDeassert = 1;
  req.txCnt = 0;
  req.rxCnt = 0;
  req.completeCB = NULL;
  
  return MXC_SPI_MasterTransaction(&req);
}

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  int32_t x, x1, x2;
  int32_t y, y1, y2;
  uint8_t *lineptr;
  unsigned int len;

  x1 = area->x1;
  x2 = area->x2;
  y1 = area->y1;
  y2 = area->y2;

  for (y = y1; y <= y2; y++) {
    st7735s_xyloc(y, x1);
    len = 0;
    lineptr = linebuf;
    for (x = x1; x <= x2; x++) {
#if (LV_COLOR_DEPTH != 1)
      *lineptr++ = color_p->ch.blue << 3;
      *lineptr++ = color_p->ch.green << 2;
      *lineptr++ = color_p->ch.red << 3;
#else
      *lineptr++ = color_p->full ? 0x00 : 0xff;
      *lineptr++ = color_p->full ? 0x00 : 0xff;
      *lineptr++ = color_p->full ? 0x00 : 0xff;
#endif
      len += 3;
      color_p++;
    }
    st7735s_write_pixels(linebuf, len);
  }
  
  /* IMPORTANT!!!
   * Inform the graphics library that you are ready with the flushing*/
  lv_disp_flush_ready(disp_drv);
}

unsigned int roll_led(void)
{
  static unsigned int state = 0;
  unsigned int max_state = (1<<num_leds)-1; /* 2^n - 1 */
  unsigned int i;
    
  if (num_leds) {
    if (++state > max_state) {
      state = 0;
    }
    for (i = 0; i < num_leds; i++) {
      if (state & (1<<i)) {
	LED_On(i);
      } else {
	LED_Off(i);
      }
    }
  }

  return state;
}

int main(void)
{
  uint32_t last_tick, last_roll_tick, last_bounce_tick;
  int x, y, incx, incy;
  lv_obj_t *label1, *led0, *led1;
  unsigned int state;
  int retVal;
  st7735s_cfg_t panel;
  
  /* Configure SysTick for 1ms rate */
  SysTick->LOAD = (SystemCoreClock / 1000);
  SysTick->VAL = 0;
  SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
  demo_ticks = 0;
  
  printf("LCD demo\n");

  /* Configure SPI interface */
  retVal = MXC_SPI_Init(MXC_SPI0, 1, 0, 1, 0, SPI_SPEED);
    
  if (retVal != E_NO_ERROR) {
    printf("Error configuring SPI\n");
    return retVal;
  }

  retVal = MXC_SPI_SetDataSize(MXC_SPI0, 9);
    
  if (retVal != E_NO_ERROR) {
    printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
    return retVal;
  }
    
  retVal = MXC_SPI_SetWidth(MXC_SPI0, SPI_WIDTH_STANDARD);
    
  if (retVal != E_NO_ERROR) {
    printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
    return retVal;
  }

  /* Initialize ST7735S controller with panel-specific sequence */
  panel.delayfn = pause_ms;
  panel.sendfn = spi_tx;
  panel.regcfg = cfaf128128b1_regcfg;
  panel.ncfgs = cfaf128128b1_ncfgs;
  st7735s_init(&panel);

  /* LittlevGL setup */
  lv_init();

  /* Initialize `disp_buf` with the buffer(s). */
  lv_disp_draw_buf_init(&disp_buf, disp_buf1, disp_buf2, DRAW_BUF_SIZE);

  /* Display driver setup */
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &disp_buf;
  disp_drv.flush_cb = disp_flush;
  disp_drv.hor_res = MY_HOR_RES;
  disp_drv.ver_res = MY_VER_RES;
  disp = lv_disp_drv_register(&disp_drv);

  /* Generate a text label to bounce around the screen */
  label1 =  lv_label_create(lv_scr_act());
  lv_label_set_text(label1, "Maxim\nIntegrated");
  lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
  
  /* Align the Label to the center of the screen
   * 0, 0 at the end means an x, y offset after alignment*/
  lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);

  /* Create simulated LEDs to mirror the physical LEDs */
  led0  = lv_led_create(lv_scr_act());
  lv_obj_set_size(led0, 10, 10);
  lv_obj_align(led0, LV_ALIGN_BOTTOM_LEFT, 5, -5);
  lv_led_set_color(led0, lv_color_hex(0xff0000));
  lv_led_off(led0);

  led1  = lv_led_create(lv_scr_act());
  lv_obj_set_size(led1, 10, 10);
  lv_obj_align(led1, LV_ALIGN_BOTTOM_LEFT, 20, -5);
  lv_led_set_color(led1, lv_color_hex(0x00ff00));
  lv_led_off(led1);

  /* Process the above calls */
  lv_task_handler();

  /* Bounce routine */
  last_tick = last_roll_tick = last_bounce_tick = lv_tick_get();
  x = y = 0;
  incx = 1;
  incy = 2;
  while (1) {
    /* Create the bouncing text effect */
    if ((last_bounce_tick + TEXT_BOUNCE_DELAY) < lv_tick_get()) {
      last_bounce_tick = lv_tick_get();
      if (x >= 20) {
	incx = -2;
      } else if (x <= -20) {
	incx = 1;
      }
      x += incx;
      if (y >= 25) {
	incy = -1;
      } else if (y <= -50) {
	incy = 3;
      }
      y += incy;

      lv_obj_set_pos(label1, x, y);
    }

    if ((last_roll_tick + 500) < lv_tick_get()) {
      last_roll_tick = lv_tick_get();
      state = roll_led();
      if (state & 1) {
	lv_led_on(led0);
      } else {
	lv_led_off(led0);
      }
      if (state & 2) {
	lv_led_on(led1);
      } else {
	lv_led_off(led1);
      }
    }

    if ((last_tick+10)  < lv_tick_get()) {
      /* The timing is not critical but should be between 1..10 ms */
      lv_task_handler();
      last_tick = lv_tick_get();
    }
    
    /* Sleep in low-power while nothing to do */
    if (demo_ticks > 1000) {
      /* Always wise to allow debugger access early in the demo */
      //__WFI(); /* FIXME -- causes visual artifacts, perhaps SPI is not done sending bits when core sleeps? */
    }
  }
  
  printf("Done.\n");
    
  return E_NO_ERROR;
}
