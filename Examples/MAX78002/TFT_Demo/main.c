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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "mxc.h"

#define TOD_START_TIME     (12 * SECS_PER_HR + 34 * SECS_PER_MIN + 56)
#define TOD_ALARM_INTERVAL 2
#define SECS_PER_MIN 60
#define SECS_PER_HR  (60 * SECS_PER_MIN)
#define SECS_PER_DAY (24 * SECS_PER_HR)
#define TS_X_MIN 254
#define TS_X_MAX 3680
#define TS_Y_MIN 193
#define TS_Y_MAX 3661
#define TFT_BUFF_SIZE		32		// TFT buffer size

volatile bool tod_alarm = false;
int image_bitmap = (int)&img_1_rgb565[0];
int font_1 = (int)&Arial12x12[0];
int font_2 = (int)&Arial24x23[0];
int font_3 = (int)&Arial28x28[0];
int font_4 = (int)&SansSerif16x16[0];
int font_5 = (int)&SansSerif19x19[0];
const int font_5_width = 19;
const int font_5_height = 19;

void TFT_Print(char *str, int x, int y, int font, int length)
{
text_t text = { .data = str, .len = length};
	
    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

void TFT_test(void)
{
area_t _area;
area_t* area;
char buff[TFT_BUFF_SIZE];

	MXC_TFT_SetRotation(ROTATE_270);
    MXC_TFT_ShowImage(0, 0, image_bitmap);

    /* Get a good look at bitmap and allow debugger time to attach */
    MXC_Delay(MXC_DELAY_SEC(3));
	MXC_TFT_SetBackGroundColor(RED);

	area = &_area;
    area->x = 10;
    area->y = 10;
    area->w = 200;
    area->h = 100;

	MXC_TFT_FillRect(area, GREEN);
	MXC_Delay(MXC_DELAY_SEC(2));
	MXC_TFT_ClearScreen();
	MXC_TFT_Line(10, 10, 200, 200, NAVY);
	MXC_Delay(MXC_DELAY_SEC(1));
	MXC_TFT_Rectangle(10, 10, 200, 200, NAVY);
	MXC_Delay(MXC_DELAY_SEC(1));
	MXC_TFT_Circle(100, 100, 50, PURPLE);
	MXC_Delay(MXC_DELAY_SEC(1));
	MXC_TFT_FillCircle(100, 100, 50, PURPLE);
	MXC_Delay(MXC_DELAY_SEC(1));
	MXC_TFT_SetBackGroundColor(BLACK);
    MXC_TFT_SetForeGroundColor(WHITE);   // set chars to white
	MXC_TFT_ClearScreen();

	memset(buff,32, TFT_BUFF_SIZE);
	sprintf(buff, "ANALOG DEVICES");
	TFT_Print(buff, 0, 10, font_1, 14);

	sprintf(buff, "Analog Devices");
	TFT_Print(buff, 0, 50, font_2, 14);

	sprintf(buff, "Analog Devices");
	TFT_Print(buff, 0, 100, font_3, 14);

	sprintf(buff, "Analog Devices");
	TFT_Print(buff, 0, 150, font_4, 14);

	sprintf(buff, "Analog Devices");
	TFT_Print(buff, 0, 200, font_5, 14);
    
    MXC_Delay(MXC_DELAY_SEC(3));
    MXC_TFT_SetBackGroundColor(BLACK);
    MXC_TFT_SetForeGroundColor(WHITE);
    MXC_TFT_ClearScreen();
}

void RTC_IRQHandler(void)
{
  int flags = MXC_RTC_GetFlags();

  MXC_RTC_ClearFlags(flags);

  /* Check time-of-day alarm flag. */
  if (flags & MXC_F_RTC_CTRL_TOD_ALARM) {
    while (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY);

    MXC_RTC_SetTimeofdayAlarm(MXC_RTC_GetSecond() + TOD_ALARM_INTERVAL);
        
    while (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY);

    tod_alarm = true;
  }
}

void print_time(void)
{
  int day, hr, min, sec;
  char buf[9];
  int x, y;
  static int last_x = 0;
  static int last_y = 0;
  
  sec = MXC_RTC_GetSecond();
  day = sec / SECS_PER_DAY;
  sec -= day * SECS_PER_DAY;
  hr = sec / SECS_PER_HR;
  sec -= hr * SECS_PER_HR;
  min = sec / SECS_PER_MIN;
  sec -= min * SECS_PER_MIN;
  
  x = rand() % (DISPLAY_WIDTH - (font_5_width * 8));
  y = rand() % (DISPLAY_HEIGHT - font_5_height);

  TFT_Print("        ", last_x, last_y, font_5, 8);
  TFT_Print(buf, x, y, font_5, snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hr, min,sec));

  last_x = x;
  last_y = y;
}

int32_t rescale(int32_t x, int32_t min, int32_t max, int32_t a, int32_t b)
{
  x = (x > max) ? max : (x < min) ? min : x;

  return a + (((x - min) * (b - a)) / (max - min));
}

int main(void)
{
  uint16_t x, y;
  int32_t xx, yy;

  MXC_ICC_Enable(MXC_ICC0);
  MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
  SystemCoreClockUpdate();

  srand(78002);

  printf("TFT Demo Example\n");
  /* Initialize TFT display */
  MXC_TFT_Init(MXC_SPI0, -1, NULL, NULL);
  TFT_test();

  /* Initialize touch screen */
  if (MXC_TS_Init(MXC_SPI0, -1, NULL, NULL))
    printf("Touch screen initialization failed\n");

  /* Initialize RTC */
  MXC_RTC_Init(TOD_START_TIME, 0);
  MXC_RTC_SetTimeofdayAlarm(TOD_START_TIME + TOD_ALARM_INTERVAL);
  MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE);
  NVIC_EnableIRQ(RTC_IRQn);
  MXC_LP_EnableRTCAlarmWakeup();
  MXC_RTC_Start();

  for (;;) {
    if (tod_alarm) {
      tod_alarm = false;
      print_time();
    }

    if (ts_event) {
      MXC_TS_GetTouch(&x, &y);
      ts_event = false;
      printf("%d,%d  ", x, y);
      xx = rescale(x, TS_X_MIN, TS_X_MAX, 0, DISPLAY_HEIGHT);
      yy = rescale(y, TS_Y_MIN, TS_Y_MAX, 0, DISPLAY_WIDTH);
      printf("%d,%d\n", xx, yy);
    }
  }
}
