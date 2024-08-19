/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portable.h"
#include "semphr.h"
#include "task.h"

#include "clcd.h"
#include "clcd_regs.h"
#include "nvic_table.h"

#include "LCD_Task.h"
#include "UsageTrace.h"

/* Update rate for the frame buffer. This drives the FreeRTOS delay time */
#define FRAME_RATE_MS (1000 / 3)

/* Panel information */
#define PANEL_W 320
#define PANEL_H 240

/* Helper constant for drawing colors bars */
#define COLOR_BAR_COUNT 8

/* FreeRTOS task handle */
static TaskHandle_t lcdTask;

/* Slightly different operation if double buffering is enabled. Double buffering
 * should eliminate any flickering related to drawing while the display is
 * updating, at the cost of an additional frame buffer
 *
 * The variable activeFrame is used for both single and double buffering by the
 * task to make code portable
 */
#ifdef LCD_DOUBLE_BUFFER
/* Support for double buffering, Has 2 frame buffers, controlled via a FreeRTOS
 * semaphore to ensure frames aren't redrawn mid-update
 */
static uint32_t frameBuffer[2][PANEL_W * PANEL_H] __attribute__((aligned(32)));
static uint32_t *activeFrame; //Frame active for drawing
static uint32_t *runningFrame; //Frame running in the controller
static SemaphoreHandle_t newFrameSem; //Sem that a new frame is ready to show
static SemaphoreHandle_t emptyFrameSem; //Sem that a buffer is avail to draw
#else
/* Frame buffer
 * Note: This assumes 24-bits per pixel, word packed to 32. For other bit-depts
 * or palletized colors, this buffer will need to change type and/or size.
 * However, the 32-bit alignment is required by the peripheral
 */
static uint32_t activeFrame[PANEL_W * PANEL_H] __attribute__((aligned(32)));
#endif

/* Local Prototypes */
static void LCD_TaskBody(void *pvParameters);
static void LCD_DrawColorBars(uint32_t *buf);
static void LCD_DrawTaskStats(uint32_t *buf);
static void LCD_DrawTextString(uint32_t *buf, const char *str, uint32_t color);
static int LCD_DrawCharacter(uint32_t *buf, char ch, uint32_t color);
static void LCD_ISR_Handler(void);

/**
 * Initializes the LCD task. Configures the LCD peripheral for the panel and
 * kicks off the FreeRTOS task.
*/
void LCD_TaskInitialize()
{
    mxc_clcd_cfg_t panel;

    panel.width = PANEL_W; //Set the size of the panel in pixels
    panel.height = PANEL_H; //Set the width of the panel in pixels
    panel.frequency = 6400000; //minimum panel supported frequency
    panel.vfrontporch = 2;
    panel.vbackporch = 2;
    panel.vsyncwidth = 10;
    panel.hfrontporch = 12;
    panel.hbackporch = 2;
    panel.hsyncwidth = 70;
    panel.bpp = MXC_BPP24; //24-bit per pixel
    panel.palette = NULL; //Only palletized for lower bit depths
    panel.paletteSize = 0;

    //Initialize the peripheral
    MXC_CLCD_Init(&panel);

#ifdef LCD_DOUBLE_BUFFER
    /* For double buffering, create the semaphores and initialize the empty
     * frame as available to draw, and the newFrame as not yet ready
     */
    newFrameSem = xSemaphoreCreateBinary();
    emptyFrameSem = xSemaphoreCreateBinary();
    xSemaphoreTake(newFrameSem, 0);
    xSemaphoreGive(emptyFrameSem);
    activeFrame = frameBuffer[0];
    runningFrame = frameBuffer[1];
    memset(runningFrame, 0, sizeof(uint32_t) * PANEL_W * PANEL_H);
    MXC_CLCD_SetFrameAddr((void *)runningFrame);

    //Setup the system to get interrupts from the LCD
    MXC_NVIC_SetVector(LCD_IRQn, LCD_ISR_Handler);
    NVIC_SetPriority(LCD_IRQn, configMAX_PRIORITIES);
    NVIC_EnableIRQ(LCD_IRQn);

    //Set interrupts on Address Ready
    MXC_CLCD->int_en = MXC_S_CLCD_INT_EN_ADDR_RDY_IE_EN;
#else
    memset(activeFrame, 0, sizeof(uint32_t) * PANEL_W * PANEL_H);
    MXC_CLCD_SetFrameAddr((void *)activeFrame);
#endif

    //Enable the CLCD to display the image
    MXC_CLCD_Enable();

    //Create the LCD task
    xTaskCreate(LCD_TaskBody, (const char *)"LCD", 1024, NULL, tskIDLE_PRIORITY + 2, &lcdTask);
}

/**
 * Task body for the LCD task.  Periodically draw the updated display.
*/
void LCD_TaskBody(void *pvParameters)
{
    while (1) //Loop forever
    {
//When double buffering, need to wait for a frame to be ready
#ifdef LCD_DOUBLE_BUFFER
        xSemaphoreTake(emptyFrameSem, portMAX_DELAY);
#endif

        //Track time spent here
        _UsageTraceStart();

#if defined(LCD_DISP_COLORBARS)
        LCD_DrawColorBars(activeFrame);
#elif defined(LCD_DISP_TASK_STATS)
        LCD_DrawTaskStats(activeFrame);
#else
//You can add your own drawing routine here
#error "No Display type defined"
#endif

        //Track time spent here
        _UsageTraceEnd();

//When double buffering, notify of a new frame ready
#ifdef LCD_DOUBLE_BUFFER
        xSemaphoreGive(newFrameSem);
#endif

        //Delay until the next frame. This will give us a close match to the
        //desired frame rate. Because of the time it takes to actually do the
        //drawing, we'll be a bit slower.  A more exact implementation would use
        //a timer and have this task wake up at the frame rate period, versus
        //delaying
        vTaskDelay(FRAME_RATE_MS / portTICK_PERIOD_MS);
    }
}

/**
 * Interrupt handler for the LCD. This specifically looks for the Address ready
 * bit, and if a new frame is available does a buffer swap
*/
static void LCD_ISR_Handler()
{
    //Get and clear all the flags
    uint32_t flags = MXC_CLCD->int_stat;
    MXC_CLCD->int_stat = flags;

#ifdef LCD_DOUBLE_BUFFER
    BaseType_t higherPriorityWoken = 0;
    uint32_t *tempAddr;
    if (flags & MXC_F_CLCD_INT_STAT_ADDR_RDY) //Address is available for update
    {
        //Try to take a new frame semaphore, only swap frames if successful
        if (xSemaphoreTakeFromISR(newFrameSem, &higherPriorityWoken) == pdTRUE) {
            tempAddr = runningFrame;
            runningFrame = activeFrame;
            MXC_CLCD_SetFrameAddr((void *)runningFrame);
            activeFrame = tempAddr;

            //Let the task know they can draw on activeFrame again
            xSemaphoreGiveFromISR(emptyFrameSem, &higherPriorityWoken);
        }
    }
#endif
}

#ifdef LCD_DISP_TASK_STATS
/* Buffer for holding task information.
 * FreeRTOS recommends 40 chars per task. Arbitrarily assume 20 tasks or less
 */
#define MAX_TASK_STATUS 20
static char taskStatusBuf[MAX_TASK_STATUS * 40];

/* Grab the font from the MSDK's Provided Library */
extern const unsigned char Liberation_Sans16x16[];

/**
 * Helper for accessing font data. For the 16x16 font, the first byte is the
 * width of the character in pixels, following by 16, 16-bitmapped vertical
 * columns
 */
#pragma pack(1)
typedef struct {
    uint8_t width;
    uint16_t vertPixels[16];
} font16x16_helper_t;
#pragma pack()

/* The Font table in the .C has 4 bytes of header info, skip ahead to the chars */
static const font16x16_helper_t *fontTable = (font16x16_helper_t *)&(Liberation_Sans16x16[4]);

/**
 * Draws the task statistics from FreeRTOS onto the screen based on the data
 * provided by vTaskGetRunTimeStats()
 *
 * It is assumed the buf
 * pointer is the correct depth and size per the PANEL_W and PANEL_H constants.
 *
 * @param buf - Frame buffer to draw
 */
static void LCD_DrawTaskStats(uint32_t *buf)
{
    //Clean slate. Could be smarter here an just erase area's we've touched
    memset(buf, 0, sizeof(uint32_t) * PANEL_W * PANEL_H);

    snprintf(taskStatusBuf, sizeof(taskStatusBuf), "Tick Count: %u", xTaskGetTickCount());
    LCD_DrawTextString(buf + (PANEL_W * 5), taskStatusBuf, 0xFFFFFFFF);
    vTaskGetRunTimeStats(taskStatusBuf);
    LCD_DrawTextString(buf + (PANEL_W * 22), taskStatusBuf, 0xFFFFFFFF);
}

/**
 * Draws a string of text on the display. Buf represents the upper left corner
 * of the display to draw.  Will automatically jump lines when a '\n' is
 * seen.  Loops until a '\0' is found.
 *
 * @param buf - Buffer to draw into
 * @param str - String to draw
 * @param color - Color to use
*/
static void LCD_DrawTextString(uint32_t *buf, const char *str, uint32_t color)
{
    uint32_t *workingBuf = buf;
    uint32_t *lineStart = buf;
    while (*str != '\0') {
        if (*str == '\n') {
            lineStart += PANEL_W * 17;
            workingBuf = lineStart;
        } else {
            workingBuf += LCD_DrawCharacter(workingBuf, *str, color) + 1;
        }
        str++;
    }
}

/**
 * Draws a character on the location of the screen. Buf is the upper left pixel
 * location. Returns the width of the pixel drawn
 *
 * @param buf - Buffer to draw into
 * @param ch - Character to draw
 * @param color - Color to use
 * @returns Widdth in pixels of the character drawn
*/
static int LCD_DrawCharacter(uint32_t *buf, char ch, uint32_t color)
{
    uint32_t *workingBuf;
    const font16x16_helper_t *charData;

    //The font supports ' '  through '~'. If outside that range, draw a space
    if ((ch < ' ') || (ch > '~')) {
        ch = ' ';
    }

    //Grab the pixel data
    charData = &fontTable[ch - ' '];

    //For each row in the font
    for (int y = 0; y < 16; y++) {
        //Get the start of the row
        workingBuf = buf + (PANEL_W * y);

        //For each column in the font
        for (int x = 0; x < 16; x++) {
            //Font is based on vertical pixels, so mask is based on y
            if (charData->vertPixels[x] & (1U << y)) {
                *workingBuf++ = color;
            } else {
                *workingBuf++ = 0x00;
            }
        }
    }

    return charData->width;
}
#endif //LCD_DISP_TASK_STATS

#ifdef LCD_DISP_COLORBARS
/* Color definitions for drawing color bars */
static const uint32_t colors[COLOR_BAR_COUNT] = { 0x00000000, 0x000000FF, 0x0000FF00, 0x0000FFFF,
                                                  0x00FF0000, 0x00FF00FF, 0x00FFFF00, 0x00FFFFFF };

/**
 * Draws a simple color bar test pattern in the buffer. It is assumed the buf
 * pointer is the correct depth and size per the PANEL_W and PANEL_H constants.
 *
 * This does a 'rolling' color bar, where the location of the bars shift by 1
 * column on every refresh.
 *
 * @param buf - Frame buffer to draw
 */
static void LCD_DrawColorBars(uint32_t *buf)
{
    static uint8_t colorIdx = 0; //Non-volatile tracking of color index

    uint32_t *ptr = buf;
    uint32_t color;

    //Loop the height of the frame
    for (int y = 0; y < PANEL_H; y++) {
        //Loop through each color
        for (int i = 0; i < COLOR_BAR_COUNT; i++) {
            //Grab the next colors
            color = colors[colorIdx];

            //Divide the width evenly
            for (int x = 0; x < (PANEL_W / COLOR_BAR_COUNT); x++) {
                *ptr++ = color;
            }

            //Increment the color index
            colorIdx = (colorIdx + 1) % COLOR_BAR_COUNT;
        }
    }

    //Once again at the end to give a rotating effect
    colorIdx = (colorIdx + 1) % COLOR_BAR_COUNT;
}
#endif //LCD_DISP_COLORBARS
