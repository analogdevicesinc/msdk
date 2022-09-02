/**
 * @file
 * @brief   TFT driver API header file
 */
/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#ifndef _TFT_H_
#define _TFT_H_

#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <gpio.h>

#include "touchscreen.h"

/************************************************************************************/

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} area_t;

typedef struct {
    char* data;
    int len;
} text_t;

typedef enum {
    SCREEN_NORMAL,
    SCREEN_FLIP,
    SCREEN_ROTATE,
} tft_rotation_t;

/************************************************************************************/

/**
 * @brief      Initialize the touchscreen and display
 *
 * @param      tft_spi          The SPI instance the TFT is connected to
 * @param      ss_idx           The SSEL index to use when communicating with the attached TFT
 * @param      reset_ctrl       The GPIO pin configuration for the TFT's reset pin.  Use NULL if
 *                              the reset pin of the TFT is not connected to the microcontroller.
 * @param      bl_ctrl          The GPIO pin configuration for the backlight enable pin.  Use NULL if
 *                              the microcontroller does not have control of the backlight enable.
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TFT_Init(mxc_spi_regs_t* tft_spi, int ss_idx, mxc_gpio_cfg_t* reset_ctrl,
                 mxc_gpio_cfg_t* bl_ctrl);

/**
 * @brief      Turns backlight on or off
 *
 * @param      on           Zero to turn off, nonzero for on
 */
void MXC_TFT_Backlight(int on);

/**
 * @brief      Fills screen with background color
 *
 */
void MXC_TFT_ClearScreen(void);

/**
 * @brief      Draw a rectangle
 *
 * @param      area   Location and size of rectangle
 * @param      color  Palette index of rectangle color
 */
void MXC_TFT_FillRect(area_t* area, int color);

/**
 * @brief      Write a Pixel on TFT display
 *
 * @param      pixelX           x location of image
 * @param      pixelY           y location of image
 * @param      width            width of pixel
 * @param      height           height of pixel
 * @param      color            RGB value of image
 */
void MXC_TFT_WritePixel(int pixelX, int pixelY, int width, int height, uint32_t color);

/**
 * @brief      Draw a bitmap
 *
 * @param      x0           x location of image
 * @param      y0           y location of image
 * @param      id           Bitmap number
 */
void MXC_TFT_ShowImage(int x0, int y0, int id);

void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t* image, int iWidth, int iHeight);

/**
 * @brief      Fills screen with one color
 *
 * @param      index_color  Palette index of screen color
 */
void MXC_TFT_SetBackGroundColor(unsigned int index_color);

/**
 * @brief      Set bounds of printf
 *
 * @param      img_id  Image number to get palette info
 */
int MXC_TFT_SetPalette(int img_id);

/**
 * @brief      Set bounds of printf
 *
 * @param      area   Location of printf outputs
 */
void MXC_TFT_ConfigPrintf(area_t* area);

/**
 * @brief       Change font
 *
 * @param       font_id         Font id
 */
void MXC_TFT_SetFont(int font_id);

/**
 * @brief      Printf out to display
 *
 * @param      format  Char array formatted like printf
 *             NOTE: up to 3 additional arguments are supported
 */
void MXC_TFT_Printf(const char* format, ...);

/**
 * @brief      Reset cursor to top left corner of printf bounds
 *
 */
void MXC_TFT_ResetCursor(void);

/**
 * @brief      Print string with selected font
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       fon_id          Font number
 * @param       str             String which will be display
 * @param       area            Location of printf outputs
 */
void MXC_TFT_PrintFont(int x0, int y0, int font_id, text_t* str, area_t* area);

/**
 * @brief      Print string with current font
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       fon_id          Font number
 * @param       str             String which will be display
 * @param       area            Location of printf outputs
 */
void MXC_TFT_Print(int x0, int y0, text_t* str, area_t* area);

/**
 * @brief       Clear area on display
 *
 * @param       area            Location on display
 * @param       color           Palette index of rectangle color
 */
void MXC_TFT_ClearArea(area_t* area, int color);

/**
 * @brief       Set TFT screen rotation
 *
 * @param       rotation            rotation of the screen
 */
void MXC_TFT_SetRotation(tft_rotation_t rotation);

/**
 * @brief       Write Screen Register
 *
 * @param       command             command or register address
 * @param       data                data for the command or register
 */
void MXC_TFT_WriteReg(unsigned short command, unsigned short data);

#endif /* _TFT_H_ */
