/**
 * @file
 * @brief   TFT driver API header file
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_TFT_SSD2119_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_TFT_SSD2119_H_

#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <gpio.h>

#if defined(TS_TSC2007)
#include "tsc2007.h"
#else
#include "tsc2046.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************************/

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} area_t;

typedef struct {
    char *data;
    int len;
} text_t;

typedef enum {
    SCREEN_NORMAL,
    SCREEN_FLIP,
    SCREEN_ROTATE,
} tft_rotation_t;

typedef struct {
    mxc_spi_regs_t *regs; // The SPI instance the touchscreen controller is connected to
    mxc_gpio_cfg_t gpio; // The SPI pins: MOSI, MISO, SCK, SS, must define all them
    unsigned int freq; // The SPI frequency
    int ss_idx; // The SSEL index to use when communicating with the touchscreen controller
} mxc_tft_spi_config;

/************************************************************************************/

/**
 * @brief      Used to register hw related configuration, need to be called before MXC_TFT_Init()
 *
 * @param      spi_config       TFT SPI configuration, SPI instance, freq, pins slave slect id
 * @param      reset_ctrl       The GPIO pin configuration for the TFT's reset pin.  Use NULL if
 *                              the reset pin of the TFT is not connected to the microcontroller.
 * @param      bl_ctrl          The GPIO pin configuration for the backlight enable pin.  Use NULL if
 *                              the microcontroller does not have control of the backlight enable.
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TFT_PreInit(mxc_tft_spi_config *spi_config, mxc_gpio_cfg_t *reset_ctrl,
                    mxc_gpio_cfg_t *bl_ctrl);

/**
 * @brief      Initialize the TFT display
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TFT_Init(void);

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
void MXC_TFT_FillRect(area_t *area, int color);

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

void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t *image, int iWidth, int iHeight);

void MXC_TFT_ShowImageCameraMono(int x0, int y0, uint8_t *image, int iWidth, int iHeight);

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
void MXC_TFT_ConfigPrintf(area_t *area);

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
void MXC_TFT_Printf(const char *format, ...);

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
void MXC_TFT_PrintFont(int x0, int y0, int font_id, text_t *str, area_t *area);

/**
 * @brief      Print string with current font
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       fon_id          Font number
 * @param       str             String which will be display
 * @param       area            Location of printf outputs
 */
void MXC_TFT_Print(int x0, int y0, text_t *str, area_t *area);

/**
 * @brief       Clear area on display
 *
 * @param       area            Location on display
 * @param       color           Palette index of rectangle color
 */
void MXC_TFT_ClearArea(area_t *area, int color);

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
void MXC_TFT_WriteReg(uint16_t command, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_DISPLAY_TFT_SSD2119_H_
