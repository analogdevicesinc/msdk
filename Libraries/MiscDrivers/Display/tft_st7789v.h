/**
 * @file    tft_st7789v.h
 * @brief   Driver for NewHaven 2.4" TFT display
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

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_TFT_ST7789V_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_TFT_ST7789V_H_

#include <stdint.h>
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup bsp
 * @defgroup tft_st7789v NewHaven TFT
 * @{
 */

#define DISPLAY_WIDTH 320 /**< TFT width in pixels. */
#define DISPLAY_HEIGHT 240 /**< TFT height in pixels. */

#define COLUMN_ADDR 0x2A
#define ROW_ADDR 0x2B
#define MEM_WRITE 0x2C
#define MEM_ACCESS_CNTL 0x36

/**
 * @brief Structure defining graphic operation bounding box.
 *
 *  Defines position and size of bounding box.
 */
typedef struct {
    uint16_t x; ///< Horizontal position
    uint16_t y; ///< Vertical position
    uint16_t w; ///< Width of area
    uint16_t h; ///< Height of area
} area_t;

/**
 * @brief Structure defining text data.
 *
 *  Contains pointer to data and its length.
 */
typedef struct {
    char *data; ///< Buffer containing text data
    int len; ///< Length of text data
} text_t;

/**
 * @brief Supported screen rotations.
 *
 *  Defines screen rotation.
 */
typedef enum {
    ROTATE_0, ///< Rotate screen data 0 degrees
    ROTATE_90, ///< Rotate screen data 90 degrees
    ROTATE_180, ///< Rotate screen data 180 degrees
    ROTATE_270 ///< Rotate screen data 270 degrees
} tft_rotation_t;

/**
 * @brief 24-bit RGB to 16-bit RGB conversion macro.
 *
 *  A Macro to convert from 24-bit RGB888 to 16-bit RGB565.
 */
#define RGB(r, g, b)                                      \
    ((b & 0xf8) << 5) | ((g & 0x1c) << 11) | (r & 0xf8) | \
        ((g & 0xe0) >> 5) /**< 5 red | 6 green | 5 blue */

/**
 * @brief A few RGB color definitions.
 *
 *  A few RGB color definitions.
 */
#define BLACK 0x0000 /**<   0,   0,   0 */
#define NAVY 0x000F /**<   0,   0, 128 */
#define DARK_GREEN 0x03E0 /**<   0, 128,   0 */
#define DARK_CYAN 0x03EF /**<   0, 128, 128 */
#define MAROON 0x7800 /**< 128,   0,   0 */
#define PURPLE 0x780F /**< 128,   0, 128 */
#define OLIVE 0x7BE0 /**< 128, 128,   0 */
#define LIGHT_GREY 0xC618 /**< 192, 192, 192 */
#define DARK_GREY 0x7BEF /**< 128, 128, 128 */
#define BLUE 0x001F /**<   0,   0, 255 */
#define GREEN 0x07E0 /**<   0, 255,   0 */
#define CYAN 0x07FF /**<   0, 255, 255 */
#define RED 0xF800 /**< 255,   0,   0 */
#define MAGENTA 0xF81F /**< 255,   0, 255 */
#define YELLOW 0xFFE0 /**< 255, 255,   0 */
#define WHITE 0xFFFF /**< 255, 255, 255 */
#define ORANGE 0xFD20 /**< 255, 165,   0 */
#define GREEN_YELLOW 0xAFE5 /**< 173, 255,  47 */

/*
  These seem very out of place here. Need to find a application/demo specific location for them.
*/

// Image bitmaps
extern const unsigned char img_1_rgb565[];
extern const unsigned char img_2_rgb565[];
extern const unsigned char img_3_rgb565[];
extern const unsigned char img_4_rgb565[];
extern const unsigned char image_pattern_rgb565[];
extern const unsigned char logo_rgb565[];
// Fonts
extern const unsigned char Arial12x12[];
extern const unsigned char Arial24x23[];
extern const unsigned char Arial28x28[];
extern const unsigned char SansSerif19x19[];
extern const unsigned char SansSerif16x16[];

/*
    SPI Transport layer functions
    Board-level files must provide implementations of these functions
    for communicating with the TFT display over SPI.
*/
/**
 * @brief      Initialize the SPI instance connected to the TFT display
 *
 */
extern void TFT_SPI_Init(void);

/**
 * @brief      Write a single byte to the TFT display
 *
 * @param      data     Data value to write
 * @param      cmd      Command/data select.  Set to 1 to write a command byte, set to 0
 *                      to use data mode.  See the ST7789 datasheet for more details
 */
extern void TFT_SPI_Write(uint8_t data, bool cmd);

/**
 * @brief      Transmit a sequence of data bytes to the TFT display
 *
 * @param      src      Pointer to the input data buffer
 * @param      count    Number of data bytes to write from the buffer
 */
extern void TFT_SPI_Transmit(void *src, int count);

/**
 * @brief      Initialize the TFT display
 *
 * @param      reset_ctrl       The GPIO pin configuration for the TFT reset pin.  Use NULL if
 *                              the reset pin of the TFT is not connected to the microcontroller.
 * @param      bl_ctrl          The GPIO pin configuration for the backlight enable pin.  Use NULL if
 *                              the microcontroller does not have control of the backlight enable.
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TFT_Init(mxc_gpio_cfg_t *reset_ctrl, mxc_gpio_cfg_t *bl_ctrl);

/**
 * @brief      Turns backlight on or off
 *
 * @param      on               Zero to turn off, nonzero for on
 */
void MXC_TFT_Backlight(int on);

/**
 * @brief      Fills screen with background color
 *
 */
void MXC_TFT_ClearScreen(void);

/**
 * @brief      Draw and fill a rectangle
 *
 * @param      area             Location and size of rectangle, \ref area_t
 * @param      color            Palette index of rectangle color
 */
void MXC_TFT_FillRect(area_t *area, int color);

/**
 * @brief      Write a Pixel on TFT display
 *
 * @param      pixelX           x location of image
 * @param      pixelY           y location of image
 * @param      width            Width of pixel
 * @param      height           Height of pixel
 * @param      color            RGB value for pixels
 */
void MXC_TFT_WritePixel(int pixelX, int pixelY, int width, int height, uint32_t color);

/**
 * @brief      Draw a bitmap
 *
 * @param      x0               x location of image
 * @param      y0               y location of image
 * @param      id               Bitmap number (pointer)
 */
void MXC_TFT_ShowImage(int x0, int y0, int id);

/**
 * @brief      Draw a mono camera image
 *
 * @param      x0               x location of image
 * @param      y0               y location of image
 * @param      image            Pointer to mono data
 * @param      iWidth           Image width
 * @param      iHeight          Image height
 */
void MXC_TFT_ShowImageCameraMono(int x0, int y0, uint8_t *image, int width, int height);

/**
 * @brief      Draw a camera image
 *
 * @param      x0               x location of image
 * @param      y0               y location of image
 * @param      image            Pointer to RGB data
 * @param      iWidth           Image width
 * @param      iHeight          Image height
 */
void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t *image, int iWidth, int iHeight);

void MXC_TFT_WriteBufferRGB565(int x0, int y0, uint8_t *image, int width, int height);

/**
 * @brief      Fills screen with one color
 *
 * @param      color            Index of screen color
 */
void MXC_TFT_SetBackGroundColor(unsigned int color);

/**
 * @brief      Sets forground color
 *
 * @param      color            Index of forground color
 */
void MXC_TFT_SetForeGroundColor(unsigned int color);

/**
 * @brief      Set palette
 *
 * @param      img_id           Image number to get palette info
 *
 *  \note Unimplemented function.
 */
int MXC_TFT_SetPalette(int img_id);

/**
 * @brief      Set bounds of printf
 *
 * @param      area             Location of printf outputs, \ref area_t
 *
 *  \note Unimplemented function.
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
 * @param      format           Char array formatted like printf
 *             NOTE: up to 3 additional arguments are supported
 * 
 *  \note Unimplemented function.
 */
void MXC_TFT_Printf(const char *format, ...);

/**
 * @brief      Reset cursor to top left corner of printf bounds
 *
 *  \note Unimplemented function.
 */
void MXC_TFT_ResetCursor(void);

/**
 * @brief      Print string with selected font
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       fon_id          Font number
 * @param       str             String which will be display, \ref text_t
 * @param       area            Location of printf outputs, \ref area_t
 */
void MXC_TFT_PrintFont(int x0, int y0, int font_id, text_t *str, area_t *area);

/**
 * @brief      Print string with current font
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       fon_id          Font number
 * @param       str             String which will be display, \ref text_t
 * @param       area            Location of printf outputs, \ref area_t
 */
void MXC_TFT_Print(int x0, int y0, text_t *str, area_t *area);

/**
 * @brief       Clear area on display
 *
 * @param       area            Location on display, \ref area_t
 * @param       color           Palette index of rectangle color
 */
void MXC_TFT_ClearArea(area_t *area, int color);

/**
 * @brief       Set TFT screen rotation
 *
 * @param       rotation        Rotation of the screen, \ref tft_rotation_t
 */
void MXC_TFT_SetRotation(tft_rotation_t rotation);

/**
 * @brief       Write Display Controller Register
 *
 * @param       command         Command or register address
 * @param       data            Data for the command or register
 */
void MXC_TFT_WriteReg(unsigned char command, unsigned char data);

/**
 * @brief      Draw a circle with specified color
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       r               Circle radius
 * @param       color           Circle color
 */
void MXC_TFT_Circle(int x0, int y0, int r, int color);

/**
 * @brief      Fill a circle with specified color
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       r               Circle radius
 * @param       color           Circle color
 */
void MXC_TFT_FillCircle(int x0, int y0, int r, int color);

/**
 * @brief      Draw a line with specified color
 *
 * @param       x0              x start location on screen
 * @param       y0              y start location on screen
 * @param       x1              x end location on screen
 * @param       y1              y end location on screen
 * @param       color           Line color
 */
void MXC_TFT_Line(int x0, int y0, int x1, int y1, int color);

/**
 * @brief      Draw a rectangle with specified color
 *
 * @param       x0              x start location on screen
 * @param       y0              y start location on screen
 * @param       x1              x end location on screen
 * @param       y1              y end location on screen
 * @param       color           Rectangle color
 */
void MXC_TFT_Rectangle(int x0, int y0, int x1, int y1, int color);
/**@} end of group tft_st7789v */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_DISPLAY_TFT_ST7789V_H_
