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

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_TFT_ILI9341_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_TFT_ILI9341_H_

#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <gpio.h>

/************************************ DEFINES ********************************/
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define TFT_SPI_FREQ 25000000 // Hz
#define TFT_SPI0_PINS MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7 | MXC_GPIO_PIN_11
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

typedef enum { ROTATE_0, ROTATE_90, ROTATE_180, ROTATE_270 } tft_rotation_t;

#define RGB(r, g, b) \
    (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3)) //5 red | 6 green | 5 blue

/* some RGB color definitions */
#define BLACK 0x0000 /*   0,   0,   0 */
#define NAVY 0x000F /*   0,   0, 128 */
#define DARK_GREEN 0x03E0 /*   0, 128,   0 */
#define DARK_CYAN 0x03EF /*   0, 128, 128 */
#define MAROON 0x7800 /* 128,   0,   0 */
#define PURPLE 0x780F /* 128,   0, 128 */
#define OLIVE 0x7BE0 /* 128, 128,   0 */
#define LIGTH_GREY 0xC618 /* 192, 192, 192 */
#define DARK_GREY 0x7BEF /* 128, 128, 128 */
#define BLUE 0x001F /*   0,   0, 255 */
#define GREEN 0x07E0 /*   0, 255,   0 */
#define CYAN 0x07FF /*   0, 255, 255 */
#define RED 0xF800 /* 255,   0,   0 */
#define MAGENTA 0xF81F /* 255,   0, 255 */
#define YELLOW 0xFFE0 /* 255, 255,   0 */
#define WHITE 0xFFFF /* 255, 255, 255 */
#define ORANGE 0xFD20 /* 255, 165,   0 */
#define GREEN_YELLOW 0xAFE5 /* 173, 255,  47 */

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

/************************************************************************************/

/**
 * @brief      Initialize the TFT display
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
int MXC_TFT_Init(mxc_spi_regs_t *tft_spi, int ss_idx, mxc_gpio_cfg_t *reset_ctrl,
                 mxc_gpio_cfg_t *bl_ctrl);

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
 * @brief      Draw and fill a rectangle
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
 * @param      id           Bitmap number (pointer)
 */
void MXC_TFT_ShowImage(int x0, int y0, int id);

/**
 * @brief      Draw a RGB565 buffer to a window location
 *
 * @param      x0           x location of image
 * @param      y0           y location of image
 * @param      image        RGB565 image buffer (pointer)
 * @param      width        image width
 * @param      height       image height
 */
void MXC_TFT_WriteBufferRGB565(int x0, int y0, uint8_t *image, int width, int height);

/**
 * @brief      Display captured RGB565 image on TFT
 *
 * @param      x0           x location of image
 * @param      y0           y location of image
 * @param      image        RGB565 image (pointer)
 * @param      iWidth       image width
 * @param      iHeight      image height
 */
void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t *image, int iWidth, int iHeight);

/**
 * @brief      Fills screen with one color
 *
 * @param      index_color  index of screen color
 */
void MXC_TFT_SetBackGroundColor(unsigned int index_color);

void MXC_TFT_SetForeGroundColor(unsigned int color);

/**
 * @brief      Set palette
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
 * @param       rotation        Rotation of the screen
 */
void MXC_TFT_SetRotation(tft_rotation_t rotation);

/**
 * @brief       Write Screen Register
 *
 * @param       command        command or register address
 * @param       data           data for the command or register
 */
void MXC_TFT_WriteReg(unsigned char command, unsigned char data);

/**
 * @brief      Draw a circle with specified color
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       r               circle radius
 * @param       color           circle color
 */
void MXC_TFT_Circle(int x0, int y0, int r, int color);

/**
 * @brief      Fill a circle with specified color
 *
 * @param       x0              x location on screen
 * @param       y0              y location on screen
 * @param       r               circle radius
 * @param       color           circle color
 */
void MXC_TFT_FillCircle(int x0, int y0, int r, int color);

/**
 * @brief      Draw a line with specified color
 *
 * @param       x0              x start location on screen
 * @param       y0              y start location on screen
 * @param       x1              x end location on screen
 * @param       y1              y end location on screen
 * @param       color           line color
 */
void MXC_TFT_Line(int x0, int y0, int x1, int y1, int color);

/**
 * @brief      Draw a rectangle with specified color
 *
 * @param       x0              x start location on screen
 * @param       y0              y start location on screen
 * @param       x1              x end location on screen
 * @param       y1              y end location on screen
 * @param       color           rectangle color
 */
void MXC_TFT_Rectangle(int x0, int y0, int x1, int y1, int color);

/**
 * @brief      Stream camera to TFT (feather only)
 *
 * @param      x0           x location of image
 * @param      y0           y location of image
 * @param      width        image width
 * @param      height       image height
 */
void MXC_TFT_Stream(int x0, int y0, int width, int height);

#endif // LIBRARIES_MISCDRIVERS_DISPLAY_TFT_ILI9341_H_
