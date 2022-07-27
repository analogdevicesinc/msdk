#ifndef CONFIG_H
#define CONFIG_H

#ifdef BOARD_EVKIT_V1
#define ENABLE_TFT
#include "bitmap.h"
#include "tft_ssd2119.h"
#endif

#ifdef BOARD_FTHR_REVA
// #define ENABLE_TFT
#include "tft_ili9341.h"
#endif

// Comment out USE_SAMPLEDATA to use Camera module
//#define USE_SAMPLEDATA

#define CAMERA_TO_LCD (1)
#define IMAGE_SIZE_X  (64)
#define IMAGE_SIZE_Y  (64)
#define CAMERA_FREQ   (10 * 1000 * 1000)
#define TFT_BUFF_SIZE 50 // TFT buffer size

#ifdef BOARD_EVKIT_V1
int image_bitmap_1 = img_1_bmp;
int image_bitmap_2 = logo_white_bg_darkgrey_bmp;
int font_1         = urw_gothic_12_white_bg_grey;
int font_2         = urw_gothic_13_white_bg_grey;
#endif
#ifdef BOARD_FTHR_REVA
int image_bitmap_1 = (int)&img_1_rgb565[0];
int image_bitmap_2 = (int)&logo_rgb565[0];
int font_1         = (int)&SansSerif16x16[0];
int font_2         = (int)&SansSerif16x16[0];
#endif

#endif
