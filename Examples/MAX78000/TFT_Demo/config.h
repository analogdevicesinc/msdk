#ifndef CONFIG_H
#define CONFIG_H

#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#include "bitmap.h"

// Enable TFT display
#define ENABLE_TFT
// Enable Touchscreen
#define ENABLE_TS
#endif

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"

// Enable TFT display
// #define ENABLE_TFT
// Enable Touchscreen
// #define ENABLE_TS
#endif

#endif
