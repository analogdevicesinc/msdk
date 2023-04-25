#ifndef TFT_UTILS_H
#define TFT_UTILS_H

#include "tft_st7789v.h"

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define LINE_WIDTH 2

void TFT_Print(char *str, int x, int y, int font, int length);
void draw_obj_rect(float* xy, int class_idx);

#endif