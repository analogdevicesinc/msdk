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

/* **** Includes **** */

#include <stdint.h>

/* **** Definitions **** */

#define SCREEN_ORIENTATION          (SCREEN_ROTATE)

// Will be different from actual screen dimensions depending on screen orientation
#define SCREEN_WIDTH                    (320)
#define SCREEN_HEIGHT                   (240)

// TODO(SW): Automatically calculate these sizes based on screen dimensions.
#define GRID_OFFSET_X                   (80)
#define GRID_OFFSET_Y                   (0)
#define GRID_LENGTH                     (224)
#define GRID_SPACING                    (8) // spacing between edge of "screen to grid".

#define BLOCK_LENGTH                    (51)
#define BLOCK_SPACING                   (4) // spacing between edge of "grid to block" and "block to block".

#define RADIUS_FOR_CORNERS              (3)

// Colors

#define RGB565_WHITE                    (0xFFFF)
#define RGB565_BLACK                    (0x0000)
#define RGB565_DARK_GRAY                (0x3084)
#define RGB565_LIGHT_GRAY               (0x5AEB)

// Block colors (2-2048) are taken from original game: https://github.com/gabrielecirulli/2048?tab=readme-ov-file
// Open-source under MIT License.
#define RGB565_BLOCK_2                  (0xEF3B)
#define RGB565_BLOCK_4                  (0xEF19)
#define RGB565_BLOCK_8                  (0xF58F)
#define RGB565_BLOCK_16                 (0xF4AC)
#define RGB565_BLOCK_32                 (0xF3EB)
#define RGB565_BLOCK_64                 (0xF2E7)
#define RGB565_BLOCK_128                (0xEE6E)
#define RGB565_BLOCK_256                (0xEE6C)
#define RGB565_BLOCK_512                (0xEE4A)
#define RGB565_BLOCK_1024               (0xEE27)
#define RGB565_BLOCK_2048               (0xEE05)

#define RGB565_BLOCK_4096               (0xFDF7)
#define RGB565_BLOCK_8192               (0xF38E)
#define RGB565_BLOCK_16384              (0xFC58)
#define RGB565_BLOCK_32768              (0xB43C)
#define RGB565_BLOCK_65536              (0x8BF9)
#define RGB565_BLOCK_131072             (0x6C3C)

// For my non-American English Speakers :)
#define RGB565_DARK_GREY                RGB565_DARK_GRAY
#define RGB565_LIGHT_GREY               RGB565_LIGHT_GRAY

// Formatted Colors
#define FORMAT_RGB565_TO_PACKET(RGB)    (0x01000100 | ((RGB & 0x00FF) << 16) | ((RGB & 0xFF00) >> 8))

// 'F_' prefix stands for "formatted into packets".
#define F_BACKGROUND_COLOR              FORMAT_RGB565_TO_PACKET(RGB565_WHITE)
#define F_GRID_COLOR                    FORMAT_RGB565_TO_PACKET(RGB565_DARK_GRAY)
#define F_EMPTY_BLOCK_COLOR             FORMAT_RGB565_TO_PACKET(RGB565_LIGHT_GRAY)

#define RGB_BLOCK_COLOR(block)          ((block) == 2 ? RGB565_BLOCK_2 \
                                            : (block) == 4 ? RGB565_BLOCK_4 \
                                            : (block) == 8 ? RGB565_BLOCK_8 \
                                            : (block) == 16 ? RGB565_BLOCK_16 \
                                            : (block) == 32 ? RGB565_BLOCK_32 \
                                            : (block) == 64 ? RGB565_BLOCK_64 \
                                            : (block) == 128 ? RGB565_BLOCK_128 \
                                            : (block) == 256 ? RGB565_BLOCK_256 \
                                            : (block) == 512 ? RGB565_BLOCK_512 \
                                            : (block) == 1024 ? RGB565_BLOCK_1024 \
                                            : (block) == 2048 ? RGB565_BLOCK_2048 \
                                            : BLOCK_2_DIGIT_PX_HEIGHT)

#define FORMATTED_RGB_BLOCK_COLOR(block)    FORMAT_RGB565_TO_PACKET(RGB_BLOCK_COLOR(block))


typedef enum {
    GRAPHICS_SLIDE_DIR_LEFT,
    GRAPHICS_SLIDE_DIR_RIGHT,
    GRAPHICS_SLIDE_DIR_UP,
    GRAPHICS_SLIDE_DIR_DOWN
} graphics_slide_direction_t;

/* **** Function Prototypes **** */

int Graphics_Init(void);

void Graphics_AddNewBlock(int row, int col, int block_2_4);

void Graphics_AddBlock(int row, int col, int value);

void Graphics_CombineBlocks(int row, int col, int new_value);

void Graphics_EraseSingleBlock(int row, int col, graphics_slide_direction_t direction);
