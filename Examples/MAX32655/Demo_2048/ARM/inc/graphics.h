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

#ifndef EXAMPLES_MAX32655_DEMO_2048_ARM_INC_GRAPHICS_H_
#define EXAMPLES_MAX32655_DEMO_2048_ARM_INC_GRAPHICS_H_

/* **** Includes **** */

#include <stdint.h>

/* **** Definitions **** */

#define SCREEN_ORIENTATION (SCREEN_ROTATE)

// Will be different from actual screen dimensions depending on screen orientation
#define SCREEN_WIDTH (320)
#define SCREEN_HEIGHT (240)

// TODO(SW): Automatically calculate these sizes based on screen dimensions.
// Set the dimensions of all the models.
#define GRID_OFFSET_X (80)
#define GRID_OFFSET_Y (0)
#define GRID_LENGTH (224)
#define GRID_SPACING (8) // spacing between edge of "screen to grid".

#define BLOCK_LENGTH (51)
#define BLOCK_SPACING (4) // spacing between edge of "grid to block" and "block to block".

#define RADIUS_FOR_CORNERS (3)

#define CFS_LOGO_OFFSET_X (4)
#define CFS_LOGO_OFFSET_Y (0)

#define GAME_LOGO_OFFSET_X (CFS_LOGO_OFFSET_X + (GRID_OFFSET_X - BLOCK_LENGTH) / 2)
#define GAME_LOGO_OFFSET_Y (80)

// Position settings for timer.
#define TIME_OFFSET_Y (215)
#define TIME_COLON_OFFSET_X (CFS_LOGO_OFFSET_X + (GRID_OFFSET_X / 2) - 1)
#define TIME_DIGIT_WIDTH (15) // Max width of digit is 15 pixels
#define TIME_DIGIT3_OFFSET_X(width)                                                                \
    ((TIME_COLON_OFFSET_X - (TIME_DIGIT_WIDTH * 2)) + ((TIME_DIGIT_WIDTH / 2) - (width / 2) - 1) - \
     (GAME_TEXT_DIGIT_COLON_WIDTH / 2))
#define TIME_DIGIT2_OFFSET_X(width)                                                                \
    ((TIME_COLON_OFFSET_X - (TIME_DIGIT_WIDTH * 1)) + ((TIME_DIGIT_WIDTH / 2) - (width / 2) - 1) - \
     (GAME_TEXT_DIGIT_COLON_WIDTH / 2))
#define TIME_DIGIT1_OFFSET_X(width)                              \
    ((TIME_COLON_OFFSET_X + (GAME_TEXT_DIGIT_COLON_WIDTH * 2)) + \
     ((TIME_DIGIT_WIDTH / 2) - (width / 2)) - 1)
#define TIME_DIGIT0_OFFSET_X(width)                                                 \
    ((TIME_COLON_OFFSET_X + (GAME_TEXT_DIGIT_COLON_WIDTH * 2) + TIME_DIGIT_WIDTH) + \
     ((TIME_DIGIT_WIDTH / 2) - (width / 2)) - 1)

// Position settings for moves counter.
#define MOVES_DIGITS_OFFSET_Y (160)
#define MOVES_TEXT_OFFSET_Y                            \
    (MOVES_DIGITS_OFFSET_Y + GAME_TEXT_DIGITS_HEIGHT + \
     4) // 4 seemed to be appropriate number of pxiels for spacing.
#define MOVES_DIGIT_WIDTH (15) // Max width of digit is 15 pixels.
#define MOVES_DIGITS_OFFSET_X ((GRID_OFFSET_X / 2) - MOVES_DIGIT_WIDTH - (MOVES_DIGIT_WIDTH / 2))
#define MOVES_TEXT_OFFSET_X ((GRID_OFFSET_X / 2) - (GAME_TEXT_MOVES_WIDTH / 2) + 4)
#define MOVES_DIGIT3_OFFSET_X(width) \
    (MOVES_DIGITS_OFFSET_X + (MOVES_DIGIT_WIDTH * 0) + (MOVES_DIGIT_WIDTH / 2) - (width - 2))
#define MOVES_DIGIT2_OFFSET_X(width) \
    (MOVES_DIGITS_OFFSET_X + (MOVES_DIGIT_WIDTH * 1) + (MOVES_DIGIT_WIDTH / 2) - (width - 2))
#define MOVES_DIGIT1_OFFSET_X(width) \
    (MOVES_DIGITS_OFFSET_X + (MOVES_DIGIT_WIDTH * 2) + (MOVES_DIGIT_WIDTH / 2) - (width - 2))
#define MOVES_DIGIT0_OFFSET_X(width) \
    (MOVES_DIGITS_OFFSET_X + (MOVES_DIGIT_WIDTH * 3) + (MOVES_DIGIT_WIDTH / 2) - (width - 2))

// Position settings for Game Over and You Win boxes,
#define GAME_OVER_BOX_OFFSET_X \
    (GRID_OFFSET_X + ((SCREEN_WIDTH - GRID_OFFSET_X) / 2) - (GAME_OVER_BOX_WIDTH / 2))
#define GAME_OVER_BOX_OFFSET_Y \
    (GRID_OFFSET_Y + ((SCREEN_HEIGHT - GRID_OFFSET_Y) / 2) - (GAME_OVER_BOX_HEIGHT / 2))
#define YOU_WIN_BOX_OFFSET_X \
    (GRID_OFFSET_X + ((SCREEN_WIDTH - GRID_OFFSET_X) / 2) - (YOU_WIN_BOX_WIDTH / 2))
#define YOU_WIN_BOX_OFFSET_Y \
    (GRID_OFFSET_Y + ((SCREEN_HEIGHT - GRID_OFFSET_Y) / 2) - (YOU_WIN_BOX_HEIGHT / 2))

// Colors 16-bit RGB565.
#define RGB565_WHITE (0xFFFF)
#define RGB565_BLACK (0x0000)
#define RGB565_DARK_GRAY (0x3084)
#define RGB565_LIGHT_GRAY (0x5AEB)

// Block colors (2-2048) are taken from original game: https://github.com/gabrielecirulli/2048?tab=readme-ov-file
// Open-source under MIT License.
#define RGB565_BLOCK_2 (0xEF3B)
#define RGB565_BLOCK_4 (0xEF19)
#define RGB565_BLOCK_8 (0xF58F)
#define RGB565_BLOCK_16 (0xF4AC)
#define RGB565_BLOCK_32 (0xF3EB)
#define RGB565_BLOCK_64 (0xF2E7)
#define RGB565_BLOCK_128 (0xEE6E)
#define RGB565_BLOCK_256 (0xEE6C)
#define RGB565_BLOCK_512 (0xEE4A)
#define RGB565_BLOCK_1024 (0xEE27)
#define RGB565_BLOCK_2048 (0xEE05)

// Unused, but left here if anyone wants to expand features.
#define RGB565_BLOCK_4096 (0xFDF7)
#define RGB565_BLOCK_8192 (0xF38E)
#define RGB565_BLOCK_16384 (0xFC58)
#define RGB565_BLOCK_32768 (0xB43C)
#define RGB565_BLOCK_65536 (0x8BF9)
#define RGB565_BLOCK_131072 (0x6C3C)

// For my non-American English Speakers :)
#define RGB565_DARK_GREY RGB565_DARK_GRAY
#define RGB565_LIGHT_GREY RGB565_LIGHT_GRAY

// Formatted Colors
#define FORMAT_RGB565_TO_PACKET(RGB) (0x01000100 | ((RGB & 0x00FF) << 16) | ((RGB & 0xFF00) >> 8))

// 'F_' prefix stands for "formatted into packets".
#define F_BACKGROUND_COLOR FORMAT_RGB565_TO_PACKET(RGB565_WHITE)
#define F_GRID_COLOR FORMAT_RGB565_TO_PACKET(RGB565_DARK_GRAY)
#define F_EMPTY_BLOCK_COLOR FORMAT_RGB565_TO_PACKET(RGB565_LIGHT_GRAY)

#define RGB_BLOCK_COLOR(block)             \
    ((block) == 2    ? RGB565_BLOCK_2 :    \
     (block) == 4    ? RGB565_BLOCK_4 :    \
     (block) == 8    ? RGB565_BLOCK_8 :    \
     (block) == 16   ? RGB565_BLOCK_16 :   \
     (block) == 32   ? RGB565_BLOCK_32 :   \
     (block) == 64   ? RGB565_BLOCK_64 :   \
     (block) == 128  ? RGB565_BLOCK_128 :  \
     (block) == 256  ? RGB565_BLOCK_256 :  \
     (block) == 512  ? RGB565_BLOCK_512 :  \
     (block) == 1024 ? RGB565_BLOCK_1024 : \
     (block) == 2048 ? RGB565_BLOCK_2048 : \
                       BLOCK_2_DIGIT_PX_HEIGHT)

#define FORMATTED_RGB_BLOCK_COLOR(block) FORMAT_RGB565_TO_PACKET(RGB_BLOCK_COLOR(block))

/**
 *  These enums help keep track of what blocks were erased,
 *      combined, or didn't move to help optimize and select
 *      the animation of for the display.
 *  IMPORTANT: Sync these commands with the RISCV core.
 */
typedef enum { EMPTY = 0, ERASE = 1, COMBINE = 2, UNMOVED = 3 } block_state_t;

/**
 *  These enums help keep track of the game state.
 *  IMPORTANT: Sync these commands with the RISCV core.
 */
typedef enum {
    IN_PROGRESS = 0,
    GAME_OVER = 1,
    WINNER = 2,
} game_state_t;

/* **** Function Prototypes **** */

/**
 *  This enum is used to keep track of which direction the blocks will slide to
 *      for the graphics.
 *  IMPORTANT: Sync these directions with the RISCV core.
 */
typedef enum {
    GRAPHICS_SLIDE_DIR_UP = 0,
    GRAPHICS_SLIDE_DIR_DOWN = 1,
    GRAPHICS_SLIDE_DIR_LEFT = 2,
    GRAPHICS_SLIDE_DIR_RIGHT = 3,
} graphics_slide_direction_t;

/* **** Function Prototypes **** */

/**
 * @brief   Initializes the Graphics of the 2048 Game Demo.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int Graphics_Init(void);

/**
 * @brief   Draw the new block (2 or 4) spawn animation on the grid. This
 *          function runs when the existing grid has finished updating and
 *          a new block needs to be drawn.
 *
 * @param   row         Row number (indexed 0).
 * @param   col         Column number (indexed 0).
 * @param   block_2_4   Pass in 2 for Block 2 spawn, 4 for Block 4 spawn.
 */
void Graphics_AddNewBlock(int row, int col, int block_2_4);

/**
 * @brief   Draws a block on the grid (no animation). This function is
 *          is mainly used for re-drawing existing blocks when they're moved.
 *
 * @param   row         Row number (indexed 0).
 * @param   col         Column number (indexed 0).
 * @param   value       Value of block to draw on grid.
 */
void Graphics_AddBlock(int row, int col, int value);

/**
 * @brief   Draws a new combined block (including blow-up/shrink-down animation).
 *
 * @param   row         Row number (indexed 0).
 * @param   col         Column number (indexed 0).
 * @param   new_value   Value of combined block to draw on grid.
 */
void Graphics_CombineSingleBlock(int row, int col, int new_value);

/**
 * @brief   Draws all blocks that were combined. This function not only draws the combined blocks,
 *          but also draws the blow-up/shrink-down animation. This function should be called
 *          after all the blocks that moved to their final locations (Graphics_AddBlock())
 *          have been drawn to display.
 *
 * @param   grid        Pointer to main 2048 grid (2-D array) which holds the current positions
 *                      of all valid blocks.
 * @param   grid_state  Pointer to grid state (2-D array) which holds the state of the blocks that
 *                      needs to combined.
 */
void Graphics_CombineBlocks(uint32_t grid[4][4], block_state_t grid_state[4][4]);

/**
 * @brief   Erases all moving blocks from the grid. This function is used before
 *          drawing the blocks that moved to their final location.
 *
 * @param   grid_state  Pointer to grid (2-D array) which holds the state of the blocks that
 *                      needs to be erased because they're moving.
 * @param   direction   Direction that the blocks are moving to.
 */
void Graphics_EraseBlocks(block_state_t grid_state[4][4], graphics_slide_direction_t direction);

/**
 * @brief   Erases the the block from the grid. This function is mainly used
 *          when blocks are moving which need to be erased, then redrawned.
 *
 * @param   row         Row number (indexed 0).
 * @param   col         Column number (indexed 0).
 */
void Graphics_EraseSingleBlock(int row, int col);

/**
 * @brief   Update the timer on the display.
 *
 * @param   total_seconds   The current total second value of RTC.
 */
void Graphics_SetTime(uint32_t total_seconds);

/**
 * @brief   Update the moves counter on the display.
 *
 * @param   moves_count The number of moves that have been executed.
 */
void Graphics_UpdateMovesCount(uint32_t moves_count);

/**
 * @brief   Draws the "game over" box popup when game is finished.
 */
void Graphics_DisplayGameOver(void);

/**
 * @brief   Draws the "you win" box popup when game is finished.
 */
void Graphics_DisplayYouWin(void);

#endif // EXAMPLES_MAX32655_DEMO_2048_ARM_INC_GRAPHICS_H_
