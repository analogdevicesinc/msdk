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
#include "mxc_device.h"
#include "mxc_delay.h"
#include "tft_ssd2119.h"

// Application Libraries.
#include "graphics.h"
#include "clear_sans_bold_scaled_block_digits.h"
#include "clear_sans_bold_game_text.h"
#include "cfs_logo.h"
#include "end_game_text.h"

/* **** Definitions **** */

// Focus on top left corner of the top left block in the grid to help with visualize calculating the coordinates.
#define BLOCK_X_POSITION(col) \
    (GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * (col)))
#define BLOCK_Y_POSITION(row) \
    (GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * (row)))
// Math is to center the digits printed to the newly created block.
#define DIGIT_CENTER_X_POSITION(x, value) \
    (x + (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_WIDTH(value) / 2))
#define DIGIT_CENTER_Y_POSITION(y, value) \
    (y + (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_HEIGHT(value) / 2))

// Test out delays so that it's not too slow and not too fast for human eye.
#define COMBINE_BLOCKS_ANIMATE_MAX_DELAY_MS (15)

/* **** Globals **** */

static int prev_timer_digit1 = 0;
static int prev_timer_digit2 = 0;
static int prev_timer_digit3 = 0;
static int prev_moves_digit1 = 0;
static int prev_moves_digit2 = 0;
static int prev_moves_digit3 = 0;

/* **** Functions **** */

int Graphics_Init(void)
{
    int error;

    // Initialize TFT Display.
    error = MXC_TFT_Init();
    if (error != E_NO_ERROR) {
        return error;
    }

    // Clear display.
    MXC_TFT_DrawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, F_BACKGROUND_COLOR);

    // Draw CFS Logo.
    MXC_TFT_DrawBitmap(CFS_LOGO_OFFSET_X, CFS_LOGO_OFFSET_Y, CFS_LOGO_WIDTH, CFS_LOGO_HEIGHT,
                       cfs_logo);

    // Draw Game Logo.
    // Forgive me for all the math who ever tries to read through the code.
    // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
    int x = GAME_LOGO_OFFSET_X;
    int y = GAME_LOGO_OFFSET_Y;

    // Math is to center the digits printed to the newly created block.
    int dx = x + (BLOCK_LENGTH / 2) - (BLOCK_2048_DIGIT_PX_WIDTH / 2);
    int dy = y + (BLOCK_LENGTH / 2) - (BLOCK_2048_DIGIT_PX_HEIGHT / 2);

    // Draw block.
    MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                            FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_2048), RADIUS_FOR_CORNERS,
                            F_BACKGROUND_COLOR);

    // Draw digits.
    // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
    MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_2048_DIGIT_PX_WIDTH, BLOCK_2048_DIGIT_PX_HEIGHT,
                           block_2048, RGB565_BLACK, RGB565_BLOCK_2048);

    // Draw move counter.
    MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT0_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)),
                                   MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(0),
                                   GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK,
                                   RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT1_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)),
                                   MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(0),
                                   GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK,
                                   RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT2_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)),
                                   MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(0),
                                   GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK,
                                   RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT3_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)),
                                   MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(0),
                                   GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK,
                                   RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(MOVES_TEXT_OFFSET_X, MOVES_TEXT_OFFSET_Y, GAME_TEXT_MOVES_WIDTH,
                                   GAME_TEXT_MOVES_HEIGHT, game_text_moves, RGB565_BLACK,
                                   RGB565_WHITE);

    // Draw timer.
    MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT0_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)), TIME_OFFSET_Y,
                                   GAME_TEXT_DIGIT_WIDTH(0), GAME_TEXT_DIGITS_HEIGHT,
                                   GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK, RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT1_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)), TIME_OFFSET_Y,
                                   GAME_TEXT_DIGIT_WIDTH(0), GAME_TEXT_DIGITS_HEIGHT,
                                   GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK, RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(TIME_COLON_OFFSET_X, TIME_OFFSET_Y, GAME_TEXT_DIGIT_COLON_WIDTH,
                                   GAME_TEXT_DIGITS_HEIGHT, game_text_colon, RGB565_BLACK,
                                   RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT2_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)), TIME_OFFSET_Y,
                                   GAME_TEXT_DIGIT_WIDTH(0), GAME_TEXT_DIGITS_HEIGHT,
                                   GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK, RGB565_WHITE);
    MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT3_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(0)), TIME_OFFSET_Y,
                                   GAME_TEXT_DIGIT_WIDTH(0), GAME_TEXT_DIGITS_HEIGHT,
                                   GAME_TEXT_DIGIT_PTR(0), RGB565_BLACK, RGB565_WHITE);

    // Draw grid.
    MXC_TFT_DrawRect(GRID_OFFSET_X + GRID_SPACING, GRID_OFFSET_Y + GRID_SPACING, GRID_LENGTH,
                     GRID_LENGTH, F_GRID_COLOR);
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
            MXC_TFT_DrawRoundedRect(GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING +
                                        ((BLOCK_LENGTH + BLOCK_SPACING) * row),
                                    GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING +
                                        ((BLOCK_LENGTH + BLOCK_SPACING) * col),
                                    BLOCK_LENGTH, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR,
                                    RADIUS_FOR_CORNERS, F_GRID_COLOR);
        }
    }

    return E_NO_ERROR;
}

void Graphics_AddNewBlock(int row, int col, int block_2_4)
{
    // 6 is chosen based on animation/display refresh rate speed to the naked eye.
    for (int frame_i = 0; frame_i < 6; frame_i++) {
        // Forgive the unreadability.
        //  The math calculates where each block needs to be drawn as its animated to grow from small to big.

        // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
        int x = ((BLOCK_LENGTH / 2) - (frame_i * 5)) + GRID_OFFSET_X + GRID_SPACING +
                BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
        int y = ((BLOCK_LENGTH / 2) - (frame_i * 5)) + GRID_OFFSET_Y + GRID_SPACING +
                BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

        // Increase the block size by 10 pixels
        MXC_TFT_DrawRoundedRect(
            x, y, ((frame_i * 10) > BLOCK_LENGTH) ? BLOCK_LENGTH : ((frame_i * 10) + 1),
            ((frame_i * 10) > BLOCK_LENGTH) ? BLOCK_LENGTH : ((frame_i * 10) + 1),
            FORMAT_RGB565_TO_PACKET(RGB_BLOCK_COLOR(block_2_4)), RADIUS_FOR_CORNERS,
            ((frame_i * 10) > BLOCK_LENGTH) ? F_EMPTY_BLOCK_COLOR : F_GRID_COLOR);

        // Don't delay when empty space is fully filled.
        if (frame_i < 5) {
            // Minimum speed visual for human eye while also being fast and not laggy.
            MXC_Delay(MXC_DELAY_MSEC(20));
        }
    }

    // Calculate the starting coordinate to write the digit at the center of the tile.
    int x = (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_WIDTH(block_2_4) / 2) + GRID_OFFSET_X +
            GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_HEIGHT(block_2_4) / 2) + GRID_OFFSET_Y +
            GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    // Draw a 2 or 4 digit.
    // Blocks 2 and 4 are lighter color, so white text won't fit. Use black text instead (Inverted).
    if (block_2_4 == 2) {
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapInvertedMask(x, y, BLOCK_2_DIGIT_PX_WIDTH, BLOCK_2_DIGIT_PX_HEIGHT,
                                       block_2, RGB565_BLACK, RGB565_BLOCK_2);
    } else {
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapInvertedMask(x, y, BLOCK_4_DIGIT_PX_WIDTH, BLOCK_4_DIGIT_PX_HEIGHT,
                                       block_4, RGB565_BLACK, RGB565_BLOCK_4);
    }
}

// Mainly used for sliding blocks to a new location.
inline void Graphics_AddBlock(int row, int col, int value)
{
    int x = BLOCK_X_POSITION(col);
    int y = BLOCK_Y_POSITION(row);

    // Math is to center the digits printed to the newly created block.
    int dx = x + (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_WIDTH(value) / 2);
    int dy = y + (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_HEIGHT(value) / 2);

    switch (value) {
    case 2:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_2), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapInvertedMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value),
                                       BLOCK_DIGIT_PX_HEIGHT(value), block_2, RGB565_BLACK,
                                       RGB565_BLOCK_2);
        break;

    case 4:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_4), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapInvertedMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value),
                                       BLOCK_DIGIT_PX_HEIGHT(value), block_4, RGB565_BLACK,
                                       RGB565_BLOCK_4);
        break;

    case 8:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_8), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_8, RGB565_BLACK, RGB565_BLOCK_8);
        break;

    case 16:
        // Draw text.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_16), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_16, RGB565_BLACK, RGB565_BLOCK_16);
        break;

    case 32:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_32), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_32, RGB565_BLACK, RGB565_BLOCK_32);
        break;

    case 64:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_64), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_64, RGB565_BLACK, RGB565_BLOCK_64);
        break;

    case 128:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_128), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_128, RGB565_BLACK, RGB565_BLOCK_128);
        break;

    case 256:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_256), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_256, RGB565_BLACK, RGB565_BLOCK_256);
        break;

    case 512:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_512), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_512, RGB565_BLACK, RGB565_BLOCK_512);
        break;

    case 1024:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_1024), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_1024, RGB565_BLACK, RGB565_BLOCK_1024);
        break;

    case 2048:
        // Draw block.
        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_2048), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Draw digits.
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value),
                               block_2048, RGB565_BLACK, RGB565_BLOCK_2048);
        break;

    default:
        break;
    }
}

void Graphics_CombineSingleBlock(int row, int col, int new_value)
{
    int x = BLOCK_X_POSITION(col);
    int y = BLOCK_Y_POSITION(row);

    MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(new_value),
                            RADIUS_FOR_CORNERS, F_GRID_COLOR);

    // Animate the blow up.
    // 4 is the amount of pixels between each block. That's the extent that the block will temporarily expand.
    for (int i = 0; i < BLOCK_SPACING + 1; i++) {
        MXC_TFT_DrawRoundedRect(x - i, y - i, BLOCK_LENGTH + (2 * i), BLOCK_LENGTH + (2 * i),
                                FORMATTED_RGB_BLOCK_COLOR(new_value), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Delay for animation.
        MXC_Delay(MXC_DELAY_MSEC(6));
    }

    // Animate the shrink down.
    for (int i = 0; i < BLOCK_SPACING; i++) {
        // Redraw block to remove corners.
        MXC_TFT_DrawRoundedRect(x - BLOCK_SPACING + i, y - BLOCK_SPACING + i,
                                BLOCK_LENGTH + ((BLOCK_SPACING - i) * 2),
                                BLOCK_LENGTH + ((BLOCK_SPACING - i) * 2),
                                FORMATTED_RGB_BLOCK_COLOR(new_value), RADIUS_FOR_CORNERS,
                                F_GRID_COLOR);

        // Remove outer borders again.
        MXC_TFT_DrawHorizontalLine(x - BLOCK_SPACING + i, y - BLOCK_SPACING + i,
                                   BLOCK_LENGTH + ((BLOCK_SPACING - i) * 2),
                                   F_GRID_COLOR); // Top line.

        MXC_TFT_DrawVerticalLine(x - BLOCK_SPACING + i, y - BLOCK_SPACING + i,
                                 BLOCK_LENGTH + ((BLOCK_SPACING - i) * 2),
                                 F_GRID_COLOR); // Left line.

        MXC_TFT_DrawHorizontalLine(x - BLOCK_SPACING + 1 + i,
                                   BLOCK_LENGTH + y + BLOCK_SPACING - 1 - i,
                                   BLOCK_LENGTH + ((BLOCK_SPACING - 1 - i) * 2),
                                   F_GRID_COLOR); // Bottom line.

        MXC_TFT_DrawVerticalLine(BLOCK_LENGTH + x + BLOCK_SPACING - 1 - i, y - BLOCK_SPACING + i,
                                 BLOCK_LENGTH - 1 + ((BLOCK_SPACING - i) * 2),
                                 F_GRID_COLOR); // Right line.

        // Delay for animation.
        MXC_Delay(MXC_DELAY_MSEC(6));
    }

    Graphics_AddBlock(row, col, new_value);
}

// TODO(SW): Definitely could be optimized and more readable. Not enough time and wrote this late at night.
//  Helper function for "_CombineBlocks function which draws the outer borders of the block with width matching the 'radius'
//  @param  x               X coordinate of the block.
//  @param  y               Y coordinate of the block.
//  @param  frame_i         Current frame of the animation.
//  @param  current_length  Length of block at current frame.
//  @param  f_color         Formatted color code of outer border.
//  @param  radius          Radius ofrounded cornewrs and width of block.
static void graphics_helper_CombineBlocks(uint32_t x, uint32_t y, int frame_i,
                                          uint32_t current_length, uint32_t f_color,
                                          uint32_t radius)
{
    // Draw top horizontal lines.
    for (int p_y = 0; p_y < radius; p_y++) {
        // Find the starting x position using pythagorean theorem.
        int dx = 0;
        int dy = radius - p_y;

        while ((dx * dx) < ((radius * radius) - (dy * dy))) {
            dx++;
        }

        // No negative horizontal distance.
        if (dx > 0) {
            dx--;
        }

        MXC_TFT_DrawHorizontalLine(x - frame_i + radius - dx, y - frame_i + p_y,
                                   current_length - (2 * (radius - dx)), f_color);
    }

    // Draw bottom horizontal lines.
    for (int p_y = (current_length - radius); p_y < current_length; p_y++) {
        // Find the starting x position using pythagorean theorem.
        int dx = 0;
        int dy = p_y - (current_length - radius - 1);

        while ((dx * dx) < ((radius * radius) - (dy * dy))) {
            dx++;
        }

        // No negative horizontal distance.
        if (dx > 0) {
            dx--;
        }

        MXC_TFT_DrawHorizontalLine(x - frame_i + radius - dx, y - frame_i + p_y,
                                   current_length - (2 * (radius - dx)), f_color);
    }

    // Draw left vertical lines.
    for (int p_x = 0; p_x < radius; p_x++) {
        // Find the starting x position using pythagorean theorem.
        int dx = radius - p_x;
        int dy = 0;

        while ((dy * dy) < ((radius * radius) - (dx * dx))) {
            dy++;
        }

        // No negative horizontal distance.
        if (dy > 0) {
            dy--;
        }

        MXC_TFT_DrawVerticalLine(x - frame_i + p_x, y - frame_i + radius - dy,
                                 current_length - (2 * (radius - dy)), f_color);
    }

    // Draw right vertical lines.
    for (int p_x = (current_length - radius); p_x < current_length; p_x++) {
        // Find the starting x position using pythagorean theorem.
        int dx = p_x - (current_length - radius - 1);
        int dy = 0;

        while ((dy * dy) < ((radius * radius) - (dx * dx))) {
            dy++;
        }

        // No negative horizontal distance.
        if (dy > 0) {
            dy--;
        }

        MXC_TFT_DrawVerticalLine(x - frame_i + p_x, y - frame_i + radius - dy,
                                 current_length - (2 * (radius - dy)), f_color);
    }
}

//  Helper function for "_CombineBlocks function which removes the corners of the blocks outside of 'radius' boundary.
//  @param  x               X coordinate of the block.
//  @param  y               Y coordinate of the block.
//  @param  frame_i         Current frame of the animation.
//  @param  current_length  Length of block at current frame.
//  @param  f_color         Formatted color code of outer border.
//  @param  radius          Radius ofrounded cornewrs and width of block.
static void graphics_helper_eraseCorners(uint32_t x, uint32_t y, int frame_i,
                                         uint32_t current_length, uint32_t f_color, uint32_t radius)
{
    for (int p_y = 0; p_y < BLOCK_LENGTH; p_y++) {
        for (int p_x = 0; p_x < BLOCK_LENGTH; p_x++) {
            // Calculate distance from the corner.
            int dx = (p_x < radius)                   ? radius - p_x :
                     (p_x >= current_length - radius) ? p_x - (current_length - radius - 1) :
                                                        0;
            int dy = (p_y < radius)                   ? radius - p_y :
                     (p_y >= current_length - radius) ? p_y - (current_length - radius - 1) :
                                                        0;

            // Use pythagorean theorem to map coordinates, square not necessary when comparing with r.
            if (((dx * dx) + (dy * dy)) > (radius * radius)) {
                MXC_TFT_WritePixel(x + p_x, y + p_y, 1, 1, f_color);
            }

            // Skip x points that aren't within corners.
            if (p_x >= radius && p_x < (current_length - radius)) {
                p_x = current_length - radius - 1;
            }
        }

        // Skip x points that aren't within corners.
        if (p_y >= radius && p_y < (current_length - radius)) {
            p_y = current_length - radius - 1;
        }
    }
}

void Graphics_CombineBlocks(uint32_t grid[4][4], block_state_t grid_state[4][4])
{
    // Animate the blow up.
    // 4 is the amount of pixels between each block. That's the extent that the block will temporarily expand.
    for (int frame_i = 0; frame_i < BLOCK_SPACING + 1; frame_i++) {
        int combine_count = 0;

        // Go through each block that is going to be combined and draw a single blow up frame.
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                if (grid_state[row][col] == COMBINE) {
                    // Only draw out edges when expanding to save time.
                    combine_count++;

                    uint32_t x = BLOCK_X_POSITION(col);
                    uint32_t y = BLOCK_Y_POSITION(row);

                    if (frame_i == 0) {
                        // Draw entire block for first frame.
                        MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH,
                                                FORMATTED_RGB_BLOCK_COLOR(grid[row][col]),
                                                RADIUS_FOR_CORNERS, F_GRID_COLOR);
                    } else {
                        graphics_helper_CombineBlocks(x, y, frame_i, BLOCK_LENGTH + (2 * frame_i),
                                                      FORMATTED_RGB_BLOCK_COLOR(grid[row][col]),
                                                      RADIUS_FOR_CORNERS);
                    }
                }
            }
        }

        // If no blocks combined, don't waste computation time and exit.
        if (combine_count == 0) {
            return;
        }

        MXC_Delay(MXC_DELAY_MSEC(COMBINE_BLOCKS_ANIMATE_MAX_DELAY_MS));
    }

    // Animate the shrink down.
    for (int frame_i = BLOCK_SPACING - 1; frame_i >= 0; frame_i--) {
        int combine_count = 0;

        // Go through each block that is going to be combined and shrink the block
        //  one pixel at a time.
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                if (grid_state[row][col] == COMBINE) {
                    combine_count++;

                    uint32_t x = BLOCK_X_POSITION(col);
                    uint32_t y = BLOCK_Y_POSITION(row);

                    // Erase outer edges.
                    MXC_TFT_DrawHorizontalLine(x - (frame_i + 1), y - (frame_i + 1),
                                               BLOCK_LENGTH + (2 * (frame_i + 1)), F_GRID_COLOR);
                    MXC_TFT_DrawVerticalLine(x - (frame_i + 1), y - (frame_i + 1),
                                             BLOCK_LENGTH + (2 * (frame_i + 1)), F_GRID_COLOR);

                    MXC_TFT_DrawHorizontalLine(x - (frame_i + 1),
                                               y + BLOCK_LENGTH + (2 * (frame_i + 1)) -
                                                   (frame_i + 1) - 1,
                                               BLOCK_LENGTH + (2 * (frame_i + 1)), F_GRID_COLOR);
                    MXC_TFT_DrawVerticalLine(
                        x + BLOCK_LENGTH + (2 * (frame_i + 1)) - (frame_i + 1) - 1,
                        y - (frame_i + 1), BLOCK_LENGTH + (2 * (frame_i + 1)), F_GRID_COLOR);

                    // Erase corners.
                    graphics_helper_eraseCorners(x - frame_i, y - frame_i, frame_i,
                                                 BLOCK_LENGTH + (2 * (frame_i)), F_GRID_COLOR,
                                                 RADIUS_FOR_CORNERS);

                    // Draw digit in last frame.
                    if ((frame_i) == 0) {
                        // graphics_helper_CombineBlocks(x, y, 0, BLOCK_LENGTH, FORMATTED_RGB_BLOCK_COLOR(grid[row][col]), RADIUS_FOR_CORNERS);

                        // Blocks 2 and 4 are the only ones with inverted digit colors because of their block color.
                        //  However, Block 2 will nevber be a combined block.
                        if (grid[row][col] == 4) {
                            MXC_TFT_DrawBitmapInvertedMask(
                                DIGIT_CENTER_X_POSITION(x, grid[row][col]),
                                DIGIT_CENTER_Y_POSITION(y, grid[row][col]),
                                BLOCK_DIGIT_PX_WIDTH(grid[row][col]),
                                BLOCK_DIGIT_PX_HEIGHT(grid[row][col]),
                                BLOCK_DIGIT_PTR(grid[row][col]), RGB565_BLACK,
                                RGB_BLOCK_COLOR(grid[row][col]));
                        } else {
                            MXC_TFT_DrawBitmapMask(DIGIT_CENTER_X_POSITION(x, grid[row][col]),
                                                   DIGIT_CENTER_Y_POSITION(y, grid[row][col]),
                                                   BLOCK_DIGIT_PX_WIDTH(grid[row][col]),
                                                   BLOCK_DIGIT_PX_HEIGHT(grid[row][col]),
                                                   BLOCK_DIGIT_PTR(grid[row][col]), RGB565_BLACK,
                                                   RGB_BLOCK_COLOR(grid[row][col]));
                        }
                    }
                }
            }
        }

        // Add delay so the shrink down is visual to human eye,
        //  except for the last frame.
        if (frame_i != 0) {
            MXC_Delay(MXC_DELAY_MSEC(COMBINE_BLOCKS_ANIMATE_MAX_DELAY_MS));
        }
    }
}

void Graphics_EraseBlocks(block_state_t grid_state[4][4], graphics_slide_direction_t direction)
{
    switch (direction) {
    case GRAPHICS_SLIDE_DIR_UP:
        // Start from bottom row to top row (column order doesn't matter).
        // This for-loop keeps track of the localized y position of where the horizontal line is drawn.
        //  A horizontal line of 1-pixel wide, and of color of empty block, is drawn to represent
        //  the block being erased, and this is done through the entire block.
        for (int p_y = (BLOCK_LENGTH - 1); p_y >= 0; p_y--) {
            // These two for-loops iterate through each block in the
            //  grid (bottom row to top row, columns direction - don't care) and
            //  erases a horizontal line (1-pixel wide)
            for (int row = 3; row >= 0; row--) {
                for (int col = 0; col < 4; col++) {
                    if (grid_state[row][col] == ERASE) {
                        int x = BLOCK_X_POSITION(col);
                        int y = BLOCK_Y_POSITION(row);

                        // Account for rounded corners.
                        if ((p_y < RADIUS_FOR_CORNERS) ||
                            (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting x position using pythagorean theorem.
                            int dx = 0;
                            int dy = (p_y < RADIUS_FOR_CORNERS) ?
                                         RADIUS_FOR_CORNERS - p_y :
                                     (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ?
                                         p_y - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) :
                                         0;

                            while ((dx * dx) <
                                   ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dy * dy))) {
                                dx++;
                            }

                            // No negative horizontal distance.
                            if (dx > 0) {
                                dx--;
                            }

                            MXC_TFT_DrawHorizontalLine(x + RADIUS_FOR_CORNERS - dx, y + p_y,
                                                       BLOCK_LENGTH -
                                                           (2 * (RADIUS_FOR_CORNERS - dx)),
                                                       F_EMPTY_BLOCK_COLOR);

                        } else {
                            MXC_TFT_DrawHorizontalLine(x, y + p_y, BLOCK_LENGTH,
                                                       F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
        }
        break;

    case GRAPHICS_SLIDE_DIR_DOWN:
        // Start from top row to bottom row (column order doesn't matter).
        // This for-loop keeps track of the localized y position of where the horizontal line is drawn.
        //  A horizontal line of 1-pixel wide, and of color of empty block, is drawn to represent
        //  the block being erased, and this is done through the entire block.
        for (int p_y = 0; p_y < BLOCK_LENGTH; p_y++) {
            // These two for-loops iterate through each block in the
            //  grid (top row to bottom row, columns direction - don't care) and
            //  erases a horizontal line (1-pixel wide)
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) {
                    if (grid_state[row][col] == ERASE) {
                        int x = BLOCK_X_POSITION(col);
                        int y = BLOCK_Y_POSITION(row);

                        // Account for rounded corners.
                        if ((p_y < RADIUS_FOR_CORNERS) ||
                            (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting x position using pythagorean theorem.
                            int dx = 0;
                            int dy = (p_y < RADIUS_FOR_CORNERS) ?
                                         RADIUS_FOR_CORNERS - p_y :
                                     (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ?
                                         p_y - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) :
                                         0;

                            while ((dx * dx) <
                                   ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dy * dy))) {
                                dx++;
                            }

                            // No negative horizontal distance.
                            if (dx > 0) {
                                dx--;
                            }

                            MXC_TFT_DrawHorizontalLine(x + RADIUS_FOR_CORNERS - dx, y + p_y,
                                                       BLOCK_LENGTH -
                                                           (2 * (RADIUS_FOR_CORNERS - dx)),
                                                       F_EMPTY_BLOCK_COLOR);

                        } else {
                            MXC_TFT_DrawHorizontalLine(x, y + p_y, BLOCK_LENGTH,
                                                       F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
        }
        break;

    case GRAPHICS_SLIDE_DIR_LEFT:
        // Start from right column to left column (row order doesn't matter).
        // This for-loop keeps track of the localized x position of where the vertical line is drawn.
        //  A vertical line of 1-pixel wide, and of color of empty block, is drawn to represent
        //  the block being erased, and this is done through the entire block.
        for (int p_x = (BLOCK_LENGTH - 1); p_x >= 0; p_x--) {
            // These two for-loops iterate through each block in the
            //  grid (right column to left column, rows direction - don't care) and
            //  erases a vertical line (1-pixel wide)
            for (int col = 3; col >= 0; col--) {
                for (int row = 0; row < 4; row++) {
                    if (grid_state[row][col] == ERASE) {
                        int x = BLOCK_X_POSITION(col);
                        int y = BLOCK_Y_POSITION(row);

                        // Account for rounded corners.
                        if ((p_x < RADIUS_FOR_CORNERS) ||
                            (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting y position using pythagorean theorem.
                            int dx = (p_x < RADIUS_FOR_CORNERS) ?
                                         RADIUS_FOR_CORNERS - p_x :
                                     (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ?
                                         p_x - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) :
                                         0;
                            int dy = 0;

                            while ((dy * dy) <
                                   ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dx * dx))) {
                                dy++;
                            }

                            // No negative vertical distance.
                            if (dy > 0) {
                                dy--;
                            }

                            MXC_TFT_DrawVerticalLine(x + p_x, y + RADIUS_FOR_CORNERS - dy,
                                                     BLOCK_LENGTH - (2 * (RADIUS_FOR_CORNERS - dy)),
                                                     F_EMPTY_BLOCK_COLOR);

                        } else {
                            MXC_TFT_DrawVerticalLine(x + p_x, y, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
        }
        break;

    case GRAPHICS_SLIDE_DIR_RIGHT:
        // Start from left column to right column (row order doesn't matter).
        // This for-loop keeps track of the localized x position of where the vertical line is drawn.
        //  A vertical line of 1-pixel wide, and of color of empty block, is drawn to represent
        //  the block being erased, and this is done through the entire block.
        for (int p_x = 0; p_x < BLOCK_LENGTH; p_x++) {
            // These two for-loops iterate through each block in the
            //  grid (right column to left column, rows direction - don't care) and
            //  erases a vertical line (1-pixel wide)
            for (int col = 0; col < 4; col++) {
                for (int row = 0; row < 4; row++) {
                    if (grid_state[row][col] == ERASE) {
                        int x = BLOCK_X_POSITION(col);
                        int y = BLOCK_Y_POSITION(row);

                        // Account for rounded corners.
                        if ((p_x < RADIUS_FOR_CORNERS) ||
                            (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting y position using pythagorean theorem.
                            int dx = (p_x < RADIUS_FOR_CORNERS) ?
                                         RADIUS_FOR_CORNERS - p_x :
                                     (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ?
                                         p_x - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) :
                                         0;
                            int dy = 0;

                            while ((dy * dy) <
                                   ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dx * dx))) {
                                dy++;
                            }

                            // No negative vertical distance.
                            if (dy > 0) {
                                dy--;
                            }

                            MXC_TFT_DrawVerticalLine(x + p_x, y + RADIUS_FOR_CORNERS - dy,
                                                     BLOCK_LENGTH - (2 * (RADIUS_FOR_CORNERS - dy)),
                                                     F_EMPTY_BLOCK_COLOR);

                        } else {
                            MXC_TFT_DrawVerticalLine(x + p_x, y, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
        }

        break;

    default:
        return;
    }
}

void Graphics_EraseSingleBlock(int row, int col)
{
    // Forgive me for all the math who ever tries to read through the code.
    // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
    int x = GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR,
                            RADIUS_FOR_CORNERS, F_GRID_COLOR);
}

void Graphics_SetTime(uint32_t total_seconds)
{
    // Convert total time to minutes:seconds (mm:ss)
    int digit0, digit1, digit2, digit3;

    // The max possible seconds is 5999, which yields 99m:59s.
    if (total_seconds >= 5999) {
        digit0 = 9;
        digit1 = 5;
        digit2 = 9;
        digit3 = 9;
    } else {
        digit0 = (total_seconds % 10);
        digit1 = ((total_seconds - digit0) % 60) / 10;

        digit2 = ((total_seconds - (digit1 * 10) - digit0) / 60) % 10;
        digit3 = ((total_seconds - (digit1 * 10) - digit0) / 60) / 10;
    }

    // Update timer on display.
    MXC_TFT_DrawRect(TIME_DIGIT0_OFFSET_X(TIME_DIGIT_WIDTH), TIME_OFFSET_Y, TIME_DIGIT_WIDTH,
                     GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
    MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT0_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit0)),
                                   TIME_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit0),
                                   GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit0),
                                   RGB565_BLACK, RGB565_WHITE);

    // Don't waste time on re-writing the same digits.
    // Changes every 10 seconds.
    if (prev_timer_digit1 != digit1) {
        MXC_TFT_DrawRect(TIME_DIGIT1_OFFSET_X(TIME_DIGIT_WIDTH), TIME_OFFSET_Y, TIME_DIGIT_WIDTH,
                         GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
        MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT1_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit1)),
                                       TIME_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit1),
                                       GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit1),
                                       RGB565_BLACK, RGB565_WHITE);
        prev_timer_digit1 = digit1;
    }

    // Changes every 60 seconds.
    if (prev_timer_digit2 != digit2) {
        MXC_TFT_DrawRect(TIME_DIGIT2_OFFSET_X(TIME_DIGIT_WIDTH), TIME_OFFSET_Y, TIME_DIGIT_WIDTH,
                         GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
        MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT2_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit2)),
                                       TIME_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit2),
                                       GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit2),
                                       RGB565_BLACK, RGB565_WHITE);
        prev_timer_digit2 = digit2;
    }

    // Changes every 600 seoncds.
    if (prev_timer_digit3 != digit3) {
        MXC_TFT_DrawRect(TIME_DIGIT3_OFFSET_X(TIME_DIGIT_WIDTH), TIME_OFFSET_Y, TIME_DIGIT_WIDTH,
                         GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
        MXC_TFT_DrawBitmapInvertedMask(TIME_DIGIT3_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit3)),
                                       TIME_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit3),
                                       GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit3),
                                       RGB565_BLACK, RGB565_WHITE);
        prev_timer_digit3 = digit3;
    }
}

void Graphics_UpdateMovesCount(uint32_t moves_count)
{
    // Convert total time to minutes:seconds (mm:ss)
    int digit0, digit1, digit2, digit3;

    // The max possible seconds is 5999, which yields 99m:59s.
    if (moves_count >= 9999) {
        digit0 = 9;
        digit1 = 9;
        digit2 = 9;
        digit3 = 9;
    } else {
        digit0 = (moves_count % 10);
        digit1 = ((moves_count / 10) % 10);
        digit2 = ((moves_count / 100) % 10);
        digit3 = ((moves_count / 1000) % 10);
    }

    // Update timer on display.
    MXC_TFT_DrawRect(MOVES_DIGIT0_OFFSET_X(MOVES_DIGIT_WIDTH), MOVES_DIGITS_OFFSET_Y,
                     MOVES_DIGIT_WIDTH, GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
    MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT0_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit0)),
                                   MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit0),
                                   GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit0),
                                   RGB565_BLACK, RGB565_WHITE);

    // Don't waste time on re-writing the same digits.
    // Changes every 10 seconds.
    if (prev_moves_digit1 != digit1) {
        MXC_TFT_DrawRect(MOVES_DIGIT1_OFFSET_X(MOVES_DIGIT_WIDTH), MOVES_DIGITS_OFFSET_Y,
                         MOVES_DIGIT_WIDTH, GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
        MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT1_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit1)),
                                       MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit1),
                                       GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit1),
                                       RGB565_BLACK, RGB565_WHITE);
        prev_moves_digit1 = digit1;
    }

    // Changes every 60 seconds.
    if (prev_moves_digit2 != digit2) {
        MXC_TFT_DrawRect(MOVES_DIGIT2_OFFSET_X(MOVES_DIGIT_WIDTH), MOVES_DIGITS_OFFSET_Y,
                         MOVES_DIGIT_WIDTH, GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
        MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT2_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit2)),
                                       MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit2),
                                       GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit2),
                                       RGB565_BLACK, RGB565_WHITE);
        prev_moves_digit2 = digit2;
    }

    // Changes every 600 seoncds.
    if (prev_moves_digit3 != digit3) {
        MXC_TFT_DrawRect(MOVES_DIGIT3_OFFSET_X(MOVES_DIGIT_WIDTH), MOVES_DIGITS_OFFSET_Y,
                         MOVES_DIGIT_WIDTH, GAME_TEXT_DIGITS_HEIGHT, F_BACKGROUND_COLOR);
        MXC_TFT_DrawBitmapInvertedMask(MOVES_DIGIT3_OFFSET_X(GAME_TEXT_DIGIT_WIDTH(digit3)),
                                       MOVES_DIGITS_OFFSET_Y, GAME_TEXT_DIGIT_WIDTH(digit3),
                                       GAME_TEXT_DIGITS_HEIGHT, GAME_TEXT_DIGIT_PTR(digit3),
                                       RGB565_BLACK, RGB565_WHITE);
        prev_moves_digit3 = digit3;
    }
}

void Graphics_DisplayGameOver(void)
{
    MXC_TFT_DrawBitmap(GAME_OVER_BOX_OFFSET_X, GAME_OVER_BOX_OFFSET_Y, GAME_OVER_BOX_WIDTH,
                       GAME_OVER_BOX_HEIGHT, game_over);
}

void Graphics_DisplayYouWin(void)
{
    MXC_TFT_DrawBitmap(YOU_WIN_BOX_OFFSET_X, YOU_WIN_BOX_OFFSET_Y, YOU_WIN_BOX_WIDTH,
                       YOU_WIN_BOX_HEIGHT, you_win);
}
