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

#include "graphics.h"

#include "clear_sans_bold_num.h"

#include "led.h"


/* **** Definitions **** */

// #define ANIMATION_DELAY(block_num)      ()   


/* **** Globals **** */

// Keep copy of grid.
// static uint32_t MAIN_GRID[4][4] = {0};

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
    MXC_TFT_DrawFastRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, F_BACKGROUND_COLOR);

    // Draw grid.
    MXC_TFT_DrawRoundedRect(GRID_OFFSET_X + GRID_SPACING, GRID_OFFSET_Y + GRID_SPACING, GRID_LENGTH, GRID_LENGTH, F_GRID_COLOR, RADIUS_FOR_CORNERS, F_BACKGROUND_COLOR);
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
            MXC_TFT_DrawRoundedRect(GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row), GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col), BLOCK_LENGTH, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR, RADIUS_FOR_CORNERS, F_GRID_COLOR);
        }
    }

    return E_NO_ERROR;
}

void Graphics_AddNewBlock(int row, int col, int block_2_4)
{
    // 6 is chosen based on animation/display refresh rate speed to the naked eye.
    for (int i = 0; i < 6; i++) {
        // Forgive the unreadability.
        //  The math calculates where each block needs to be drawn as its animated to grow from small to big.

        // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
        int x = ((BLOCK_LENGTH / 2) - (i * 5)) + GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
        int y = ((BLOCK_LENGTH / 2) - (i * 5)) + GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

        MXC_TFT_DrawRoundedRect(x, y, ((i * 10) > BLOCK_LENGTH) ? BLOCK_LENGTH : ((i * 10) + 1), ((i * 10) > BLOCK_LENGTH) ? BLOCK_LENGTH : ((i * 10) + 1), FORMAT_RGB565_TO_PACKET(RGB_BLOCK_COLOR(block_2_4)), RADIUS_FOR_CORNERS, ((i * 10) > BLOCK_LENGTH) ? F_EMPTY_BLOCK_COLOR : F_GRID_COLOR);
        
        // Don't delay when empty space is fully filled.
        if (i < 5) {
            // Minimum speed visual for human eye while also being fast and not laggy.
            MXC_Delay(MXC_DELAY_MSEC(15));
        }
    }

    // Calculate the starting coordinate to write the digit at the center of the tile.
    int x = (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_WIDTH(block_2_4) / 2) + GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_HEIGHT(block_2_4) / 2) + GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    // Draw a 2 or 4 digit.
    // Blocks 2 and 4 are lighter color, so white text won't fit. Use black text instead (Inverted).
    if (block_2_4 == 2) {
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapInvertedMask(x, y, BLOCK_2_DIGIT_PX_WIDTH, BLOCK_2_DIGIT_PX_HEIGHT, block_2, RGB565_BLACK, RGB565_BLOCK_2);
    } else {
        // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
        MXC_TFT_DrawBitmapInvertedMask(x, y, BLOCK_4_DIGIT_PX_WIDTH, BLOCK_4_DIGIT_PX_HEIGHT, block_4, RGB565_BLACK, RGB565_BLOCK_4);
    }
}

// Mainly used for sliding blocks to a new location.
inline void Graphics_AddBlock(int row, int col, int value)
{
    // Forgive me for all the math who ever tries to read through the code.
    // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
    int x = GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    // Math is to center the digits printed to the newly created block.
    int dx = x + (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_WIDTH(value) / 2);
    int dy = y + (BLOCK_LENGTH / 2) - (BLOCK_DIGIT_PX_HEIGHT(value) / 2);

    switch (value) {
        case 2:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_2), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapInvertedMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_2, RGB565_BLACK, RGB565_BLOCK_2);
            break;

        case 4:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_4), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapInvertedMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_4, RGB565_BLACK, RGB565_BLOCK_4);
            break;

        case 8:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_8), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_8, RGB565_BLACK, RGB565_BLOCK_8);
            break;

        case 16:
            // Draw text.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_16), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_16, RGB565_BLACK, RGB565_BLOCK_16);
            break;

        case 32:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_32), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_32, RGB565_BLACK, RGB565_BLOCK_32);
            break;

        case 64:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_64), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_64, RGB565_BLACK, RGB565_BLOCK_64);
            break;

        case 128:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_128), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_128, RGB565_BLACK, RGB565_BLOCK_128);
            break;
        
        case 256:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_256), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_256, RGB565_BLACK, RGB565_BLOCK_256);
            break;
        
        case 512:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_512), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_512, RGB565_BLACK, RGB565_BLOCK_512);
            break;
        
        case 1024:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_1024), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_1024, RGB565_BLACK, RGB565_BLOCK_1024);
            break;
        
        case 2048:
            // Draw block.
            MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(RGB565_BLOCK_2048), RADIUS_FOR_CORNERS, F_GRID_COLOR);

            // Draw digits.
            // 0x0000 is RGB color BLACK (background of digit) which will be masked out to match color of block.
            MXC_TFT_DrawBitmapMask(dx, dy, BLOCK_DIGIT_PX_WIDTH(value), BLOCK_DIGIT_PX_HEIGHT(value), block_2048, RGB565_BLACK, RGB565_BLOCK_2048);
            break;

        default:
            break;
    }
}

void Graphics_CombineBlocks(int row, int col, int new_value)
{
    // Forgive me for all the math who ever tries to read through the code.
    // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
    int x = GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, FORMAT_RGB565_TO_PACKET(new_value), RADIUS_FOR_CORNERS, F_GRID_COLOR);

    MXC_Delay(MXC_DELAY_MSEC(2));

    // Animate the blow up.
    // 4 is the amount of pixels between each block. That's the extent that the block will temporarily expand.   
    for (int i = 0; i < BLOCK_SPACING + 1; i++) {
        MXC_TFT_DrawRoundedRect(x - i, y - i, BLOCK_LENGTH + (2*i), BLOCK_LENGTH + (2*i), FORMATTED_RGB_BLOCK_COLOR(new_value), RADIUS_FOR_CORNERS, F_GRID_COLOR);

        // Delay for animation.
        MXC_Delay(MXC_DELAY_MSEC(6));
    }

    // Animate the shrink down.
    for (int i = 0; i < BLOCK_SPACING; i++) {
        // Redraw block to remove corners.
        MXC_TFT_DrawRoundedRect(x - BLOCK_SPACING + i, y - BLOCK_SPACING + i, BLOCK_LENGTH + ((BLOCK_SPACING - i)*2), BLOCK_LENGTH + ((BLOCK_SPACING - i)*2), FORMATTED_RGB_BLOCK_COLOR(new_value), RADIUS_FOR_CORNERS, F_GRID_COLOR);

        // Remove outer borders again.
        MXC_TFT_DrawHorizontalLine(x - BLOCK_SPACING + i, y - BLOCK_SPACING + i, BLOCK_LENGTH + ((BLOCK_SPACING - i)*2), F_GRID_COLOR); // Top line.

        MXC_TFT_DrawVerticalLine(x - BLOCK_SPACING + i, y - BLOCK_SPACING + i, BLOCK_LENGTH + ((BLOCK_SPACING - i)*2), F_GRID_COLOR); // Left line.

        MXC_TFT_DrawHorizontalLine(x - BLOCK_SPACING + 1 + i, BLOCK_LENGTH + y + BLOCK_SPACING - 1 - i, BLOCK_LENGTH + ((BLOCK_SPACING - 1 - i)*2), F_GRID_COLOR); // Bottom line.

        MXC_TFT_DrawVerticalLine(BLOCK_LENGTH + x + BLOCK_SPACING - 1 - i, y - BLOCK_SPACING + i, BLOCK_LENGTH - 1 + ((BLOCK_SPACING - i)*2), F_GRID_COLOR); // Right line.

        // Delay for animation.
        MXC_Delay(MXC_DELAY_MSEC(6));
    }

    Graphics_AddBlock(row, col, new_value);
}

void Graphics_EraseSingleBlockAnimated(int row, int col, graphics_slide_direction_t direction)
{
    // Forgive me for all the math who ever tries to read through the code.
    // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
    int x = GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    switch (direction) {
        case GRAPHICS_SLIDE_DIR_UP:
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    // Remove each column (x) of block left to right, pixel by pixel.
                    for (int p_y = (BLOCK_LENGTH - 1); p_y >= 0; p_y--) {
                        // Account for rounded corners.
                        if ((p_y < RADIUS_FOR_CORNERS) || (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting y position using pythagorean theorem and knowing the max y distance is the radius of the rounded corner.

                            // With y and the radius, get the integral to calculate the y position.
                            int dy = (p_y < RADIUS_FOR_CORNERS) ? RADIUS_FOR_CORNERS - p_y : (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ? p_y - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) : 0;
                            int dx = 0;

                            while ((dx * dx) < ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dy * dy))) {
                                dx++;
                            }

                            // No negative vertical distance.
                            if (dx > 0) {
                                dx--;
                            }

                            MXC_TFT_DrawVerticalLine(x + RADIUS_FOR_CORNERS - dx, y + p_y, BLOCK_LENGTH - (2 * (RADIUS_FOR_CORNERS - dx)), F_EMPTY_BLOCK_COLOR);
                        
                        } else {
                            MXC_TFT_DrawVerticalLine(x, y + p_y, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
            break;
        
        case GRAPHICS_SLIDE_DIR_DOWN:
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    // Remove each column (x) of block left to right, pixel by pixel.
                    for (int p_y = 0; p_y < BLOCK_LENGTH; p_y++) {
                        // Account for rounded corners.
                        if ((p_y < RADIUS_FOR_CORNERS) || (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting y position using pythagorean theorem and knowing the max y distance is the radius of the rounded corner.

                            // With y and the radius, get the integral to calculate the y position.
                            int dy = (p_y < RADIUS_FOR_CORNERS) ? RADIUS_FOR_CORNERS - p_y : (p_y >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ? p_y - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) : 0;
                            int dx = 0;

                            while ((dx * dx) < ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dy * dy))) {
                                dx++;
                            }

                            // No negative vertical distance.
                            if (dx > 0) {
                                dx--;
                            }

                            MXC_TFT_DrawVerticalLine(x + RADIUS_FOR_CORNERS - dx, y + p_y, BLOCK_LENGTH - (2 * (RADIUS_FOR_CORNERS - dx)), F_EMPTY_BLOCK_COLOR);
                        
                        } else {
                            MXC_TFT_DrawVerticalLine(x, y + p_y, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
            break;
        
        case GRAPHICS_SLIDE_DIR_LEFT:
            for (int c = 0; c < 4; c++) {
                for (int r = 0; r < 4; r++) {
                    // Remove each column (x) of block left to right, pixel by pixel.
                    for (int p_x = (BLOCK_LENGTH - 1); p_x >= 0; p_x--) {
                        // Account for rounded corners.
                        if ((p_x < RADIUS_FOR_CORNERS) || (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting y position using pythagorean theorem and knowing the max y distance is the radius of the rounded corner.

                            // With x and the radius, get the integral to calculate the y position.
                            int dx = (p_x < RADIUS_FOR_CORNERS) ? RADIUS_FOR_CORNERS - p_x : (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ? p_x - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) : 0;
                            int dy = 0;

                            while ((dy * dy) < ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dx * dx))) {
                                dy++;
                            }

                            // No negative vertical distance.
                            if (dy > 0) {
                                dy--;
                            }

                            MXC_TFT_DrawVerticalLine(x + p_x, y + RADIUS_FOR_CORNERS - dy, BLOCK_LENGTH - (2 * (RADIUS_FOR_CORNERS - dy)), F_EMPTY_BLOCK_COLOR);
                        
                        } else {
                            MXC_TFT_DrawVerticalLine(x + p_x, y, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
            break;
        
        case GRAPHICS_SLIDE_DIR_RIGHT:
            for (int c = 0; c < 4; c++) {
                for (int r = 0; r < 4; r++) {
                    // Remove each column (x) of block left to right, pixel by pixel.
                    for (int p_x = 0; p_x < BLOCK_LENGTH; p_x++) {
                        // Account for rounded corners.
                        if ((p_x < RADIUS_FOR_CORNERS) || (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS))) {
                            // Find the starting y position using pythagorean theorem and knowing the max y distance is the radius of the rounded corner.
                            int dx = (p_x < RADIUS_FOR_CORNERS) ? RADIUS_FOR_CORNERS - p_x : (p_x >= (BLOCK_LENGTH - RADIUS_FOR_CORNERS)) ? p_x - (BLOCK_LENGTH - RADIUS_FOR_CORNERS - 1) : 0;
                            int dy = 0;

                            while ((dy * dy) < ((RADIUS_FOR_CORNERS * RADIUS_FOR_CORNERS) - (dx * dx))) {
                                dy++;
                            }

                            // No negative vertical distance.
                            if (dy > 0) {
                                dy--;
                            }

                            MXC_TFT_DrawVerticalLine(x + p_x, y + RADIUS_FOR_CORNERS - dy, BLOCK_LENGTH - (2 * (RADIUS_FOR_CORNERS - dy)), F_EMPTY_BLOCK_COLOR);
                        
                        } else {
                            MXC_TFT_DrawVerticalLine(x + p_x, y, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR);
                        }
                    }
                }
            }
            break;
        
        default:
            return;
    }
}

void Graphics_EraseSingleBlock(int row, int col) {
    // Forgive me for all the math who ever tries to read through the code.
    // Focusing on top left corner of the top left block in the grid to help with visualizing the coordinates and calculations.
    int x = GRID_OFFSET_X + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * col);
    int y = GRID_OFFSET_Y + GRID_SPACING + BLOCK_SPACING + ((BLOCK_LENGTH + BLOCK_SPACING) * row);

    MXC_TFT_DrawRoundedRect(x, y, BLOCK_LENGTH, BLOCK_LENGTH, F_EMPTY_BLOCK_COLOR, RADIUS_FOR_CORNERS, F_GRID_COLOR);
}
