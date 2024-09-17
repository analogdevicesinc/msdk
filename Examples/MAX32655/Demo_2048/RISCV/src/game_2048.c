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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mxc_device.h"
#include "trng.h"

#include "game_2048.h"

/* **** Definitions **** */

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif

#define COLUMN_SUM(col)                                                          \
    ((MAIN_2048_GRID[0][col] + MAIN_2048_GRID[1][col] + MAIN_2048_GRID[2][col] + \
      MAIN_2048_GRID[3][col]))
#define ROW_SUM(row)                                                             \
    ((MAIN_2048_GRID[row][0] + MAIN_2048_GRID[row][1] + MAIN_2048_GRID[row][2] + \
      MAIN_2048_GRID[row][3]))

/* **** Globals **** */

// Main 4x4 2048 grid that is used to save current state of game.
//  Did not choose to create defines for the size of the grid because
//  2048 is played on 4x4 grid.
// When using the grid as a single dimensional array, these are the
//  the indices assigned to each of the 16 squares.
//   0 | 1 | 2 | 3
//  ---|---|---|---
//   4 | 5 | 6 | 7
//  ---|---|---|---
//   8 | 9 | a | b
//  ---|---|---|---
//   c | d | e | f
static uint32_t MAIN_2048_GRID[4][4];
static block_state_t PREV_2048_GRID_STATE[4][4];

static game_state_t GAME_STATE;

static uint16_t AVAILABLE_EMPTY_BLOCKS_NUM;

/* **** Functions **** */

/**
 *  Must have TRNG initialized first before calling this function.
 * 
 *  @param  is_init                         True/False if a new block is added during
 *                                          initialization when grid is first created.
 *  @param  new_block_1D_idx_location       Pointer to hold the index (0-15) of the new
 *                                          block location.
 * 
 *  @return If true (non-zero value), new block added. If false (zero),
 *          game over.
 */
static bool add_new_block(bool is_init, uint8_t *new_block_1D_idx_location)
{
    uint8_t block_2_or_4;
    uint32_t open_space_idx;
    uint32_t available_open_spaces = 0;
    uint8_t open_space_count = 0;

    // If at the start of the new program, start with the 2 block.
    if (is_init == true) {
        block_2_or_4 = 2;
    } else {
        // Select whether a 2 or 4 block will be placed.
        //  Add more weight to 2.
        if (MXC_TRNG_RandomInt() % 3) { // 1 or 2
            block_2_or_4 = 2;
        } else { // 0
            block_2_or_4 = 4;
        }
    }

    // Find available spaces in the grid by representing the grid (2-D array)
    //  as a 1-D array.
    //      Locations of main           Locations of main
    //      grid represented            grid represented as
    //      as indices for              coordinates (row,col)
    //      1-D array:                  for 2-D array:
    //       0 | 1 | 2 | 3              (0,0) | (0,1) | (0,2) | (0,3)
    //      ---|---|---|---             ------|-------|-------|------
    //       4 | 5 | 6 | 7              (1,0) | (1,1) | (1,2) | (1,3)
    //      ---|---|---|---   ======>   ------|-------|-------|------
    //       8 | 9 | a | b              (2,0) | (2,1) | (2,2) | (2,3)
    //      ---|---|---|---             ------|-------|-------|------
    //       c | d | e | f              (3,0) | (3,1) | (3,2) | (3,3)
    for (int i = 0; i < 16; i++) {
        if (MAIN_2048_GRID[i / 4][i % 4] == 0) {
            available_open_spaces += 1;
        }
    }

    // This should be the only place that this variable is written to.
    AVAILABLE_EMPTY_BLOCKS_NUM = available_open_spaces;

    // No available space, game over.
    if (available_open_spaces == 0) {
        return false;
    }

    open_space_idx = (MXC_TRNG_RandomInt() % available_open_spaces);

    // Fill the "n-th" available open space where n is variable "open_space_idx".
    // We have the 1-D array index and need to convert to 2-D array coordinate location.
    int idx = 0;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if (MAIN_2048_GRID[row][col] == 0) {
                if (open_space_count == open_space_idx) {
                    // Found "n-th" available open space, update grid.
                    MAIN_2048_GRID[row][col] = block_2_or_4;
                    *new_block_1D_idx_location = idx;

                    // Decrement as a new block was just filled into the function.
                    AVAILABLE_EMPTY_BLOCKS_NUM--;
                    return true;
                }

                open_space_count++;
            }
            idx++;
        }
    }

    // Added for build warning.
    //  Shouldn't reach here as there's a check earlier in the function that checks if there
    //  are any available spaces to save time from iterating through the for loop.
    return false;
}

int Game_2048_Init(uint8_t *new_block_location_idx)
{
    int error;

    // Clear the grid.
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            MAIN_2048_GRID[i][j] = 0;
        }
    }

    // Initialize TRNG that is used to randomly select a 2 or 4 block in the grid.
    error = MXC_TRNG_Init();
    if (error != E_NO_ERROR) {
        return error;
    }

    // Should never reach here since we cleared the grid earlier in the function.
    if (add_new_block(true, new_block_location_idx) == false) {
        return E_BAD_STATE;
    }

    GAME_STATE = IN_PROGRESS;

    return E_NO_ERROR;
}

inline static bool row_logic_left(void)
{
    bool blocks_moved = false;

    for (int row = 0; row < 4; row++) {
        // Don't waste processing time if row is empty by checking if sum of all blocks in row is 0.
        if (ROW_SUM(row) == 0) {
            continue;
        }

        uint32_t prev_row[4] = { 0 };
        uint32_t temp_row[4] = { 0 };
        int temp_col_num = 0; // Also tracks how many valid blocks there are in a row.

        // Get all valid blocks in column to help with same-value pair logic.
        for (int col = 0; col < 4; col++) { // Start at left of main row.
            prev_row[col] = MAIN_2048_GRID[row][col];

            if (MAIN_2048_GRID[row][col] != 0) {
                temp_row[temp_col_num] = MAIN_2048_GRID[row][col];

                temp_col_num += 1;

                // Clear valid block on main grid for temp_row to write the updated
                //  row into after logic is done.
                MAIN_2048_GRID[row][col] = 0;
            }
        }

        // If there's only one valid block, don't bother with the same-value pair logic.
        if (temp_col_num == 1) {
            MAIN_2048_GRID[row][0] = temp_row[0];

            // Check if the single block moved left.
            if (prev_row[0] != MAIN_2048_GRID[row][0]) {
                blocks_moved = true;

                // Represent the state of the original block location as deleted for
                //  display.
                for (int col = 3; col >= 0; col--) {
                    if (prev_row[col] != 0) {
                        PREV_2048_GRID_STATE[row][col] = ERASE;
                        break;
                    }
                }
            } else {
                PREV_2048_GRID_STATE[row][0] = UNMOVED;
            }

            continue;
        }

        // Main logic: Combine the same-value pairs and write updated row to main grid.
        int main_grid_col = 0; // index used to keep track of where to write next on main grid.

        // Start at top of temp column.
        for (int t_col = 0; t_col < 4; t_col++) {
            if (temp_row[t_col] == 0 || t_col >= temp_col_num) {
                // Reached end of valid blocks in row.
                break;
            }

            // Write to main grid.
            MAIN_2048_GRID[row][main_grid_col] = temp_row[t_col];

            // Prevent illegal memory reads past main array size when at last column.
            if (t_col < 3) {
                // If same-value pair detected, combine then re-update the same block in main grid.
                if (temp_row[t_col] == temp_row[t_col + 1]) {
                    // Combine then write to main grid.
                    MAIN_2048_GRID[row][main_grid_col] = (temp_row[t_col] * 2);

                    PREV_2048_GRID_STATE[row][main_grid_col] = COMBINE;

                    // Because a same-value pair was combined at index "col" and "col + 1",
                    //  increment the col so that the next iteration of the for loop
                    //  will start at "col + 2" in the temp_row.
                    t_col += 1;
                }
            }

            main_grid_col += 1;
        }

        // Check if any blocks moved starting from right to left, and keep track of what blocks were moved
        //  to delete on the display.
        for (int col = 3; col >= 0; col--) {
            if (prev_row[col] != MAIN_2048_GRID[row][col]) {
                blocks_moved = true;

                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = ERASE;
                }
            } else if ((prev_row[col] != 0) && (prev_row[col] == MAIN_2048_GRID[row][col])) {
                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = UNMOVED;
                }
            }
        }
    }

    return blocks_moved;
}

inline static bool row_logic_right(void)
{
    bool blocks_moved = false;

    for (int row = 0; row < 4; row++) {
        // Don't waste processing time if row is empty by checking if sum of all blocks in row is 0.
        if (ROW_SUM(row) == 0) {
            continue;
        }

        uint32_t prev_row[4] = { 0 };
        uint32_t temp_row[4] = { 0 };
        int temp_col_num = 0; // Also tracks how many valid blocks there are in a row.

        // Get all valid blocks to help with same-value pair logic.
        for (int col = 3; col >= 0; col--) { // Start at right of main row.
            prev_row[col] = MAIN_2048_GRID[row][col];

            if (MAIN_2048_GRID[row][col] != 0) {
                temp_row[temp_col_num] = MAIN_2048_GRID[row][col];

                temp_col_num += 1;

                // Clear valid block on main grid for temp_row to write the updated
                //  row into after logic is done.
                MAIN_2048_GRID[row][col] = 0;
            }
        }

        // If there's only one valid block, don't bother with the same-value pair logic.
        if (temp_col_num == 1) {
            // Don't forget, starting at end of main row, but temp_row is still in order (left to right).
            MAIN_2048_GRID[row][3] = temp_row[0];

            // Check if the single block moved right.
            if (prev_row[3] != MAIN_2048_GRID[row][3]) {
                blocks_moved = true;

                // Represent the state of the original block location as deleted.
                for (int col = 0; col < 4; col++) {
                    if (prev_row[col] != 0) {
                        PREV_2048_GRID_STATE[row][col] = ERASE;
                        break;
                    }
                }
            } else {
                PREV_2048_GRID_STATE[row][3] = UNMOVED;
            }

            continue;
        }

        // Main logic: Combine the same-value pairs and write updated row to main grid.
        int main_grid_col = 3; // index used to keep track of where to write next on main grid.

        // Start at top of temp column.
        for (int t_col = 0; t_col < 4; t_col++) {
            if (temp_row[t_col] == 0 || t_col >= temp_col_num) {
                // Reached end of valid blocks.
                break;
            }

            // Write to main grid.
            MAIN_2048_GRID[row][main_grid_col] = temp_row[t_col];

            // Prevent illegal memory reads past main array size when at last column.
            if (t_col < 3) {
                // If same-value pair detected, combine then re-update the same block in main grid.
                if (temp_row[t_col] == temp_row[t_col + 1]) {
                    // Combine then write to main grid.
                    MAIN_2048_GRID[row][main_grid_col] = (temp_row[t_col] * 2);

                    PREV_2048_GRID_STATE[row][main_grid_col] = COMBINE;

                    // Because a same-value pair was combined at index "col" and "col + 1",
                    //  increment the col so that the next iteration of the for loop
                    //  will start at "col + 2" in the temp_row.
                    t_col += 1;
                }
            }

            main_grid_col -= 1;
        }

        // Check if any blocks moved starting from left to right, and keep track of what blocks were moved
        //  to delete on the display.
        for (int col = 0; col < 4; col++) {
            // Don't forget direction, start at end.
            if (prev_row[col] != MAIN_2048_GRID[row][col]) {
                blocks_moved = true;

                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = ERASE;
                }
            } else if ((prev_row[col] != 0) && (prev_row[col] == MAIN_2048_GRID[row][col])) {
                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = UNMOVED;
                }
            }
        }
    }

    return blocks_moved;
}

inline static bool column_logic_up(void)
{
    bool blocks_moved = false;

    for (int col = 0; col < 4; col++) {
        // Don't waste processing time if column is empty by checking if sum of all blocks in column is 0.
        if (COLUMN_SUM(col) == 0) {
            continue;
        }

        uint32_t prev_col[4] = { 0 };
        uint32_t temp_column[4] = { 0 };
        int temp_row_num = 0; // Also tracks how many valid blocks there are.

        // Get all valid blocks to help with same-value pair logic.
        for (int row = 0; row < 4; row++) { // Start at top of main column.
            prev_col[row] = MAIN_2048_GRID[row][col];

            if (MAIN_2048_GRID[row][col] != 0) {
                temp_column[temp_row_num] = MAIN_2048_GRID[row][col];

                temp_row_num += 1;

                // Clear valid block on main grid for temp_column to write the updated
                //  column into after logic is done.
                MAIN_2048_GRID[row][col] = 0;
            }
        }

        // If there's only one valid block, don't bother with the same-value pair logic.
        if (temp_row_num == 1) {
            MAIN_2048_GRID[0][col] = temp_column[0];

            // Check if the single block moved up.
            if (prev_col[0] != MAIN_2048_GRID[0][col]) {
                blocks_moved = true;

                for (int row = 3; row >= 0; row--) {
                    if (prev_col[row] != 0) {
                        PREV_2048_GRID_STATE[row][col] = ERASE;
                        break;
                    }
                }

            } else {
                PREV_2048_GRID_STATE[0][col] = UNMOVED;
            }

            continue;
        }

        // Main logic: Combine the same-value pairs and write updated column to main grid.
        int main_grid_row = 0; // index used to keep track of where to write next on main grid.

        // Start at top of temp column.
        for (int t_row = 0; t_row < 4; t_row++) {
            if (temp_column[t_row] == 0 || t_row >= temp_row_num) {
                // Reached end of valid blocks.
                break;
            }

            // Write to main grid.
            MAIN_2048_GRID[main_grid_row][col] = temp_column[t_row];

            // Prevent illegal memory reads past main array size when at last row.
            if (t_row < 3) {
                // If same-value pair detected, combine then re-update grid.
                if (temp_column[t_row] == temp_column[t_row + 1]) {
                    // Combine then write to main grid.
                    MAIN_2048_GRID[main_grid_row][col] = (temp_column[t_row] * 2);

                    PREV_2048_GRID_STATE[main_grid_row][col] = COMBINE;

                    // Because a same-value pair was combined at index "row" and "row + 1",
                    //  increment the row so that the next iteration of the for loop
                    //  will start at "row + 2" in the temp_column.
                    t_row += 1;
                }
            }

            main_grid_row += 1;
        }

        // Check if any blocks moved starting from bottom to top, and keep track of what blocks were moved
        //  to delete on the display.
        for (int row = 3; row >= 0; row--) {
            if (prev_col[row] != MAIN_2048_GRID[row][col]) {
                blocks_moved = true;

                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = ERASE;
                }
            } else if ((prev_col[row] != 0) && (prev_col[row] == MAIN_2048_GRID[row][col])) {
                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = UNMOVED;
                }
            }
        }
    }

    return blocks_moved;
}

static bool column_logic_down(void)
{
    bool blocks_moved = false;

    for (int col = 0; col < 4; col++) {
        // Don't waste processing time if column is empty by checking if sum of all blocks in column is 0.
        if (COLUMN_SUM(col) == 0) {
            continue;
        }

        uint32_t prev_col[4] = { 0 };
        uint32_t temp_column[4] = { 0 };
        int temp_row_num = 0; // Also tracks how many valid blocks there are in column.

        // Get all valid blocks to help with same-value pair logic.
        for (int row = 3; row >= 0; row--) { // Start at bottom of main column.
            prev_col[row] = MAIN_2048_GRID[row][col];

            if (MAIN_2048_GRID[row][col] != 0) {
                temp_column[temp_row_num] = MAIN_2048_GRID[row][col];

                temp_row_num += 1;

                // Clear valid block on main grid for temp_column to write the updated
                //  column into after logic is done.
                MAIN_2048_GRID[row][col] = 0;
            }
        }

        // If there's only one valid block, don't bother with the same-value pair logic.
        if (temp_row_num == 1) {
            // Confusing but temp column goes in order no matter the direction (up/down).
            MAIN_2048_GRID[3][col] = temp_column[0];

            // Check if the single block moved down.
            if (prev_col[3] != MAIN_2048_GRID[3][col]) {
                blocks_moved = true;

                // Represent the state of the original block location as deleted.
                for (int row = 0; row < 4; row++) {
                    if (prev_col[row] != 0) {
                        PREV_2048_GRID_STATE[row][col] = ERASE;
                        break;
                    }
                }
            } else {
                PREV_2048_GRID_STATE[3][col] = UNMOVED;
            }

            continue;
        }

        // Main logic: Combine the same-value pairs and write updated column to main grid.
        int main_grid_row = 3; // index used to keep track of where to write next on main grid.

        // Start at top of temp column.
        for (int t_row = 0; t_row < 4; t_row++) {
            if (temp_column[t_row] == 0 || t_row >= temp_row_num) {
                // Reached end of valid blocks.
                break;
            }

            // Write to main grid.
            MAIN_2048_GRID[main_grid_row][col] = temp_column[t_row];

            // Prevent illegal memory reads (-1) index of temp column.
            if (t_row < 3) {
                // If same-value pair detected, combine then re-update grid.
                if (temp_column[t_row] == temp_column[t_row + 1]) {
                    // Combine then write to main grid.
                    MAIN_2048_GRID[main_grid_row][col] = (temp_column[t_row] * 2);

                    PREV_2048_GRID_STATE[main_grid_row][col] = COMBINE;

                    // Because a same-value pair was combined at index "row" and "row + 1",
                    //  increment the row so that the next iteration of the for loop
                    //  will start at "row + 2" in the temp_column.
                    // Don't forget, temp column goes in order (top to bottom) even though
                    //  we go backwards in main column due to down direction.
                    t_row += 1;
                }
            }

            main_grid_row -= 1;
        }

        // Check if any blocks moved starting from bottom to top, and keep track of what blocks were moved
        //  to delete on the display.
        for (int row = 0; row < 4; row++) {
            if (prev_col[row] != MAIN_2048_GRID[row][col]) {
                blocks_moved = true;

                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = ERASE;
                }
            } else if ((prev_col[row] != 0) && (prev_col[row] == MAIN_2048_GRID[row][col])) {
                if (PREV_2048_GRID_STATE[row][col] != COMBINE) {
                    PREV_2048_GRID_STATE[row][col] = UNMOVED;
                }
            }
        }
    }

    return blocks_moved;
}

game_state_t Game_2048_CheckState(void)
{
    // Check if there's a 2048 block.
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if (MAIN_2048_GRID[row][col] == 2048) {
                return WINNER;
            }
        }
    }

    // Check if the new block that was just added filled in the
    //  last available empty block.
    if (AVAILABLE_EMPTY_BLOCKS_NUM == 0) {
        // If so, check if there are any swipes that can still be made before calling game over.
        //  This can be done by checking if there are any same-value pairs next to each other.
        // Check if a row has same value pair.
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 3; c++) {
                if (MAIN_2048_GRID[r][c] == MAIN_2048_GRID[r][c + 1]) {
                    return IN_PROGRESS;
                }
            }
        }

        // Check if a column has a same-value pair.
        for (int c = 0; c < 4; c++) {
            for (int r = 0; r < 3; r++) {
                if (MAIN_2048_GRID[r][c] == MAIN_2048_GRID[r + 1][c]) {
                    return IN_PROGRESS;
                }
            }
        }

        // If it reaches here, then there were no same-value pairs and no more moves can be made
        return GAME_OVER;
    } else {
        return IN_PROGRESS;
    }
}

bool Game_2048_UpdateGrid(input_direction_t direction, uint8_t *new_block_1D_idx)
{
    bool blocks_moved;

    // Clear grid state.
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            PREV_2048_GRID_STATE[row][col] = 0;
        }
    }

    // Run main game logic.
    switch (direction) {
    case INPUT_UP:
        printf("UP\n");
        blocks_moved = column_logic_up();
        break;

    case INPUT_DOWN:
        printf("DOWN\n");
        blocks_moved = column_logic_down();
        break;

    case INPUT_LEFT:
        printf("LEFT\n");
        blocks_moved = row_logic_left();
        break;

    case INPUT_RIGHT:
        printf("RIGHT\n");
        blocks_moved = row_logic_right();
        break;

    // Should never reach here.
    default:
        return false;
    }

    // Once the main game logic is done, insert a new block.
    if (blocks_moved == true) {
        if (add_new_block(false, new_block_1D_idx) == true) {
            // Successfully added new block.
            return true;
        } else {
            // New block could not be added, nothing happened.
            return false;
        }
    } else {
        // Blocks didn't move.
        return false;
    }
}

void Game_2048_PrintGrid(void)
{
    // Imitate the grid is refreshing on terminal.
    PRINT("\n\n\n\n\n\n\n\n\n\n");

    for (int row = 0; row < 4; row++) {
        PRINT("        |        |        |        \n");

        for (int col = 0; col < 4; col++) {
            if (MAIN_2048_GRID[row][col] != 0) {
                PRINT("  %04d  ", MAIN_2048_GRID[row][col]);
            } else {
                PRINT("        ");
            }

            // Only print border 3 times.
            if (col < 3) {
                PRINT("|");
            }
        }

        PRINT("\n        |        |        |        \n");

        // Only print the row border 3 times.
        if (row < 3) {
            PRINT("-----------------------------------\n");
        }
    }
}

void Game_2048_GetGrid(uint32_t grid[4][4])
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            grid[row][col] = MAIN_2048_GRID[row][col];
        }
    }
}

void Game_2048_GetGridState(uint8_t grid_state[4][4])
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            grid_state[row][col] = PREV_2048_GRID_STATE[row][col];
        }
    }
}
