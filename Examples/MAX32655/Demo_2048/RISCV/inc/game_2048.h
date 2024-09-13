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

/**
 *  This enum is used to keep track of which direction the blocks will slide to
 *      for the main logic to handle. 
 */
typedef enum {
    INPUT_UP = 0,
    INPUT_DOWN = 1,
    INPUT_LEFT = 2,
    INPUT_RIGHT = 3,
} input_direction_t;

/* **** Function Prototypes **** */

int Game_2048_Init(void);

int Game_2048_UpdateGrid(input_direction_t direction);

void Game_2048_PrintGrid(void);

void Game_2048_GetGrid(uint32_t grid[4][4]);

void Game_2048_GetGridMailBox(uint8_t *grid_1D_size_64B);
