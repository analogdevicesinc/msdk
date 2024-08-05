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
#ifndef EXAMPLES_MAX32672_ADC_DMA_RMS_MOVINGAVERAGE_H_
#define EXAMPLES_MAX32672_ADC_DMA_RMS_MOVINGAVERAGE_H_
#include <stdint.h>

/**
 * Structure for holding a moving average instance
 */
typedef struct {
    uint16_t *window; //Needs to be memory allocated by user
    uint32_t workingSum;
    uint16_t lastVal;
    uint8_t index;
    uint8_t count;
    uint8_t windowSize;
} moving_average_instance_t;

/**
 * Initializes an instance of a moving average filter.  Both the instance and
 * window should be pre-allocated memory by the user. The window buffer must be
 * at least windowSize big to prevent overflow
 *
 * @param inst - Instance pointer to initialize
 * @param window - Window buffer
 * @param windowSize - size of the window
*/
void MovingAverageInitialize(moving_average_instance_t *inst, uint16_t *window, uint8_t windowSize);

/**
 * Resets the state of the Moving Average Filter
 *
 * @param inst - Instance to reset
 */
void MovingAverageReset(moving_average_instance_t *inst);

/**
 * Performs the moving average filtering operation.
 *
 * @param inst - Instance to work on
 * @param sample - Sample to filter
 * @returns New filtered result
 */
uint16_t MovingAverageFilter(moving_average_instance_t *inst, uint16_t sample);

#endif // EXAMPLES_MAX32672_ADC_DMA_RMS_MOVINGAVERAGE_H_
