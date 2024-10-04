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
#include "MovingAverage.h"

void MovingAverageInitialize(moving_average_instance_t *inst, uint16_t *window, uint8_t windowSize)
{
    inst->window = window;
    inst->index = 0;
    inst->count = 0;
    inst->windowSize = windowSize;
    inst->workingSum = 0;
    inst->lastVal = 0;
}

void MovingAverageReset(moving_average_instance_t *inst)
{
    inst->count = 0;
    inst->index = 0;
    inst->workingSum = 0;
    inst->lastVal = 0;
}

uint16_t MovingAverageFilter(moving_average_instance_t *inst, uint16_t sample)
{
    if (inst->count == inst->windowSize) //Window has reached capacity
    {
        inst->workingSum -= inst->window[inst->index];
        inst->window[inst->index] = sample;
        inst->workingSum += sample;
    } else { //Window hasn't filled up yet
        inst->window[inst->index] = sample;
        inst->workingSum += sample;
        inst->count++;
    }

    /* Note: If windowSize/count becomes power of 2, both these steps can
       be simplified/optimized for execution time
     */
    inst->lastVal = inst->workingSum / inst->count;
    inst->index = (inst->index + 1) % inst->windowSize;
    return inst->lastVal;
}
