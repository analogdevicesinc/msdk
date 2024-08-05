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
#include "RMS.h"
#include "CM4_Utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void RMS_Initialize(rms_instance_t *instance)
{
    memset(instance, 0, sizeof(rms_instance_t));
    instance->sumCountRemaining = SQ_AVG_COUNT;
}

bool RMS_ProcessSamples(rms_instance_t *instance, uint32_t *sampleBuffer, uint32_t sampleCount)
{
    int64_t workingSum = instance->sumOfSquares;
    uint32_t sumCount = instance->sumCountRemaining;
    int64_wrapper_t *samplePtr = (int64_wrapper_t *)sampleBuffer;
    int64_wrapper_t sample;

    /* Pseudo-Unroll the loop by working on 64-bits at a time. Reducing the
     * number of decrements and checks
     */
    uint32_t remaining = sampleCount / 2;

    /* By running this as a while loop, doing a != 0 check, the compiler can use
     * the flags set by the decrement operation directly, and avoid having to do
     * an additional CMP instruction if we were to do a < type loop terminator
     * like a for loop
     */
    while (remaining != 0) {
        /* Try working in 64-bit chunks to make use of instructions like LDRD
           and limit the number of loop iterations, decrements, branches needed
           for processing the block
         */
        sample = *samplePtr++;
        sample.words.i32hi -= RMS_DC_OFFSET;
        sample.words.i32lo -= RMS_DC_OFFSET;

        //There is probably a way to trick the compiler into generating this
        //instruction, but its not as obvious since we're only caring about the
        //lower word of a 32-bit.  Easiest just to do the ASM ourselves.
        //By using this instruction, we can discard the status/unused portion of
        //the 32-bit ADC data in the instruction itself without needing masking
        //or other calls
        workingSum = __SMLALBB(sample.words.i32hi, sample.words.i32hi, workingSum);
        workingSum = __SMLALBB(sample.words.i32lo, sample.words.i32lo, workingSum);

        remaining--;
    }

    /* Do this with a decrement so we can compare against 0 */
    sumCount -= sampleCount;
    if (sumCount == 0) //Completed this block
    {
        uint32_t value = (uint32_t)(workingSum >> SQ_AVG_SHIFT);
        instance->sumCountRemaining = SQ_AVG_COUNT;
        instance->sumOfSquares = 0;
        instance->lastRMS = sqrt32(value);
        return true;
    } else { //More to go
        instance->sumOfSquares = workingSum;
        instance->sumCountRemaining = sumCount;
        return false;
    }
}
