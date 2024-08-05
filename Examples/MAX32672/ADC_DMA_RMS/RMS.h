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
#ifndef EXAMPLES_MAX32672_ADC_DMA_RMS_RMS_H_
#define EXAMPLES_MAX32672_ADC_DMA_RMS_RMS_H_

#include <stdint.h>
#include <stdbool.h>

/* These define the number of samples used to generated the average of squares.
 * These should be set based on the overall system sample rate and desired
 * output rate.  Alternatively they could be configured at runtime, however that
 * will impact performance.
 */
#define SQ_AVG_SHIFT 15 //Right shift this many bits to divide by count
#define SQ_AVG_COUNT (1U << SQ_AVG_SHIFT)

/* This value will be subtracted from every sample to generate AC data */
#define RMS_DC_OFFSET (int32_t)(1 << 11) //Half of a 12-bit ADC

/**
 * Instance structure to track state. Helpful if multiple instances are needed
 */
typedef struct {
    uint64_t sumOfSquares;
    uint32_t sumCountRemaining;
    uint16_t lastRMS;
} rms_instance_t;

/**
 * Initializes the RMS instance.
 *
 * @param instance - Instance to initialize
 */
void RMS_Initialize(rms_instance_t *instance);

/**
 * Processes a block of samples.  This runs the sample block through the math,
 * if the sumCount is achieved, the lastRMS value of the instance is updated and
 * this returns True to indicate new data, otherwise returns false.
 *
 * IMPORTANT: This assumes that the block sizes coming in will always be a
 * divisor of/add up to SQ_AVG_COUNT. This doesn't account for variable size
 * sampleCounts or non-divisible counts.
 *
 * IMPORTANT: This also assumes the sampleCount is even (/2) to reduce loop
 *            loop iterations in the code
 *
 * NOTE: This implementation (taking in uint32_t buffers) is for the internal
 * ADC which stores the ADC data in the lower 16-bits, and the upper 16-bits is
 * status or 0. An alternative implementation (more efficient with math) could
 * be used for ADC data which is packed in 16-bit intervals
 *
 * @param instance - Instance to work with
 * @param sampleBuffer - ADC data
 * @param sampleCount - Size of the ADC data
 * @return True if a new output sample is available, false otherwise
 */
bool RMS_ProcessSamples(rms_instance_t *instance, uint32_t *sampleBuffer, uint32_t sampleCount);

#endif // EXAMPLES_MAX32672_ADC_DMA_RMS_RMS_H_
