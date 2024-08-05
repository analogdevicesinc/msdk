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
#ifndef EXAMPLES_MAX32672_ADC_DMA_RMS_INTERNALADC_H_
#define EXAMPLES_MAX32672_ADC_DMA_RMS_INTERNALADC_H_

#include <stdint.h>
#include <stddef.h>

/* Sample rate of the ADC in SPS. This is a informative constant. The code needs
   to be changed to change the sample rate
 */
#define INTERNAL_ADC_SAMPLE_RATE 490196

/* How many samples per block of DMA conversions before notifying the user. This
   has a direct impact on IRQ frequency and RAM utilization
 */
#define INTERNAL_ADC_DMA_BLOCK_SIZE 1024

/* Prototype for callback functionality */
typedef void (*ADC_Callback)(uint32_t *blockAddr, uint32_t count);

/**
 * Initializes the internal ADC interface
 */
void InternalADC_Init(void);

/**
 * Starts the autonomous conversion process. Callback will be called from an ISR
 * context on every completed DMA block
 *
 * @param callback - Callback function
 */
void InternalADC_StartSampling(ADC_Callback callback);

/**
 * Stops the background ADC processing
 */
void InternalADC_StopSampling(void);

#endif // EXAMPLES_MAX32672_ADC_DMA_RMS_INTERNALADC_H_
