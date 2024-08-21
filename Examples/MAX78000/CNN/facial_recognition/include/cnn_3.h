/******************************************************************************
 *
 * Copyright (C) 2019-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

/*
 * This header file was automatically @generated for the 85_dotprod_112_noffset_new_k network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef __CNN_3_H__
#define __CNN_3_H__

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK 1

/*
  SUMMARY OF OPS
  Hardware: 65,536 ops (65,536 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 65,536 ops (65,536 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 65,536 bytes out of 442,368 bytes total (14.8%)
  Bias memory:   0 bytes out of 2,048 bytes total (0.0%)
*/

/* Number of outputs for this network */
#define CNN_3_NUM_OUTPUTS 1024
#define CNN_3_OUTPUT_SHIFT 2
#define Threshold 70 //70 IJBB & IJBC

/* Stopwatch - holds the runtime when accelerator finishes */
extern volatile uint32_t cnn_time;

/* Enable clocks and power to accelerator, enable interrupt */
int cnn_3_enable(uint32_t clock_source, uint32_t clock_divider);

/* Perform minimum accelerator initialization so it can be configured */
int cnn_3_init(void);

/* Configure accelerator for the given network */
int cnn_3_configure(void);

/* Load accelerator weights */
int cnn_3_load_weights(void);

/* Load accelerator bias values (if needed) */
int cnn_3_load_bias(void);

/* Start accelerator processing */
int cnn_3_start(void);

/* Unload results from accelerator */
int cnn_3_unload(uint32_t *out_buf);

#endif // __CNN_3_H__
