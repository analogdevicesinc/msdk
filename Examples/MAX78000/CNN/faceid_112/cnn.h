/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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
 * This header file was automatically @generated for the faceid_112 network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef __CNN_H__
#define __CNN_H__

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK 1

/*
  SUMMARY OF OPS
  Hardware: 199,784,640 ops (198,019,072 macc; 1,746,752 comp; 18,816 add; 0 mul; 0 bitwise)
    Layer 0: 11,239,424 ops (10,838,016 macc; 401,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 29,403,136 ops (28,901,376 macc; 501,760 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 58,003,456 ops (57,802,752 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 21,876,736 ops (21,676,032 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 7,375,872 ops (7,225,344 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 21,826,560 ops (21,676,032 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 1,630,720 ops (1,605,632 macc; 25,088 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 14,450,688 ops (14,450,688 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9: 12,544 ops (0 macc; 0 comp; 12,544 add; 0 mul; 0 bitwise)
    Layer 10: 3,261,440 ops (3,211,264 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 10,888,192 ops (10,838,016 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 912,576 ops (903,168 macc; 9,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 10,838,016 ops (10,838,016 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 809,088 ops (802,816 macc; 6,272 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 7,225,344 ops (7,225,344 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 22,656 ops (16,384 macc; 0 comp; 6,272 add; 0 mul; 0 bitwise)
    Layer 17: 8,192 ops (8,192 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 365,408 bytes out of 442,368 bytes total (82.6%)
  Bias memory:   1,296 bytes out of 2,048 bytes total (63.3%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 64

/* Use this timer to time the inference */
#define CNN_INFERENCE_TIMER MXC_TMR0

/* Port pin actions used to signal that processing is active */

#define CNN_START LED_On(1)
#define CNN_COMPLETE LED_Off(1)
#define SYS_START LED_On(0)
#define SYS_COMPLETE LED_Off(0)

/* Run software SoftMax on unloaded data */
void softmax_q17p14_q15(const q31_t *vec_in, const uint16_t dim_vec, q15_t *p_out);
/* Shift the input, then calculate SoftMax */
void softmax_shift_q17p14_q15(q31_t *vec_in, const uint16_t dim_vec, uint8_t in_shift,
                              q15_t *p_out);

/* Stopwatch - holds the runtime when accelerator finishes */
extern volatile uint32_t cnn_time;

/* Custom memcopy routines used for weights and data */
void memcpy32(uint32_t *dst, const uint32_t *src, int n);
void memcpy32_const(uint32_t *dst, int n);

/* Enable clocks and power to accelerator, enable interrupt */
int cnn_enable(uint32_t clock_source, uint32_t clock_divider);

/* Disable clocks and power to accelerator */
int cnn_disable(void);

/* Perform minimum accelerator initialization so it can be configured */
int cnn_init(void);

/* Configure accelerator for the given network */
int cnn_configure(void);

/* Load accelerator weights */
int cnn_load_weights(void);

/* Verify accelerator weights (debug only) */
int cnn_verify_weights(void);

/* Load accelerator bias values (if needed) */
int cnn_load_bias(void);

/* Start accelerator processing */
int cnn_start(void);

/* Force stop accelerator */
int cnn_stop(void);

/* Continue accelerator after stop */
int cnn_continue(void);

/* Unload results from accelerator */
int cnn_unload(uint32_t *out_buf);

/* Turn on the boost circuit */
int cnn_boost_enable(mxc_gpio_regs_t *port, uint32_t pin);

/* Turn off the boost circuit */
int cnn_boost_disable(mxc_gpio_regs_t *port, uint32_t pin);

#endif // __CNN_H__
