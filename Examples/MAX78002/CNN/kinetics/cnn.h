/******************************************************************************
 *
 * Copyright (C) 2019-2023 Maxim Integrated Products, Inc. All Rights Reserved.
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
 * This header file was automatically @generated for the kinetics network from a template.
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
  Hardware: 344,024,768 ops (342,827,488 macc; 1,179,168 comp; 18,112 add; 0 mul; 0 bitwise)
    Layer 0: 22,348,800 ops (22,118,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 132,940,800 ops (132,710,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 132,940,800 ops (132,710,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (res1_out): 33,465,600 ops (33,177,600 macc; 288,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (conv2): 8,366,400 ops (8,294,400 macc; 72,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5 (conv2_1): 936,000 ops (921,600 macc; 14,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (conv2_p): 8,366,400 ops (8,294,400 macc; 72,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7 (res2_out): 14,400 ops (0 macc; 0 comp; 14,400 add; 0 mul; 0 bitwise)
    Layer 8 (conv3): 1,822,016 ops (1,806,336 macc; 15,680 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (conv3_1): 203,840 ops (200,704 macc; 3,136 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (conv3_p): 1,822,016 ops (1,806,336 macc; 15,680 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (res3_out): 3,136 ops (0 macc; 0 comp; 3,136 add; 0 mul; 0 bitwise)
    Layer 12 (conv4): 334,656 ops (331,776 macc; 2,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13 (conv4_1): 37,440 ops (36,864 macc; 576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14 (conv4_p): 334,656 ops (331,776 macc; 2,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15 (res4_out): 576 ops (0 macc; 0 comp; 576 add; 0 mul; 0 bitwise)
    Layer 16 (buffer_shift): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (conv5): 18,464 ops (18,432 macc; 32 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18 (tcn0): 40,352 ops (39,936 macc; 416 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19 (tcn1): 27,936 ops (27,648 macc; 288 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (tcn2): 480 ops (480 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 378,272 bytes out of 2,396,160 bytes total (15.8%)
  Bias memory:   933 bytes out of 8,192 bytes total (11.4%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 5

/* Use this timer to time the inference */
#define CNN_INFERENCE_TIMER MXC_TMR0

/* Port pin actions used to signal that processing is active */

#define CNN_START LED_On(1)
#define CNN_COMPLETE LED_Off(1)
#define SYS_START LED_On(0)
#define SYS_COMPLETE LED_Off(0)

/* Run software SoftMax on unloaded data */
void softmax_q17p14_q15(const q31_t * vec_in, const uint16_t dim_vec, q15_t * p_out);
/* Shift the input, then calculate SoftMax */
void softmax_shift_q17p14_q15(q31_t * vec_in, const uint16_t dim_vec, uint8_t in_shift, q15_t * p_out);

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
