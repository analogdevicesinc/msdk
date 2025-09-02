/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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
 * This header file was automatically @generated for the spectrumsense network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef EXAMPLES_MAX78000_CNN_SPECTRUMSENSE_UNET_DEMO_CNN_H_
#define EXAMPLES_MAX78000_CNN_SPECTRUMSENSE_UNET_DEMO_CNN_H_

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK 1

/*
  SUMMARY OF OPS
  Hardware: 623,246,800 ops (619,767,808 macc; 3,478,992 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 24,285,184 ops (23,789,568 macc; 495,616 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 32,215,040 ops (31,719,424 macc; 495,616 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 16,107,520 ops (15,859,712 macc; 247,808 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 17,904,128 ops (17,842,176 macc; 61,952 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 4,019,136 ops (3,902,976 macc; 116,160 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 6,911,520 ops (6,830,208 macc; 81,312 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 6,870,864 ops (6,830,208 macc; 40,656 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 1,517,824 ops (1,517,824 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 27,320,832 ops (27,320,832 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9: 27,347,936 ops (27,320,832 macc; 27,104 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10: 27,320,832 ops (27,320,832 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 27,375,040 ops (27,320,832 macc; 54,208 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 15,611,904 ops (15,611,904 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 53,898,240 ops (53,526,528 macc; 371,712 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 214,601,728 ops (214,106,112 macc; 495,616 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 32,215,040 ops (31,719,424 macc; 495,616 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 32,215,040 ops (31,719,424 macc; 495,616 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17: 31,719,424 ops (31,719,424 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 23,789,568 ops (23,789,568 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 280,288 bytes out of 442,368 bytes total (63.4%)
  Bias memory:   892 bytes out of 2,048 bytes total (43.6%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 371712

/* Port pin actions used to signal that processing is active */

#define CNN_START LED_On(1)
#define CNN_COMPLETE LED_Off(1)
#define SYS_START LED_On(0)
#define SYS_COMPLETE LED_Off(0)

/* Run software SoftMax on unloaded data */
void softmax_q17p14_q15(const q31_t * vec_in, const uint16_t dim_vec, q15_t * p_out);
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

#endif // EXAMPLES_MAX78000_CNN_SPECTRUMSENSE_UNET_DEMO_CNN_H_
