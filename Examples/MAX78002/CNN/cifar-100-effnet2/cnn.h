/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically generated for the cifar-100-effnet2 network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef __CNN_H__
#define __CNN_H__

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK   1

/*
  SUMMARY OF OPS
  Hardware: 168,397,824 ops (166,899,712 macc; 1,158,144 comp; 339,968 add; 0 mul; 0 bitwise)
    Layer 0: 232,448 ops (221,184 macc; 11,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 131,072 ops (131,072 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 2,375,680 ops (2,359,296 macc; 16,384 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (l3): 524,288 ops (524,288 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (res00): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 9,469,952 ops (9,437,184 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (res01): 1,048,576 ops (1,048,576 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 8,192 ops (0 macc; 0 comp; 8,192 add; 0 mul; 0 bitwise)
    Layer 8: 9,469,952 ops (9,437,184 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (l8): 1,572,864 ops (1,572,864 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (res10): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 21,282,816 ops (21,233,664 macc; 49,152 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12 (res11): 2,359,296 ops (2,359,296 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 12,288 ops (0 macc; 0 comp; 12,288 add; 0 mul; 0 bitwise)
    Layer 14: 2,408,448 ops (2,359,296 macc; 49,152 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 491,520 ops (442,368 macc; 49,152 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16 (l14): 4,718,592 ops (4,718,592 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (res20): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 9,535,488 ops (9,437,184 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19: 983,040 ops (884,736 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (res21): 9,437,184 ops (9,437,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21: 24,576 ops (0 macc; 0 comp; 24,576 add; 0 mul; 0 bitwise)
    Layer 22: 9,535,488 ops (9,437,184 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 23: 983,040 ops (884,736 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 24 (l22): 12,582,912 ops (12,582,912 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 25 (res30): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26: 16,908,288 ops (16,777,216 macc; 131,072 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27: 1,310,720 ops (1,179,648 macc; 131,072 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28 (res31): 16,777,216 ops (16,777,216 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29: 32,768 ops (0 macc; 0 comp; 32,768 add; 0 mul; 0 bitwise)
    Layer 30: 33,816,576 ops (33,554,432 macc; 262,144 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31: 262,144 ops (0 macc; 0 comp; 262,144 add; 0 mul; 0 bitwise)
    Layer 32: 102,400 ops (102,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 753,952 bytes out of 2,396,160 bytes total (31%)
  Bias memory:   5,236 bytes out of 8,192 bytes total (64%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 100

/* Use this timer to time the inference */
#define CNN_INFERENCE_TIMER MXC_TMR0

/* Port pin actions used to signal that processing is active */

#define CNN_START    LED_On(1)
#define CNN_COMPLETE LED_Off(1)
#define SYS_START    LED_On(0)
#define SYS_COMPLETE LED_Off(0)

/* Run software SoftMax on unloaded data */
void softmax_q17p14_q15(const q31_t* vec_in, const uint16_t dim_vec, q15_t* p_out);
/* Shift the input, then calculate SoftMax */
void softmax_shift_q17p14_q15(q31_t* vec_in, const uint16_t dim_vec, uint8_t in_shift,
                              q15_t* p_out);

/* Stopwatch - holds the runtime when accelerator finishes */
extern volatile uint32_t cnn_time;

/* Custom memcopy routines used for weights and data */
void memcpy32(uint32_t* dst, const uint32_t* src, int n);
void memcpy32_const(uint32_t* dst, int n);

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
int cnn_unload(uint32_t* out_buf);

/* Turn on the boost circuit */
int cnn_boost_enable(mxc_gpio_regs_t* port, uint32_t pin);

/* Turn off the boost circuit */
int cnn_boost_disable(mxc_gpio_regs_t* port, uint32_t pin);

#endif // __CNN_H__
