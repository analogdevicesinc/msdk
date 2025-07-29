/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically @generated for the imagenet-riscv network from a template.
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
  Hardware: 1,018,869,248 ops (1,014,353,408 macc; 3,926,272 comp; 589,568 add; 0 mul; 0 bitwise)
    Layer 0 (l0): 2,847,488 ops (2,709,504 macc; 137,984 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1 (l1): 1,605,632 ops (1,605,632 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2 (l2): 29,102,080 ops (28,901,376 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (l3): 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (l4): 116,006,912 ops (115,605,504 macc; 401,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5 (l5): 12,845,056 ops (12,845,056 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (l6): 100,352 ops (0 macc; 0 comp; 100,352 add; 0 mul; 0 bitwise)
    Layer 7 (l7): 116,006,912 ops (115,605,504 macc; 401,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8 (l8): 19,267,584 ops (19,267,584 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (l9): 260,714,496 ops (260,112,384 macc; 602,112 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (l10): 28,901,376 ops (28,901,376 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (l11): 150,528 ops (0 macc; 0 comp; 150,528 add; 0 mul; 0 bitwise)
    Layer 12 (l12): 260,714,496 ops (260,112,384 macc; 602,112 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13 (l13): 28,901,376 ops (28,901,376 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14 (l14): 338,688 ops (0 macc; 301,056 comp; 37,632 add; 0 mul; 0 bitwise)
    Layer 15 (l15): 3,687,936 ops (3,612,672 macc; 75,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16 (l16): 752,640 ops (677,376 macc; 75,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (l17): 7,225,344 ops (7,225,344 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18 (l18): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19 (l19): 14,601,216 ops (14,450,688 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (l20): 1,505,280 ops (1,354,752 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21 (l21): 14,450,688 ops (14,450,688 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 22 (l22): 75,264 ops (0 macc; 0 comp; 75,264 add; 0 mul; 0 bitwise)
    Layer 23 (l23): 14,601,216 ops (14,450,688 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 24 (l24): 1,505,280 ops (1,354,752 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 25 (l25): 19,267,584 ops (19,267,584 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26 (l26): 3,336,704 ops (3,211,264 macc; 125,440 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27 (l27): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28 (l28): 12,945,408 ops (12,845,056 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29 (l29): 1,003,520 ops (903,168 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 30 (l30): 12,845,056 ops (12,845,056 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31 (l31): 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 32 (l32): 25,890,816 ops (25,690,112 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 33 (l33): 1,224,704 ops (1,024,000 macc; 0 comp; 200,704 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 1,686,080 bytes out of 2,396,160 bytes total (70.4%)
  Bias memory:   5,544 bytes out of 8,192 bytes total (67.7%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 1000

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
