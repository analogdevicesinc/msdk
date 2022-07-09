/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically generated for the cifar-100-simplewide2x-mixed network from a template.
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
  Hardware: 43,182,528 ops (42,957,568 macc; 224,960 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 688,128 ops (663,552 macc; 24,576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 7,110,656 ops (7,077,888 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 9,469,952 ops (9,437,184 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 9,469,952 ops (9,437,184 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 2,400,256 ops (2,359,296 macc; 40,960 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 2,367,488 ops (2,359,296 macc; 8,192 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 4,734,976 ops (4,718,592 macc; 16,384 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 2,379,776 ops (2,359,296 macc; 20,480 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 2,363,392 ops (2,359,296 macc; 4,096 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9: 1,185,792 ops (1,179,648 macc; 6,144 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10: 266,240 ops (262,144 macc; 4,096 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 393,984 ops (393,216 macc; 768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 332,736 ops (331,776 macc; 960 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 19,200 ops (19,200 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 312,200 bytes out of 2,396,160 bytes total (13%)
  Bias memory:   1,500 bytes out of 8,192 bytes total (18%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 100

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
