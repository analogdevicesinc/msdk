/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically generated for the cifar-100-mobilenet-v2-0.75 network from a template.
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
  Hardware: 26,293,760 ops (25,695,744 macc; 566,016 comp; 32,000 add; 0 mul; 0 bitwise)
    Layer 0: 688,128 ops (663,552 macc; 24,576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 245,760 ops (221,184 macc; 24,576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 294,912 ops (294,912 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 958,464 ops (884,736 macc; 73,728 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 258,048 ops (165,888 macc; 92,160 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 368,640 ops (368,640 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 645,120 ops (614,400 macc; 30,720 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 307,200 ops (276,480 macc; 30,720 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 614,400 ops (614,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10: 5,120 ops (0 macc; 0 comp; 5,120 add; 0 mul; 0 bitwise)
    Layer 11: 645,120 ops (614,400 macc; 30,720 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 107,520 ops (69,120 macc; 38,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 184,320 ops (184,320 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 230,400 ops (221,184 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 92,160 ops (82,944 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 1,536 ops (0 macc; 0 comp; 1,536 add; 0 mul; 0 bitwise)
    Layer 19: 230,400 ops (221,184 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20: 92,160 ops (82,944 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 22: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 23: 1,536 ops (0 macc; 0 comp; 1,536 add; 0 mul; 0 bitwise)
    Layer 24: 230,400 ops (221,184 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 25: 32,256 ops (20,736 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26: 110,592 ops (110,592 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 30: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31: 768 ops (0 macc; 0 comp; 768 add; 0 mul; 0 bitwise)
    Layer 32: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 33: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 34: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 35: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 36: 768 ops (0 macc; 0 comp; 768 add; 0 mul; 0 bitwise)
    Layer 37: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 38: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 39: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 40: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 41: 768 ops (0 macc; 0 comp; 768 add; 0 mul; 0 bitwise)
    Layer 42: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 43: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 44: 331,776 ops (331,776 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 45: 504,576 ops (497,664 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 46: 69,120 ops (62,208 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 47: 497,664 ops (497,664 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 48: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 49: 1,152 ops (0 macc; 0 comp; 1,152 add; 0 mul; 0 bitwise)
    Layer 50: 504,576 ops (497,664 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 51: 69,120 ops (62,208 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 52: 497,664 ops (497,664 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 53: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 54: 1,152 ops (0 macc; 0 comp; 1,152 add; 0 mul; 0 bitwise)
    Layer 55: 504,576 ops (497,664 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 56: 69,120 ops (62,208 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 57: 829,440 ops (829,440 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 58: 1,393,920 ops (1,382,400 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 59: 115,200 ops (103,680 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 60: 1,382,400 ops (1,382,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 61: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 62: 1,920 ops (0 macc; 0 comp; 1,920 add; 0 mul; 0 bitwise)
    Layer 63: 1,393,920 ops (1,382,400 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 64: 115,200 ops (103,680 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 65: 1,382,400 ops (1,382,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 66: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 67: 1,920 ops (0 macc; 0 comp; 1,920 add; 0 mul; 0 bitwise)
    Layer 68: 1,393,920 ops (1,382,400 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 69: 115,200 ops (103,680 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 70: 2,764,800 ops (2,764,800 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 71: 3,701,760 ops (3,686,400 macc; 15,360 comp; 0 add; 0 mul; 0 bitwise)
    Layer 72: 111,360 ops (96,000 macc; 0 comp; 15,360 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 1,341,960 bytes out of 2,396,160 bytes total (56%)
  Bias memory:   6,508 bytes out of 8,192 bytes total (79%)
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
