/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically generated for the cifar-100-residual network from a template.
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
  Hardware: 18,636,416 ops (18,461,184 macc; 146,560 comp; 28,672 add; 0 mul; 0 bitwise)
    Layer 0: 458,752 ops (442,368 macc; 16,384 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 2,969,600 ops (2,949,120 macc; 20,480 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 3,706,880 ops (3,686,400 macc; 20,480 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 3,727,360 ops (3,686,400 macc; 20,480 comp; 20,480 add; 0 mul; 0 bitwise)
    Layer 5: 947,200 ops (921,600 macc; 25,600 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 926,720 ops (921,600 macc; 5,120 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 2,043,904 ops (2,027,520 macc; 11,264 comp; 5,120 add; 0 mul; 0 bitwise)
    Layer 9: 1,230,848 ops (1,216,512 macc; 14,336 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 1,330,176 ops (1,327,104 macc; 3,072 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 671,232 ops (663,552 macc; 4,608 comp; 3,072 add; 0 mul; 0 bitwise)
    Layer 13: 200,192 ops (196,608 macc; 3,584 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 262,656 ops (262,144 macc; 512 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 148,096 ops (147,456 macc; 640 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 12,800 ops (12,800 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 381,792 bytes out of 442,368 bytes total (86%)
  Bias memory:   0 bytes out of 2,048 bytes total (0%)
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
