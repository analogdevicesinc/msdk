/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically @generated for the qrcode_tinierssd_nobias network from a template.
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
  Hardware: 1,728,946,640 ops (1,717,787,520 macc; 11,159,120 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 68,812,800 ops (66,355,200 macc; 2,457,600 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 710,246,400 ops (707,788,800 macc; 2,457,600 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 357,580,800 ops (353,894,400 macc; 3,686,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 178,483,200 ops (176,947,200 macc; 1,536,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 177,254,400 ops (176,947,200 macc; 307,200 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 44,620,800 ops (44,236,800 macc; 384,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 44,313,600 ops (44,236,800 macc; 76,800 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 88,627,200 ops (88,473,600 macc; 153,600 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8 (fire8): 44,275,200 ops (44,236,800 macc; 38,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (fire9): 2,812,800 ops (2,764,800 macc; 48,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (fire10): 656,320 ops (645,120 macc; 11,200 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 323,680 ops (322,560 macc; 1,120 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12 (conv12_2): 35,760 ops (34,560 macc; 1,200 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 5,529,600 ops (5,529,600 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 1,382,400 ops (1,382,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 322,560 ops (322,560 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 34,560 ops (34,560 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17: 2,764,800 ops (2,764,800 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 691,200 ops (691,200 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19: 161,280 ops (161,280 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20: 17,280 ops (17,280 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 336,096 bytes out of 442,368 bytes total (76.0%)
  Bias memory:   416 bytes out of 2,048 bytes total (20.3%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 120

/* Use this timer to time the inference */
#define CNN_INFERENCE_TIMER MXC_TMR1

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
