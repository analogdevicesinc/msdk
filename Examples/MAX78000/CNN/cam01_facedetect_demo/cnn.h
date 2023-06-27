/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically @generated for the facedet_tinierssd network from a template.
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
  Hardware: 589,595,888 ops (588,006,720 macc; 1,589,168 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 4,327,680 ops (4,064,256 macc; 263,424 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 11,063,808 ops (10,838,016 macc; 225,792 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 43,502,592 ops (43,352,064 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 86,854,656 ops (86,704,128 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 86,854,656 ops (86,704,128 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 86,854,656 ops (86,704,128 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 173,709,312 ops (173,408,256 macc; 301,056 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7 (backbone_conv8): 86,779,392 ops (86,704,128 macc; 75,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8 (backbone_conv9): 5,513,088 ops (5,419,008 macc; 94,080 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (backbone_conv10): 1,312,640 ops (1,290,240 macc; 22,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (conv12_1): 647,360 ops (645,120 macc; 2,240 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (conv12_2): 83,440 ops (80,640 macc; 2,800 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 1,354,752 ops (1,354,752 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 40,320 ops (40,320 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 677,376 ops (677,376 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 20,160 ops (20,160 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 275,184 bytes out of 442,368 bytes total (62.2%)
  Bias memory:   536 bytes out of 2,048 bytes total (26.2%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 140

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
