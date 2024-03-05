/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically @generated for the dotprod_comb_112 network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef __CNN_3_H__
#define __CNN_3_H__

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK 1

/*
  SUMMARY OF OPS
  Hardware: 65,536 ops (65,536 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 73: 65,536 ops (65,536 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 65,536 bytes out of 2,396,160 bytes total (2.7%)
  Bias memory:   0 bytes out of 8,192 bytes total (0.0%)
*/

/* Number of outputs for this network */
#define CNN_3_NUM_OUTPUTS 1024
#define CNN_3_OUTPUT_SHIFT 2

#define Threshold 64

/* Stopwatch - holds the runtime when accelerator finishes */
extern volatile uint32_t cnn_time;

/* Enable clocks and power to accelerator, enable interrupt */
int cnn_3_enable(uint32_t clock_source, uint32_t clock_divider);

/* Perform minimum accelerator initialization so it can be configured */
int cnn_3_init(void);

/* Configure accelerator for the given network */
int cnn_3_configure(void);

/* Load accelerator weights */
int cnn_3_load_weights(void);

/* Verify accelerator weights (debug only) */
int cnn_3_verify_weights(void);

/* Load accelerator bias values (if needed) */
int cnn_3_load_bias(void);

/* Start accelerator processing */
int cnn_3_start(void);

/* Unload results from accelerator */
int cnn_3_unload(uint32_t *out_buf);

#endif // __CNN_3_H__
