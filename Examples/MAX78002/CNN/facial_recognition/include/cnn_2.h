/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically @generated for the mobilefacenet_comb_112 network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef __CNN_2_H__
#define __CNN_2_H__

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK 1

/*
  SUMMARY OF OPS
  Hardware: 445,470,720 ops (440,252,416 macc; 4,848,256 comp; 370,048 add; 0 mul; 0 bitwise)
    Layer 0: 22,478,848 ops (21,676,032 macc; 802,816 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 2,809,856 ops (1,806,336 macc; 1,003,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 231,612,416 ops (231,211,008 macc; 401,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 1,404,928 ops (903,168 macc; 501,760 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 6,522,880 ops (6,422,528 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 1,003,520 ops (903,168 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9: 50,176 ops (0 macc; 0 comp; 50,176 add; 0 mul; 0 bitwise)
    Layer 10: 6,522,880 ops (6,422,528 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 1,003,520 ops (903,168 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 50,176 ops (0 macc; 0 comp; 50,176 add; 0 mul; 0 bitwise)
    Layer 15: 6,522,880 ops (6,422,528 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 1,003,520 ops (903,168 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19: 50,176 ops (0 macc; 0 comp; 50,176 add; 0 mul; 0 bitwise)
    Layer 20: 6,522,880 ops (6,422,528 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21: 1,003,520 ops (903,168 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 22: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 23: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 24: 50,176 ops (0 macc; 0 comp; 50,176 add; 0 mul; 0 bitwise)
    Layer 25: 13,045,760 ops (12,845,056 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26: 702,464 ops (451,584 macc; 250,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28: 6,472,704 ops (6,422,528 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29: 501,760 ops (451,584 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 30: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 32: 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 33: 6,472,704 ops (6,422,528 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 34: 501,760 ops (451,584 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 35: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 36: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 37: 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 38: 6,472,704 ops (6,422,528 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 39: 501,760 ops (451,584 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 40: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 41: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 42: 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 43: 6,472,704 ops (6,422,528 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 44: 501,760 ops (451,584 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 45: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 46: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 47: 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 48: 6,472,704 ops (6,422,528 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 49: 501,760 ops (451,584 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 50: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 51: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 52: 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 53: 6,472,704 ops (6,422,528 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 54: 501,760 ops (451,584 macc; 50,176 comp; 0 add; 0 mul; 0 bitwise)
    Layer 55: 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 56: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 57: 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 58: 12,945,408 ops (12,845,056 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 59: 351,232 ops (225,792 macc; 125,440 comp; 0 add; 0 mul; 0 bitwise)
    Layer 60: 3,211,264 ops (3,211,264 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 61: 1,618,176 ops (1,605,632 macc; 12,544 comp; 0 add; 0 mul; 0 bitwise)
    Layer 62: 125,440 ops (112,896 macc; 12,544 comp; 0 add; 0 mul; 0 bitwise)
    Layer 63: 1,605,632 ops (1,605,632 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 64: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 65: 6,272 ops (0 macc; 0 comp; 6,272 add; 0 mul; 0 bitwise)
    Layer 66: 1,618,176 ops (1,605,632 macc; 12,544 comp; 0 add; 0 mul; 0 bitwise)
    Layer 67: 125,440 ops (112,896 macc; 12,544 comp; 0 add; 0 mul; 0 bitwise)
    Layer 68: 1,605,632 ops (1,605,632 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 69: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 70: 6,272 ops (0 macc; 0 comp; 6,272 add; 0 mul; 0 bitwise)
    Layer 71: 809,088 ops (802,816 macc; 6,272 comp; 0 add; 0 mul; 0 bitwise)
    Layer 72: 14,464 ops (8,192 macc; 0 comp; 6,272 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 909,952 bytes out of 2,396,160 bytes total (38.0%)
  Bias memory:   7,296 bytes out of 8,192 bytes total (89.1%)
*/

/* Number of outputs for this network */
#define CNN_2_NUM_OUTPUTS 64

/* Stopwatch - holds the runtime when accelerator finishes */
extern volatile uint32_t cnn_time;

/* Enable clocks and power to accelerator, enable interrupt */
int cnn_2_enable(uint32_t clock_source, uint32_t clock_divider);

/* Perform minimum accelerator initialization so it can be configured */
int cnn_2_init(void);

/* Configure accelerator for the given network */
int cnn_2_configure(void);

/* Load accelerator weights */
int cnn_2_load_weights(void);

/* Verify accelerator weights (debug only) */
int cnn_2_verify_weights(void);

/* Load accelerator bias values (if needed) */
int cnn_2_load_bias(void);

/* Start accelerator processing */
int cnn_2_start(void);

/* Unload results from accelerator */
int cnn_2_unload(uint32_t *out_buf);

#endif // __CNN_2_H__
