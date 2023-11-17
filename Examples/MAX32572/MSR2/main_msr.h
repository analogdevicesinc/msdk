/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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

#ifndef MAIN_MSR_H_
#define MAIN_MSR_H_

/* Structure to contain result of a track decode */
typedef struct {
  uint8_t error_code;   /**< Error code value */
  uint8_t parity_errs;  /**< Number of characters with parity errors */
  uint8_t lrc;          /**< LRC check value. A value of '0' indicates a
                             successful LRC check. Any other value should be
                             considered a failure. */
  uint8_t direction;    /**< Swipe direction determined from decode */
  uint16_t len;          /**< Number or decoded characters. This does not include
                             the sentinels or the LRC. */
  uint16_t speed;       /**< Approximate swipe rate in (0.1 in/sec) units */
  uint8_t data[136];    /**< The decoded data, characters */
} msr_decoded_track_t;

/* Structure to set waveform filtering parameters */
typedef   struct  {
  uint8_t   flens[4];     /* filter OSR values */
  uint16_t  fscale_up[4]; /* filter scale up factors */
  uint16_t  spd_thr[4];   /*  speed zones settings, 0.1 in/s  units;
                              last element[3] is bit density, bit/in */
  uint16_t  thr_fct[4];   /*  search threshold settings */
  uint16_t  fscale_dn;    /*  filter scale down shift */
  uint16_t  ilen;         /* initial FLENS index {0-3} */
} track_flt_setup_t;

/* Structure to hold Track encoding parameters */
typedef struct {
  uint8_t startsen; /* start sentinel */
  uint8_t endsen;   /* end sentinel */
  uint8_t nbits;    /* # of bits per char */
  uint8_t density;  /* bit density used for swipe rate calculation (Note: Track 2 is 2x normal density) */
  uint8_t *charset; /* character set */
} parse_bits_t;

/* Structure to hold HW register setup */
typedef struct  {
  uint32_t  address;  /* HW register address */
  uint16_t  value;    /* setup value */
  uint8_t   pos;      /* setup bit-field position */
  uint8_t   len;      /* setup bit-field length */
} hw_register_setup_t;

#endif /* MAIN_MSR_H_ */
