/*
 * Copyright (C) 2010-2018 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* 
 * Portions Copyright (C) 2020 Maxim Integrated Products, Inc.
 */

/* ----------------------------------------------------------------------
 * Project:      NN Library
 * Title:        softmax.c
 * Description:  Q17.14 softmax function with Q15 output
 *
 * $Date:        22. April 2020
 * $Revision:    V.1.0.1
 *
 * Target Processor:  Cortex-M and RISC-V cores
 *
 * -------------------------------------------------------------------- */

#include "mxc.h"
#include "cnn.h"

/**
 *  @ingroup groupNN
 */

/**
 * @addtogroup Softmax
 * @{
 */

  /**
   * @brief Q17.14 fixed point softmax function, returns Q15
   * @param[in]       vec_in      pointer to input vector
   * @param[in]       dim_vec     input vector dimension
   * @param[out]      p_out       pointer to output vector
   * @return none.
   *
   * @details
   *
   *  Here, instead of typical e based softmax, we use
   *  2-based softmax, i.e.,:
   *
   *  y_i = 2^(x_i/16384) / sum(2^(x_j/16384))
   *
   *  The relative output will be different here.
   *  But mathematically, the gradient will be the same
   *  with a log(2) scaling factor.
   */

void softmax_q17p14_q15(const q31_t * vec_in, const uint16_t dim_vec, q15_t * p_out)
{
    q31_t     sum;
    int16_t   i;
    uint8_t   shift;
    q31_t     base;
    base = -1 * 0x80000000;

    for (i = 0; i < dim_vec; i++)
    {
        if (vec_in[i] > base)
        {
            base = vec_in[i];
        }
    }

    /* we ignore really small values
     * anyway, they will be 0 after shrinking
     * to q15_t
     */

    base = base - (16<<14);

    sum = 0;

    for (i = 0; i < dim_vec; i++)
    {
        if (vec_in[i] > base)
        {
            shift = (uint8_t)((8192 + vec_in[i] - base) >> 14);
            sum += (0x1 << shift);
        }
    }


    /* This is effectively (0x1 << 32) / sum */
    int64_t div_base = 0x100000000LL;
    int32_t output_base = (int32_t)(div_base / sum);
    int32_t out;

    /* Final confidence will be output_base >> ( 17 - (vec_in[i] - base)>>14 )
     * so 32768 (0x1<<15) -> 100% confidence when sum = 0x1 << 16, output_base = 0x1 << 16
     * and vec_in[i]-base = 16
     */

    for (i = 0; i < dim_vec; i++)
    {
        if (vec_in[i] > base)
        {
            /* Here minimum value of 17+base-vec[i] will be 1 */
            shift = (uint8_t)(17+((8191 + base - vec_in[i]) >> 14));

            out = (output_base >> shift);

            if (out > 32767)
            	out = 32767;

            p_out[i] = (q15_t)out;


        } else
        {
            p_out[i] = 0;
        }
    }

}

  /**
   * @brief Q17.14 fixed point softmax function with input shift, returns Q15
   * @param[in]       vec_in      pointer to input vector
   * @param[in]       dim_vec     input vector dimension
   * @param[in]       in_shift    input vector shift count
   * @param[out]      p_out       pointer to output vector
   * @return none.
   *
   * @details
   *
   *  Here, instead of typical e based softmax, we use
   *  2-based softmax, i.e.,:
   *
   *  y_i = 2^(x_i/16384) / sum(2^(x_j/16384))
   *
   *  The relative output will be different here.
   *  But mathematically, the gradient will be the same
   *  with a log(2) scaling factor.
   */

void softmax_shift_q17p14_q15(q31_t * vec_in, const uint16_t dim_vec, uint8_t in_shift, q15_t * p_out)
{
    int16_t   i;

    for (i = 0; i < dim_vec; i++)
    {
        vec_in[i] <<= in_shift;
    }

    softmax_q17p14_q15(vec_in, dim_vec, p_out);
}

/**
 * @} end of Softmax group
 */
