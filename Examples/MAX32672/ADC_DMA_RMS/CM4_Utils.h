/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
#ifndef EXAMPLES_MAX32672_ADC_DMA_RMS_CM4_UTILS_H_
#define EXAMPLES_MAX32672_ADC_DMA_RMS_CM4_UTILS_H_

#include <stdint.h>

/*
 * Helper union for working with 64-bit data in memory and pulling out the
 * 32-bit components
 */
typedef union {
    int64_t i64;
    struct {
        int32_t i32lo;
        int32_t i32hi;
    } words;
} int64_wrapper_t;

/**
 * Helper union for working with 32-bit data in memory and pulling out the
 * 16-bit components
 */
typedef union {
    int32_t i32;
    struct {
        int16_t i16lo;
        int16_t i16hi;
    } words;
} int32_wrapper_t;

/*******************************************************************************
 * The GCC built in intrinsics for assembly calls is not complete, specifically
 * with instructions that have optional extra features built into the encodings
 * such as UXTAH {<Rd>,} <Rn>, <Rm> {, <rotation>} where the rotation value is
 * 8,16, or 24 bits, and is part of the actual instruction.
 *
 * This collection of inline functions provides shortcuts to including assembly
 * calls into code
 */

/* Signed Multiply, Accumulate Long, Bottom x Bottom
 * This multiplies the bottom 16-bits of both word1 and word2 together, and adds
 * result to the 64-bit accumulation value
 */
inline int64_t __SMLALBB(int32_t word1, int32_t word2, int64_t acc)
{
    int64_wrapper_t llr = { .i64 = acc };

    asm("smlalbb %0, %1, %2, %3"
        : "=r"(llr.words.i32lo), "=r"(llr.words.i32hi)
        : "r"(word1), "r"(word2), "0"(llr.words.i32lo), "1"(llr.words.i32hi));
    return (llr.i64);
}

/* Signed Multiply, Accumulate Long, Top x Top
 * This multiplies the top 16-bits of both word1 and word2 together, and adds
 * result to the 64-bit accumulation value
 */
inline int64_t __SMLALTT(int32_t word1, int32_t word2, int64_t acc)
{
    int64_wrapper_t llr = { .i64 = acc };

    asm("smlaltt %0, %1, %2, %3"
        : "=r"(llr.words.i32lo), "=r"(llr.words.i32hi)
        : "r"(word1), "r"(word2), "0"(llr.words.i32lo), "1"(llr.words.i32hi));
    return (llr.i64);
}

/* Dual signed 16 subtraction. Subtracts each halfword from the second operand
 * from the corresponding halfword of the first operand
 */
inline uint32_t __SSUB16(uint32_t src1, uint32_t src2)
{
    uint32_t result;
    asm("ssub16 %0, %1, %2 " : "=r "(result) : "r"(src1), "r"(src2));
    return (result);
}

/**
 * Our implementation of a integer square root function for 32-bit values
 *
 * @param a - Value to calculate square root of
 * @returns Square root value
 */
uint16_t sqrt32(uint32_t a);

#endif // EXAMPLES_MAX32672_ADC_DMA_RMS_CM4_UTILS_H_
