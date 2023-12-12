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

#include <string.h>
#include <ucl/ucl_hash.h>

#ifdef HASH_SHA256

#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>

#include <hash/sha256.h>

static u32 _wsb_b2w(u8 *src)
{
    return ((u32)src[3] | ((u32)src[2] << 8) | ((u32)src[1] << 16) | ((u32)src[0] << 24));
}

static void _wsb_w2b(u8 *dst, u32 src)
{
    dst[3] = src & 0xFF;
    src >>= 8;
    dst[2] = src & 0xFF;
    src >>= 8;
    dst[1] = src & 0xFF;
    src >>= 8;
    dst[0] = src & 0xFF;
}

void swapcpy_b2w(u32 *dst, const u8 *src, u32 wordlen)
{
    int i;

    for (i = 0; i < (int)wordlen; i++) {
        dst[i] = _wsb_b2w((u8 *) src);
        src += 4;
    }
}


void swapcpy_w2b(u8 *dst, const u32 *src, u32 wordlen)
{
    int i;

    for (i = 0; i < (int)wordlen; i++) {
        _wsb_w2b(dst, src[i]);
        dst += 4;
    }
}

void swapcpy_b2b(u8 *dst, u8 *src, u32 wordlen)
{
    u8 tmp;
    int i;

    for (i = 0; i < (int)wordlen; i++) {
        tmp = src[0];
        dst[0] = src[3];
        dst[3] = tmp;

        tmp = src[1];
        dst[1] = src[2];
        dst[2] = tmp;

        dst += 4;
        src += 4;
    }
}

int ucl_sha256_init(ucl_sha256_ctx_t *ctx)
{
    if (ctx == NULL) {
        return UCL_INVALID_INPUT;
    }

    ctx->state[0] = 0x6A09E667;
    ctx->state[1] = 0xBB67AE85;
    ctx->state[2] = 0x3C6EF372;
    ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f;
    ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab;
    ctx->state[7] = 0x5be0cd19;

    ctx->count[0] = 0;
    ctx->count[1] = 0;

    return UCL_OK;
}

int ucl_sha256_core(ucl_sha256_ctx_t *ctx, u8 *data, u32 dataLen)
{
    u32 indexh, partLen, i;
    if (ctx == NULL) {
        return UCL_INVALID_INPUT;
    }

    if ((data == NULL)  || (dataLen == 0)) {
        return UCL_NOP;
    }

    /** Compute number of bytes mod 64 */
    indexh = (u32)((ctx->count[1] >> 3) & 0x3F);

    /** Update number of bits */
    if ((ctx->count[1] += ((u32)dataLen << 3)) < ((u32)dataLen << 3)) {
        ctx->count[0]++;
    }

    ctx->count[0] += ((u32)dataLen >> 29);

    partLen = 64 - indexh;

    /** Process 512-bits block as many times as possible. */

    if (dataLen >= partLen) {
        memcpy(&ctx->buffer[indexh], data, partLen);

        swapcpy_b2b(ctx->buffer, ctx->buffer, 16);

        sha256_stone(ctx->state, (u32 *) ctx->buffer);

        for (i = partLen; i + 63 < dataLen; i += 64) {
            swapcpy_b2b(ctx->buffer, &data[i], 16);

            sha256_stone(ctx->state, (u32 *) ctx->buffer);
        }

        indexh = 0;
    } else {
        i = 0;
    }

    /** Buffer remaining data */
    memcpy(&ctx->buffer[indexh], &data[i], dataLen - i);

    return UCL_OK;
}


int ucl_sha256_finish(u8 *hash, ucl_sha256_ctx_t *ctx)
{
    u8 bits[8];
    u32 indexh, padLen;
    u8 padding[64];

    padding[0] = 0x80;

    memset(padding + 1, 0, 63);

    if (hash == NULL) {
        return UCL_INVALID_OUTPUT;
    }

    if (ctx == NULL) {
        return UCL_INVALID_INPUT;
    }

    /** Save number of bits */
    swapcpy_w2b(bits, ctx->count, 2);

    /** Pad out to 56 mod 64. */
    indexh = (u32)((ctx->count[1] >> 3) & 0x3f);

    padLen = (indexh < 56) ? (56 - indexh) : (120 - indexh);

    ucl_sha256_core(ctx, padding, padLen);

    /** Append length (before padding) */
    ucl_sha256_core(ctx, bits, 8);

    /** Store state in digest */
    swapcpy_w2b(hash, ctx->state, 8);

    /** Zeroize sensitive information. */
    memset(ctx, 0, sizeof(*ctx));

    return UCL_OK;
}

int ucl_sha256(u8 *hash, u8 *message, u32 byteLength)
{
    ucl_sha256_ctx_t ctx;

    if (hash == NULL) {
        return UCL_INVALID_OUTPUT;
    }

    ucl_sha256_init(&ctx);
    ucl_sha256_core(&ctx, message, byteLength);
    ucl_sha256_finish(hash, &ctx);

    return UCL_OK;
}

#endif // HASH_SHA256
