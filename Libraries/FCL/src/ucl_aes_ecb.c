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

#include <ucl/ucl_types.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_aes.h>
#include <ucl/ucl_aes_ecb.h>
#include <string.h>

int ucl_aes_ecb(u8 *dst, u8 *src, u32 len, u8 *key, u32 keylen, int mode)
{
    ucl_aes_ctx_t ctx;
    int ret;

    if ((src == NULL) || (key == NULL)) {
        return UCL_INVALID_INPUT;
    }

    if ((dst == NULL)) {
        return UCL_INVALID_OUTPUT;
    }

    if ((len % UCL_AES_BLOCKSIZE) != 0) {
        return UCL_INVALID_ARG;
    }

    if ((mode != UCL_CIPHER_DECRYPT) && (mode != UCL_CIPHER_ENCRYPT)) {
        return UCL_INVALID_MODE;
    }

    if ((keylen != UCL_AES_KEYLEN_128) && (keylen != UCL_AES_KEYLEN_192) && (keylen != UCL_AES_KEYLEN_256)) {
        return UCL_INVALID_ARG;
    }

    ucl_aes_ecb_init(&ctx, key, keylen, mode);
    ucl_aes_ecb_core(dst, &ctx, src, len);
    ucl_aes_ecb_finish(&ctx);

    return UCL_OK;
}

int ucl_aes_ecb_init(ucl_aes_ctx_t *ctx, u8 *key, u32 keylen, int mode)
{
    int i;
    if (ctx == NULL) {
        return UCL_INVALID_OUTPUT;
    }

    if (key == NULL) {
        return UCL_INVALID_INPUT;
    }

    if ((keylen != UCL_AES_KEYLEN_128) && (keylen != UCL_AES_KEYLEN_192) && (keylen != UCL_AES_KEYLEN_256)) {
        return UCL_INVALID_ARG;
    }

    ctx->mode = mode;
    for (i = 0; i < (int)keylen; i++) {
        ctx->origin_key[i] = key[i];
    }

    ctx->origin_keylen = (int)keylen;

    if (mode == UCL_CIPHER_ENCRYPT) {
        aes_set_ekey(&ctx->key, key, keylen);
    } else if (mode == UCL_CIPHER_DECRYPT) {
        aes_set_dkey(&ctx->key, key, keylen);
    } else {
        return UCL_INVALID_MODE;
    }

    return UCL_OK;
}

int ucl_aes_ecb_core(u8 *dst, ucl_aes_ctx_t *ctx, u8 *src, u32 len)
{
    u8 *data_end;
    int ret;

    if ((ctx == NULL) || (dst == NULL)) {
        return UCL_INVALID_OUTPUT;
    }

    if (src == NULL) {
        return UCL_INVALID_INPUT;
    }

    if ((len % UCL_AES_BLOCKSIZE) != 0) {
        return UCL_INVALID_ARG;
    }

    data_end = len + src;

    if (ctx->mode == UCL_CIPHER_ENCRYPT) {
        while (src != data_end) {
            aes_encrypt(dst, src, &ctx->key);
            src += UCL_AES_BLOCKSIZE;
            dst += UCL_AES_BLOCKSIZE;
        }
    } else if (ctx->mode == UCL_CIPHER_DECRYPT) {
        while (src != data_end) {
            aes_decrypt(dst, src, &ctx->key);
            src += UCL_AES_BLOCKSIZE;
            dst += UCL_AES_BLOCKSIZE;
        }
    }

    return UCL_OK;
}

int ucl_aes_ecb_finish(ucl_aes_ctx_t *ctx)
{
    if (ctx == NULL) {
        return UCL_INVALID_OUTPUT;
    }

    memset((unsigned char *)ctx, 0, sizeof(*ctx));

    return UCL_OK;
}
