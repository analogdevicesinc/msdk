/******************************************************************************* 
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
* * This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*     Module Name: SHA512
*     Description: performs SHA512 operations
*        Filename: ucl_sha512.c
*          Author: GR
*        Compiler: gcc
*
*******************************************************************************/
#include <ucl/ucl_hash.h>
#ifdef HASH_SHA512
#include <string.h>

#include "ucl/ucl_sha512.h"
#include <ucl/ucl_sys.h>
#include <ucl/ucl_retdefs.h>
#include "ucl/bignum_ecdsa_generic_api.h"


void _wsb_ll2b(u8 *dst, u64 src)
{
  dst[7] = src & 0xFF;
  src >>= 8;
  dst[6] = src & 0xFF;
  src >>= 8;
  dst[5] = src & 0xFF;
  src >>= 8;
  dst[4] = src & 0xFF;
  src >>= 8;
  dst[3] =src & 0xFF;
  src >>= 8;
  dst[2] = src & 0xFF;
  src >>= 8;
  dst[1] = src & 0xFF;
  src >>= 8;
  dst[0] = src & 0xFF;
}

void swapcpy_ll2b(u8 *dst, const u64 *src, u32 wordlen)
{
    int i;

    for (i = 0 ; i < (int)wordlen ; i++)
    {
        _wsb_ll2b(dst, src[i]);
        dst += 8;
    }
}

void swapcpy_b2b64(u8 *dst, u8 *src, u32 wordlen)
{
    u8 tmp;
    int i;

    for (i = 0 ; i < (int)wordlen ; i++)
    {
        tmp = src[0];
        dst[0] = src[7];
        dst[7] = tmp;
        tmp = src[1];
        dst[1] = src[6];
        dst[6] = tmp;
        tmp = src[2];
        dst[2] = src[5];
        dst[5] = tmp;
        tmp = src[3];
        dst[3] = src[4];
        dst[4] = tmp;

        dst += 8;
        src += 8;
    }
}

int ucl_sha512_init(ucl_sha512_ctx_t *ctx)
{
    if (ctx == NULL)
        return UCL_INVALID_INPUT;
    ctx->state[0] = 0x6a09e667f3bcc908ULL;
    ctx->state[1] = 0xbb67ae8584caa73bULL;
    ctx->state[2] = 0x3c6ef372fe94f82bULL;
    ctx->state[3] = 0xa54ff53a5f1d36f1ULL;
    ctx->state[4] = 0x510e527fade682d1ULL;
    ctx->state[5] = 0x9b05688c2b3e6c1fULL;
    ctx->state[6] = 0x1f83d9abfb41bd6bULL;
    ctx->state[7] = 0x5be0cd19137e2179ULL;
    //used to accumulate the size of data
    ctx->count[0] = 0;
    ctx->count[1] = 0;
    return UCL_OK;
}

int ucl_sha512_core(ucl_sha512_ctx_t *ctx, u8 *data, u32 dataLen)
{
    u32 indexh, partLen, i;
    if (ctx == NULL)
        return UCL_INVALID_INPUT;
    if ((data == NULL)  || (dataLen == 0))
        return UCL_NOP;
    indexh = (u32)((ctx->count[1] >> 3) & 0x7F);
    ctx->count[1]+=(u64)(dataLen << 3);
    ctx->count[0] += ((u64)dataLen >> 29);
    partLen = 128 - indexh;
    if (dataLen >= partLen)
      {
        memcpy(&ctx->buffer[indexh], data, partLen);
	swapcpy_b2b64(ctx->buffer, ctx->buffer, 16);
        sha512_stone(ctx->state, (u64 *) ctx->buffer);
        for (i = partLen; i + 127 < dataLen; i += 128)
	  {
            swapcpy_b2b64(ctx->buffer, &data[i], 16);
            sha512_stone(ctx->state, (u64 *) ctx->buffer);
	  }
        indexh = 0;
      }
    else
      {
        i = 0;
      }
    memcpy(&ctx->buffer[indexh], &data[i], dataLen - i);
    return UCL_OK;
}

int ucl_sha512_finish(u8 *hash, ucl_sha512_ctx_t *ctx)
{
    u8 bits[16];
    u64 indexh, padLen;
    u8 padding[128];
    padding[0] = 0x80;
    memset(padding + 1, 0, 127);
    if (hash == NULL)
        return UCL_INVALID_OUTPUT;
    if (ctx == NULL)
        return UCL_INVALID_INPUT;
    swapcpy_ll2b(bits, ctx->count, 2);
    indexh = (u32)((ctx->count[1] >> 3) & 0x7f);
    padLen = (indexh < 112) ? (112 - indexh) : (240 - indexh);
    ucl_sha512_core(ctx, padding, (u32)padLen);
    ucl_sha512_core(ctx, bits, 16);
    swapcpy_ll2b(hash, ctx->state, 8);
    return UCL_OK;
}
int ucl_sha512(u8 *hash, u8 *message, u32 byteLength)
{
    ucl_sha512_ctx_t ctx;
    if (hash == NULL)
        return UCL_INVALID_OUTPUT;
    ucl_sha512_init(&ctx);
    ucl_sha512_core(&ctx, message, byteLength);
    ucl_sha512_finish(hash, &ctx);
    return UCL_OK;
}
#endif//HASH_SHA512
