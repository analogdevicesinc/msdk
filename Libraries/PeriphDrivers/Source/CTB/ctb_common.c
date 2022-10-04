/* ****************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#include "ctb.h"
#include "ctb_common.h"

/* ************************************************************************* */

unsigned int MXC_CTB_Common_Hash_GetBlockSize(mxc_ctb_hash_func_t function)
{
    // Block size in bytes indexed by hash function
    switch (function) {
    case MXC_CTB_HASH_DIS:
        return 0;

    case MXC_CTB_HASH_SHA1:
        return 64;

    case MXC_CTB_HASH_SHA224:
        return 64;

    case MXC_CTB_HASH_SHA256:
        return 64;

    case MXC_CTB_HASH_SHA384:
        return 128;

    case MXC_CTB_HASH_SHA512:
        return 128;
    }

    // if returns this bad param was passed in or disable.
    return 0;
}

unsigned int MXC_CTB_Common_Hash_GetDigestSize(mxc_ctb_hash_func_t function)
{
    // Digest length in bytes indexed by hash function
    switch (function) {
    case MXC_CTB_HASH_DIS:
        return 0;

    case MXC_CTB_HASH_SHA1:
        return 20;

    case MXC_CTB_HASH_SHA224:
        return 28;

    case MXC_CTB_HASH_SHA256:
        return 32;

    case MXC_CTB_HASH_SHA384:
        return 48;

    case MXC_CTB_HASH_SHA512:
        return 64;
    }

    // if returns this bad param was passed in or disable.
    return 0;
}

unsigned int MXC_CTB_Common_Cipher_GetKeySize(mxc_ctb_cipher_t cipher)
{
    switch (cipher) {
    case MXC_CTB_CIPHER_DIS:
        return 0;

    case MXC_CTB_CIPHER_AES128:
        return 16;

    case MXC_CTB_CIPHER_AES192:
        return 24;

    case MXC_CTB_CIPHER_AES256:
        return 32;

    case MXC_CTB_CIPHER_DES:
        return 8;

    case MXC_CTB_CIPHER_TDES:
        return 24;
    }

    // if returns this bad param was passed in or disable.
    return 0;
}

unsigned int MXC_CTB_Common_Cipher_GetBlockSize(mxc_ctb_cipher_t cipher)
{
    switch (cipher) {
    case MXC_CTB_CIPHER_DIS:
        return 0;

    case MXC_CTB_CIPHER_AES128:
        return 16;

    case MXC_CTB_CIPHER_AES192:
        return 16;

    case MXC_CTB_CIPHER_AES256:
        return 16;

    case MXC_CTB_CIPHER_DES:
        return 8;

    case MXC_CTB_CIPHER_TDES:
        return 8;
    }

    // if returns this bad param was passed in or disable.
    return 0;
}
