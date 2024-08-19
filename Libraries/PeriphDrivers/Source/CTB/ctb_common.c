/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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
