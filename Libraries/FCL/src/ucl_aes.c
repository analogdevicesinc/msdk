/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

int ucl_aes(u8 *dst, u8 *src, u8 *key, u32 keylen, int mode)
{
    int resu;
    AES_KEY aeskey;

    if (mode == UCL_CIPHER_ENCRYPT) {
        aes_set_ekey(&aeskey, key, keylen);

        return(aes_encrypt(dst, src, &aeskey));
    } else if (mode == UCL_CIPHER_DECRYPT) {
        aes_set_dkey(&aeskey, key, keylen);

        return(aes_decrypt(dst, src, &aeskey));
    } else {
        return(UCL_INVALID_ARG);
    }

    return UCL_OK;
}
