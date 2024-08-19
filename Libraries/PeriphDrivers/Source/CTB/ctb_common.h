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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CTB_CTB_COMMON_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CTB_CTB_COMMON_H_

#include "ctb.h"

unsigned int MXC_CTB_Common_Hash_GetBlockSize(mxc_ctb_hash_func_t function);
unsigned int MXC_CTB_Common_Hash_GetDigestSize(mxc_ctb_hash_func_t function);
unsigned int MXC_CTB_Common_Cipher_GetKeySize(mxc_ctb_cipher_t cipher);
unsigned int MXC_CTB_Common_Cipher_GetBlockSize(mxc_ctb_cipher_t cipher);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CTB_CTB_COMMON_H_
