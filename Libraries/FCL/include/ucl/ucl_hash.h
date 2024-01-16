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
//these defines are used to select or not hash functions
//useful on platforms with limited resources

#ifndef LIBRARIES_FCL_INCLUDE_UCL_UCL_HASH_H_
#define LIBRARIES_FCL_INCLUDE_UCL_UCL_HASH_H_

#define HASH_SHA256
#define HASH_SHA384
#define HASH_SHA512
#define HASH_SIA256
#define HASH_SHA3

#define MAX_HASH_FUNCTIONS 5

extern int hash_size[MAX_HASH_FUNCTIONS];

#endif // LIBRARIES_FCL_INCLUDE_UCL_UCL_HASH_H_
