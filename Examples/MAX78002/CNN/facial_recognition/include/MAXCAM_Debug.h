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
#ifndef MAXCAM_DEBUG_H_
#define MAXCAM_DEBUG_H_

#include <stdio.h>
#include <string.h>

//#define IMAGE_TO_UART

#ifndef IMAGE_TO_UART
#define PR_DEBUG(fmt, args...) \
    if (0)                     \
    printf("D[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#define PR_INFO(fmt, args...) \
    if (0)                    \
    printf("I[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#define PR_WARN(fmt, args...) \
    if (1)                    \
    printf("W[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#define PR_ERR(fmt, args...) \
    if (1)                   \
    printf("E[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#else
#define PR_DEBUG(fmt, args...)
#define PR_INFO(fmt, args...)
#define PR_WARN(fmt, args...)
#define PR_ERR(fmt, args...)
#endif
#endif /* MAXCAM_DEBUG_H_ */
