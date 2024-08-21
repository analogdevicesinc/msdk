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

#ifndef EXAMPLES_MAX32690_SCPA_OTP_DUMP_UTILS_H_
#define EXAMPLES_MAX32690_SCPA_OTP_DUMP_UTILS_H_

void print_str(const char *str);
void print_hex(int value);
void print_decimal(int value);
//
void utils_byteArr2str(unsigned char *dst, unsigned char *arr, int numberOfItem, int itemsize,
                       const char *prefix, const char *postfix);

#endif // EXAMPLES_MAX32690_SCPA_OTP_DUMP_UTILS_H_
