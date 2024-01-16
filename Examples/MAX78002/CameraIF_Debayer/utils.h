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
#ifndef EXAMPLES_MAX78002_CAMERAIF_DEBAYER_UTILS_H_
#define EXAMPLES_MAX78002_CAMERAIF_DEBAYER_UTILS_H_

/*****************************     INCLUDES  *********************************/
#include <stdint.h>

/*****************************     MACROS    *********************************/

/*****************************     VARIABLES *********************************/

/*****************************     FUNCTIONS *********************************/
void utils_hexDump(const char *title, uint8_t *buf, uint32_t len);
int utils_send_img_to_pc(uint8_t *img, uint32_t imgLen, int w, int h, uint8_t *pixelformat);
int utils_stream_img_to_pc_init(uint8_t *img, uint32_t imgLen, int w, int h, uint8_t *pixelformat);
int utils_stream_image_row_to_pc(uint8_t *img, uint32_t imgRowLen);

#endif // EXAMPLES_MAX78002_CAMERAIF_DEBAYER_UTILS_H_
