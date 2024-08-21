/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

#ifndef __CAMERA_UTIL_H__
#define __CAMERA_UTIL_H__

#define CAMERA_FREQ (10 * 1000 * 1000)
#define BYTE_PER_PIXEL 2

#if defined(CAMERA_OV7692)
#define IMAGE_XRES 240
#define IMAGE_YRES 240
#endif

#define X_START 0
#define Y_START 0

#define TFT_W 320
#define TFT_H 240

int initialize_camera(void);
void load_input_camera(int);
void capture_and_display_camera(void);

#endif
