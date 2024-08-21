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

#ifndef _FACEDETECTION_H_
#define _FACEDETECTION_H_

// Feather board default orientation: horizontal, camera at the top
#define ROTATE_FEATHER_BOARD // rotate 180 degree, camera at the bottom

#define CAMERA_FREQ (10 * 1000 * 1000)

#define TFT_WIDTH 320
#define TFT_HEIGHT 240
#define IMAGE_XRES 224
#define IMAGE_YRES 168

#define IMAGE_SIZE_X 168
#define IMAGE_SIZE_Y 224

#define HEIGHT_DET 224
#define WIDTH_DET 168

#define IMAGE_H 168
#define IMAGE_W 224

#define X_START (TFT_HEIGHT - IMAGE_YRES) / 2
#define Y_START (TFT_HEIGHT - IMAGE_YRES) / 2

#define FRAME_ORANGE 0xFD20
#define FRAME_BLUE 0x001F

#define BYTE_PER_PIXEL 2

#define LOW_LIGHT_THRESHOLD 20

// Data input: HWC (little data): 160x120x3
#define DATA_SIZE_IN_DET (224 * 168 * 3)

int face_detection(void);

#endif // _FACEDETECTION_H_
