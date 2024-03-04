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

#ifndef _FACEID_H_
#define _FACEID_H_

#define USE_BOX_ONLY
//#define UNNORMALIZE_RECORD // Do not normalize the recorded embeddings

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

#define CAPTURE_X 100
#define CAPTURE_Y 300

#define HEIGHT_ID 112
#define WIDTH_ID 112
#define THICKNESS 1 //4

#define FRAME_ORANGE 0xFD20

#define MAX_X_OFFSET 23 //(WIDTH_DET - WIDTH)/2 // 24 pixels
#define MAX_Y_OFFSET 31 //(HEIGHT_DET - HEIGHT)/2 // 32 pixels

// Data input: HWC (little data): 160x120x3
#define DATA_SIZE_IN_ID (112 * 112 * 3)

int face_id(void);

#endif // _FACEID_H_
