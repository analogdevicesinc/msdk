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

#ifndef _FACEID_H_
#define _FACEID_H_

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

#define CAPTURE_X 50
#define CAPTURE_Y 290

#define HEIGHT_ID 112
#define WIDTH_ID 112
#define THICKNESS 1 //4

#define MAX_X_OFFSET 23 //(IMAGE_H - WIDTH)/2 // 24 pixels
#define MAX_Y_OFFSET 31 //(IMAGE_W - HEIGHT)/2 // 32 pixels

// Data input: HWC (little data): 160x120x3
#define DATA_SIZE_IN_ID (112 * 112 * 3)

int face_id(void);

#endif // _FACEID_H_
