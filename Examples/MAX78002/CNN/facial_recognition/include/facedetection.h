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

#ifndef _FACEDETECTION_H_
#define _FACEDETECTION_H_

#define CAMERA_FREQ (10 * 1000 * 1000)
//#define LP_MODE_ENABLE
#define LP_MODE 4 // 0:NO SLEEP, 1:SLEEP, 2:LPM, 3:UPM, 4:STANDBY, 5:BACKUP, 6:POWERDOWN

#define TFT_WIDTH 320
#define TFT_HEIGHT 240

#define HEIGHT_DET 224
#define WIDTH_DET 168

#define X_START (TFT_HEIGHT - WIDTH_DET) / 2
#define Y_START 30

#define FRAME_BLUE 0x001F

#define BYTE_PER_PIXEL 2

#define LOW_LIGHT_THRESHOLD 0

// Data input: HWC (little data): 160x120x3
#define DATA_SIZE_IN_DET (224 * 168 * 3)

int face_detection(void);

#endif // _FACEDETECTION_H_
