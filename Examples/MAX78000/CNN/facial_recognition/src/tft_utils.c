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
#include <stdlib.h>
#include <stdint.h>
#include "mxc.h"
#include "mxc_device.h"
#include "tft_utils.h"
#include "facedetection.h"

void draw_obj_rect(float *xy, uint32_t w, uint32_t h)
{
#ifdef TFT_ENABLE
    int x1 = w * xy[0] + X_START;
    int y1 = h * xy[1] + Y_START;
    int x2 = w * xy[2] + X_START - 1;
    int y2 = h * xy[3] + Y_START - 1;

    // Draw rectangle around detected face
    //Rotated here because the way DMA Stream works
    MXC_TFT_Rectangle(y1, x1, y2, x2, FRAME_ORANGE);
    printf("x1: %d, y1: %d, x2: %d, y2: %d\n", x1, y1, x2, y2);
#endif
}
