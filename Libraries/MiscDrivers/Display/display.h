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

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_DISPLAY_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_DISPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DISP_E_SUCCESS 0
#define DISP_E_ERROR -1
#define DISP_E_BUSY -99
#define DISP_E_BAD_STATE -98
#define DISP_E_TIME_OUT -97
#define DISP_E_BAD_PARAM -96
#define DISP_E_NOT_CONFIGURED -95

typedef struct {
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
} display_area_t;

typedef enum {
    DISPLAY_NORMAL,
    DISPLAY_FLIP,
    DISPLAY_ROTATE,
} display_rotation_t;

typedef struct {
    int (*init)(void);
    int (*write)(uint8_t *data, uint32_t data_len);
    uint8_t *comm_buffer;
    uint32_t comm_buffer_len;
} display_comm_api;

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_DISPLAY_DISPLAY_H_
