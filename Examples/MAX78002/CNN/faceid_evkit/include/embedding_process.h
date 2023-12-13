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

#ifndef _EMBEDDINGS_PROCESS_H_
#define _EMBEDDINGS_PROCESS_H_

#include <stdint.h>

/*****************************     MACROS    *********************************/

#define thresh_for_unknown_subject 9993
//#define thresh_for_unknown_subject 313600
#define closest_sub_buffer_size 3 * 7

/*****************************     VARIABLES *********************************/

typedef struct __attribute__((packed)) sMeanDistance {
    uint8_t subID;
    uint8_t number;
    int32_t distance;
} tsMeanDistance;

typedef struct __attribute__((packed)) sMinDistance {
    uint8_t subID;
    int32_t distance;
} tsMinDistance;

/*****************************     FUNCTIONS *********************************/

int init_database(void);

char *get_subject(int ID);

int calculate_minDistance(const uint8_t *embedding);

void get_min_dist_counter(uint8_t **counter, uint8_t *counter_len);
tsMinDistance *get_min_distance();

#endif // _EMBEDDINGS_PROCESS_H_
