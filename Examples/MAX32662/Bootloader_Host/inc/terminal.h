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

#ifndef EXAMPLES_MAX32662_BOOTLOADER_HOST_INC_TERMINAL_H_
#define EXAMPLES_MAX32662_BOOTLOADER_HOST_INC_TERMINAL_H_

/*******************************      INCLUDES    ****************************/

/*******************************      DEFINES     ****************************/
#define KEY_ESC -1
#define KEY_CANCEL -1

/******************************* Type Definitions ****************************/
typedef struct {
    const char *name;
    int (*callback)(const char *parentName);
} list_t;

/******************************* Public Functions ****************************/
int terminal_init(void);
int terminal_printf(const char *format, ...);
void terminal_hexdump(const char *title, char *buf, unsigned int len);
int terminal_read_num(unsigned int timeout);
int terminal_select_from_list(const char *title, const list_t *items, int nb_items, int nb_col);

#endif // EXAMPLES_MAX32662_BOOTLOADER_HOST_INC_TERMINAL_H_
