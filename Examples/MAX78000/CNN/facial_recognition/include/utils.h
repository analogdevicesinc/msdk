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

#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdint.h>
#include "uart.h"

/*****************************     MACROS    *********************************/

/*****************************     VARIABLES *********************************/

/*****************************     FUNCTIONS *********************************/

uint32_t utils_get_time_ms(void);

void utils_send_bytes(mxc_uart_regs_t *uart, uint8_t *ptr, int length);

void utils_delay_ms(uint32_t ms);
void utils_hexDump(const char *title, uint8_t *buf, uint32_t len);
int utils_send_img_to_pc(uint8_t *img, uint32_t imgLen, int w, int h, uint8_t *pixelformat);

#endif // _UTILS_H_
