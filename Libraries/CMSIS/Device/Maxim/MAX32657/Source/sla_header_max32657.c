/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

extern uint32_t _application_end;

#define SLA_HEADER_MAGIC0 0xBF1421E4
#define SLA_HEADER_MAGIC1 0x461A8CF5
#define SLA_HEADER_VERSION 0x00000001
#define SLA_HEADER_ALGORITHM_ECDSA 0x516A0001
#define SLA_HEADER_RESERVED 0x00000000

typedef struct {
    uint32_t magic0;
    uint32_t magic1;
    uint32_t version;
    uint32_t verifytype;
    uint32_t sigaddress;
    uint32_t reserved5;
    uint32_t reserved6;
    uint32_t reserved7;
} flash_app_header_t;

__attribute__((section(".sla_header"))) __attribute__((__used__))
const flash_app_header_t sla_header = { .magic0 = SLA_HEADER_MAGIC0,
                                        .magic1 = SLA_HEADER_MAGIC1,
                                        .version = SLA_HEADER_VERSION,
                                        .verifytype = SLA_HEADER_ALGORITHM_ECDSA,
                                        .sigaddress = (uint32_t)&_application_end,
                                        .reserved5 = SLA_HEADER_RESERVED,
                                        .reserved6 = SLA_HEADER_RESERVED,
                                        .reserved7 = SLA_HEADER_RESERVED };
