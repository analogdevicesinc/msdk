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
#ifndef EXAMPLES_MAX78002_QSPI_APS6404_H_
#define EXAMPLES_MAX78002_QSPI_APS6404_H_

#include <stdint.h>

#define MFID_EXPECTED 0x0D
#define KGD_EXPECTED 0x5D
#define DENSITY_EXPECTED 0b010

typedef struct {
    uint8_t MFID;
    uint8_t KGD;
    uint8_t density;
    int EID;
} ram_id_t;

int ram_init();

int ram_reset();

int ram_enter_quadmode();

int ram_exit_quadmode();

int ram_read_id(ram_id_t *out);

int ram_read_slow(uint32_t address, uint8_t *out, unsigned int len);

int ram_read_quad(uint32_t address, uint8_t *out, unsigned int len);

int ram_write(uint32_t address, uint8_t *data, unsigned int len);

int ram_write_quad(uint32_t address, uint8_t *data, unsigned int len);

int benchmark_dma_overhead(unsigned int *out);

#endif // EXAMPLES_MAX78002_QSPI_APS6404_H_
