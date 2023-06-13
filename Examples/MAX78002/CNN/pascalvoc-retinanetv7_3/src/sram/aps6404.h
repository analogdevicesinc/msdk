/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#ifndef APS6404_H
#define APS6404_H

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

#endif