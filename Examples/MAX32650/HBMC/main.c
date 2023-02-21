/**
 * @file
 * @brief   Hyperbus RAM example
 * @details Demonstrates the ability of Hyperbus/Xccella RAM to be used for data and code
 *
 */

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

/* **** Includes **** */
#include <stdio.h>
#include <string.h>

#include "mxc_errors.h"
#include "board.h"
#include "led.h"
#include "hpb.h"

/* **** Definitions **** */

/* Note: Define only one of these! */
#define HYPERRAM /* Hyperbus RAM  */
//#define XCCELABUS /* Xccelabus PSRAM */

/* Valid values for chip select are 0 and 1 */
#define HPB_CS 0

#if defined(XCCELABUS) && defined(HYPERRAM)
#error Please define only ONE of the two test options
#endif

/* **** Globals **** */

/* Linker-defined symbols */
#if defined(__GNUC__)
extern uint8_t __load_start_hpb_cs0;
extern uint8_t __load_length_hpb_cs0;
extern uint8_t __load_start_hpb_cs1;
extern uint8_t __load_length_hpb_cs1;
extern uint8_t __hpb_cs0_start;
extern uint8_t __hpb_cs1_start;
#elif defined(__CC_ARM)
/* Note: This demo has not been tested under IAR and should be considered non-functional */
extern int Image$$RW_IRAM2$$Length;
extern char Image$$RW_IRAM2$$Base[];
uint8_t *__hpb_addr;
#endif

/* **** Functions **** */
void debug_hpb(void)
{
    printf("DEBUG: HPB status: 0x%08x\tHPB flags: 0x%08x\n", MXC_HPB->status, MXC_HPB->intfl);
}

/* ************************************************************************** */
int init_xcella_psram(int cs)
{
    mxc_hpb_mem_config_t mem;
    mxc_hpb_cfg_reg_val_t cfg_reg[1];
    int result;

    /* Xcella configuration for fixed latency mode (see datasheet) */
    cfg_reg[0].addr = 0x0000;
    cfg_reg[0].val = 0x29;

    /* Hyperbus/Xccelabus controller configuration */
    if (cs) {
        mem.base_addr = (unsigned int)&__hpb_cs1_start;
    } else {
        mem.base_addr = (unsigned int)&__hpb_cs0_start;
    }
    mem.device_type = MXC_HPB_DEV_XCCELA_PSRAM;
    mem.cfg_reg_val = cfg_reg;
    mem.cfg_reg_val_len = 1;
    mem.read_cs_high = MXC_HPB_CS_HIGH_2_5;
    mem.write_cs_high = MXC_HPB_CS_HIGH_2_5;
    mem.read_cs_setup = MXC_HPB_CS_SETUP_HOLD_2;
    mem.write_cs_setup = MXC_HPB_CS_SETUP_HOLD_2;
    mem.read_cs_hold = MXC_HPB_CS_SETUP_HOLD_2;
    mem.write_cs_hold = MXC_HPB_CS_SETUP_HOLD_2;
    mem.latency_cycle = MXC_V_HPB_MTR_LATENCY_5CLK;
    mem.fixed_latency = 1;

    if (cs) {
        /* Xccela PSRAM on chip select 1 */
        result = MXC_HPB_Init(NULL, &mem);
    } else {
        /* Xccela PSRAM on chip select 0 */
        result = MXC_HPB_Init(&mem, NULL);
    }

    return result;
}

/* ************************************************************************** */
int init_hyperbus_ram(int cs)
{
    mxc_hpb_mem_config_t mem;
    mxc_hpb_cfg_reg_val_t cfg_reg[1];
    int result;

    /* Hyperbus RAM chip configuration */
    cfg_reg[0].addr = 0x01000;
    cfg_reg[0].val = 0x801f;

    /* Hyperbus/Xccelabus controller configuration */
    if (cs) {
        mem.base_addr = (unsigned int)&__hpb_cs1_start;
    } else {
        mem.base_addr = (unsigned int)&__hpb_cs0_start;
    }
    mem.device_type = MXC_HPB_DEV_HYPER_RAM;
    mem.cfg_reg_val = cfg_reg;
    mem.cfg_reg_val_len = 1;
    mem.read_cs_high = MXC_HPB_CS_HIGH_10_5;
    mem.write_cs_high = MXC_HPB_CS_HIGH_10_5;
    mem.read_cs_setup = MXC_HPB_CS_SETUP_HOLD_16;
    mem.write_cs_setup = MXC_HPB_CS_SETUP_HOLD_14;
    mem.read_cs_hold = MXC_HPB_CS_SETUP_HOLD_5;
    mem.write_cs_hold = MXC_HPB_CS_SETUP_HOLD_12;
    mem.latency_cycle = MXC_V_HPB_MTR_LATENCY_6CLK;
    mem.fixed_latency = 0;

    if (cs) {
        /* Xccela PSRAM on chip select 1 */
        result = MXC_HPB_Init(NULL, &mem);
    } else {
        /* Xccela PSRAM on chip select 0 */
        result = MXC_HPB_Init(&mem, NULL);
    }

    return result;
}

/* ************************************************************************** */
int main(void)
{
    void (*func)(void);
    int x;
    void *int_flash_src, *ext_flash_dest;
    uint32_t int_flash_len;

    printf("\n\n***** Hyperbus/Xccelabus Controller Example *****\n");

#if defined(HYPERRAM)
    printf("Hyperbus RAM on CS%u selected.\n", HPB_CS);
    x = init_hyperbus_ram(HPB_CS);
#else
    printf("Xccelabus PSRAM on CS%u selected.\n", HPB_CS);
    x = init_xcella_psram(HPB_CS);
#endif

    if (x != E_NO_ERROR) {
        printf("Failed to configure external memory: %d\n", x);
    }

    if (HPB_CS == 0) {
        int_flash_src = &__load_start_hpb_cs0;
        int_flash_len = (uint32_t)&__load_length_hpb_cs0;
        ext_flash_dest = &__hpb_cs0_start;
    } else {
        int_flash_src = &__load_start_hpb_cs1;
        int_flash_len = (uint32_t)&__load_length_hpb_cs1;
        ext_flash_dest = &__hpb_cs1_start;
    }

    /* Copy function from flash to external RAM */
    printf("Copying function from internal flash @ 0x%08x to external RAM @ 0x%08x (%u bytes)\n",
           int_flash_src, ext_flash_dest, int_flash_len);
    memcpy(ext_flash_dest, int_flash_src, int_flash_len);

    /* Demonstrate reads */
    printf("Flash source:\n");
    for (x = 0; x < int_flash_len; x++) {
        printf("0x%02x ", *((uint8_t *)(int_flash_src + x)));
        if ((x & 0xf) == 0xf) {
            printf("\n");
        }
    }

    printf("\n");
    printf("HPB source:\n");
    for (x = 0; x < int_flash_len; x++) {
        printf("0x%02x ", *((uint8_t *)(ext_flash_dest + x)));
        if ((x & 0xf) == 0xf) {
            printf("\n");
        }
    }
    printf("\n");

    if (!memcmp(ext_flash_dest, int_flash_src, int_flash_len)) {
        func = (void (*)(void))((uint32_t)ext_flash_dest | 0x1);
        printf("Jumping to external flash (@ 0x%08x), watch for blinking LED.\n", func);
        func();
        printf("Returned from external flash\n");
    } else {
        printf("MEMORY MIS-COMPARE!\n");
        return E_FAIL;
    }

    printf("\nEND OF LINE.\n");

    return E_NO_ERROR;
}
