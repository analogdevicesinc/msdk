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
MEMORY {
    ROM (rx)   : ORIGIN = 0x00000000, LENGTH = 0x00020000 /* 128kB ROM */
    FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 0x00300000 /* 1MB flash */
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x000BE000 /* 760kB SRAM */

    XIP_0 (rx)   : ORIGIN = 0x08000000, LENGTH = 0x00010000
    XIP_1 (rx)   : ORIGIN = 0x08010000, LENGTH = 0x00010000
    XIP_2 (rx)   : ORIGIN = 0x08020000, LENGTH = 0x00010000
    XIP_SLEEP (rx)   : ORIGIN = 0x08030000, LENGTH = 0x00010000    
}

SECTIONS {
    .rom :
    {
        KEEP(*(.rom_vector))
        *(.rom_handlers*)
    } > ROM

    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)    /* program code */
        *(.rodata*)  /* read-only data: "const" */

        KEEP(*(.init))
        KEEP(*(.fini))

        /* C++ Exception handling */
        KEEP(*(.eh_frame*))
        _etext = .;
    } > FLASH

    /* This section allows code to be compiled/linked for XIP addresses, but reside in
     * flash until copied by code to the external XIP flash device
     */
    .xip_section :
    {
        KEEP(*(.xip_section*))
    } > FLASH

    __load_start_xip = LOADADDR(.xip_section);
    __load_length_xip = SIZEOF(.xip_section);

    /* it's used for C++ exception handling      */
    /* we need to keep this to avoid overlapping */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    .data :
    {
        _data = ALIGN(., 4);
        *(.data*)           /*read-write initialized data: initialized global variable*/
        *(.spix_config*)    /* SPIX configuration functions need to be run from SRAM */
        *(.flashprog*)      /* Flash program */

        /* These array sections are used by __libc_init_array to call static C++ constructors */
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        _edata = ALIGN(., 4);
    } > SRAM AT>FLASH
    __load_data = LOADADDR(.data);

    .xip_0 :  AT(0x10000)
    {
        _XIP_START_0 = .;
        *(.xip_0*)
        _XIP_END_0 = .;
    } > XIP_0

    .xip_1 :  AT(0x11000)
    {
        _XIP_START_1 = .;
        *(.xip_1*)
        _XIP_END_1 = .;
    } > XIP_1

    .xip_2 : AT(0x12000)
    {
        _XIP_START_2 = .;
        *(.xip_2*)
        _XIP_END_2 = .;
    } > XIP_2

    .xip_sleep : AT(0x13000)
    {
        _XIP_START_SLEEP = .;
        *(.xip_sleep*)
        _XIP_END_SLEEP = .;
    } > XIP_SLEEP

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /*read-write zero initialized data: uninitialzed global variable*/
        *(COMMON)
        _ebss = ALIGN(., 4);
    } > SRAM

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SRAM

    .heap (COPY):
    {
        . = ALIGN(4);
        *(.heap*)
        __HeapLimit = ABSOLUTE(__StackLimit);
    } > SRAM

    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
