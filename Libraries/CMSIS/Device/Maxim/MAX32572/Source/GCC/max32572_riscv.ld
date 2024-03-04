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

MSR_DATA_LEN = 0x300;

MEMORY {
    SRAM5 (rw) : ORIGIN = 0x20060000, LENGTH = 8K
    SRAM5_MSRDATA (rw) : ORIGIN = 0x20062000, LENGTH = MSR_DATA_LEN
    SRAM5_ROMDATA (rw) : ORIGIN = 0x20062000 + MSR_DATA_LEN, LENGTH = 24K - MSR_DATA_LEN
    SRAM6 (rwx) : ORIGIN = 0x20068000, LENGTH = 16K
/*    ROM (rx) : ORIGIN = 0x2006C000, LENGTH = 32K */
}

SECTIONS {
    
    .text :
    {
        _text = ALIGN(.,4);
        KEEP(*(.isr_vector))
        KEEP(*(.c_iv))
        *(.text*)  /* program code */

        KEEP(*(.init))
        KEEP(*(.fini))

        /* C++ Exception handling */
        KEEP(*(.eh_frame*))
        _etext = .;
    } > SRAM6

    /* short/global data section */
    .sdata :
    {
        _sdata = ALIGN(., 4);
        __sdata_start = .; 
        PROVIDE(__global_pointer$ = . + 0x800);        
        *(.sdata*)  /*read-write initialized data: initialized global variable*/

        _esdata = ALIGN(., 4);
        __sdata_end = .;
    
    } > SRAM5
    __load_sdata = LOADADDR(.sdata);

    .data :
    {
        _data = ALIGN(., 4);
        __data_start__ = . ;            /* Standard newlib definition. */
    
        *(.data*)           /*read-write initialized data: initialized global variable*/
        *(.sdata*)  /*read-write initialized data: initialized global variable*/
        *(.srodata*) /* read-only data: "const" */
        *(.rodata*)  /* read-only data: "const" */

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
        __data_end__ = . ;                /* Standard newlib definition. */
    } > SRAM5
    __load_data = LOADADDR(.data);

    .sbss :
    {
        . = ALIGN(4);
        __sbss_start = .;                 /* Standard newlib definition. */
        __sbss_start__ = .;                /* Standard newlib definition. */
        _sbss = .;
        *(.sbss*)     /*read-write zero initialized data: uninitialzed global variable*/
        *(COMMON)
        _esbss = ALIGN(., 4);
        __sbss_end__ = .;                /* Standard newlib definition. */
        __sbss_end = .;                    /* Standard newlib definition. */
    } > SRAM5

    .bss :
    {
        . = ALIGN(4);
        __bss_start = .;                 /* Standard newlib definition. */
        __bss_start__ = .;                 /* Standard newlib definition. */
        _bss = .;
        *(.bss*)     /*read-write zero initialized data: uninitialzed global variable*/
        *(COMMON)
        _ebss = ALIGN(., 4);
        __bss_end__ = .;                /* Standard newlib definition. */
        __bss_end = .;                    /* Standard newlib definition. */
     } > SRAM5

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM5) + LENGTH(SRAM5);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SRAM5

    .heap (COPY):
    {
        . = ALIGN(4);
        *(.heap*)
        __HeapLimit = ABSOLUTE(__StackLimit);
    } > SRAM5

    /* init two words preceding const seg to address and length of segment */
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
