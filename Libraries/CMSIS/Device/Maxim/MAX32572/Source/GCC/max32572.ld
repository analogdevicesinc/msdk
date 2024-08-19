/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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

MEMORY {
    ROM (rx)      : ORIGIN = 0x00000000, LENGTH = 64K
    SPIX (rx)     : ORIGIN = 0x08000000, LENGTH = 128M
    SRAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 0x00060000 /* Total: 432kB SRAM */
    SRAM_RV (rwx) : ORIGIN = 0x20060000, LENGTH = 0x0000C000
}

ENTRY(Reset_Handler)
PROVIDE( _start_SWAP = (((Reset_Handler) >> 24) | (((Reset_Handler) & 0x00FF0000) >> 8) | (((Reset_Handler) & 0x0000FF00) << 8) | ((Reset_Handler) << 24)));
PROVIDE_HIDDEN( _SLA_Size = __end_image - __end_header );
PROVIDE( _SLA_Size_SWAP = (((_SLA_Size) >> 24) | (((_SLA_Size) & 0x00FF0000) >> 8) | (((_SLA_Size) & 0x0000FF00) << 8) | ((_SLA_Size) << 24)));

SECTIONS {
    .sla_header : ALIGN(4)
    {
        FILL(0xFF);
        KEEP(*(.sb_sla_header)) /* Header for ROM code */
        __end_header = .;
        . = ALIGN(512);
    } > SPIX

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

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        /* C++ Exception handling */
        KEEP(*(.eh_frame*))
        _etext = .;
    } > SPIX

    /* Binary import */
    .bin_storage :
    {
       FILL(0xFF)
      _bin_start_ = .;
      KEEP(*(.bin_storage_img))
      _bin_end_ = .;
      . = ALIGN(4);
    } > SPIX

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > SPIX

    /* it's used for C++ exception handling      */
    /* we need to keep this to avoid overlapping */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > SPIX

    /* This section will keep the SPIX data until loaded into the external device */
    /* Upon initialization of SPIX (user code needs to do this) */
    .xip_section :
    {
        KEEP(*(.xip_section*))
    } > SPIX AT>SPIX
    __load_start_xip = LOADADDR(.xip_section);
    __load_length_xip = SIZEOF(.xip_section);

    .data :
    {
        _data = ALIGN(., 4);
        *(vtable)
        *(.data*)           /* read-write initialized data: initialized global variable */
        *(.spix_config*)    /* SPIX configuration functions need to be run from SRAM */

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
    } > SRAM AT>SPIX
    __load_data = LOADADDR(.data);
    __end_data = LOADADDR(.data)+SIZEOF(.data);

    .sram_rv :
    {
        _data_rv = ALIGN(., 4);
        *(.sram_rv*)
        _edata_rv = ALIGN(., 4);
    } > SRAM_RV

    .sb_sla_trailer : AT(__end_data)
    {
        KEEP(*(.sb_sla_trailer))
        /* Align image with 16-byte boundary to conform to flash encryption block size. */
        FILL(0xDEADC0DE);
        /* NOTE: The FILL and ALIGN will not work unless something is written to the section.  So, we use LONG. */
        LONG(0xDEADC0DE);
        . = ALIGN(16);
    }  > SPIX
    __end_image = LOADADDR(.sb_sla_trailer)+SIZEOF(.sb_sla_trailer);

    .sig :
    {
        KEEP(*(.sig))
        LONG(0xDEADBEEF);
    }  > SPIX

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /* read-write zero-initialized data: uninitialized global variable */
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
