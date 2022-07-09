/* *****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2016-05-02 14:15:59 -0700 (Mon, 02 May 2016) $
 * $Revision: 22594 $
 *
 **************************************************************************** */

MEMORY {
    ROM (rx)   : ORIGIN = 0x00000000, LENGTH = 0x00010000 /* 64kB ROM */
    HEADER (rx): ORIGIN = 0x10000000, LENGTH = 0x200
    FLASH (rx) : ORIGIN = 0x10000200, LENGTH = 0x002FFE00 /* 3MB internal flash */
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00100000 /* 1MB SRAM */
    /*
     * Note that CS0/CS1 address mappings may be reversed using MXC_HPC->mbr0 and ->mbr1
     * The following mappings are selected for simplicity
     */
    HPB_CS0 (rwx)  : ORIGIN = 0x60000000, LENGTH = 0x10000000  /* External Hyperbus/Xccelabus chip select 0 */
    HPB_CS1 (rwx)  : ORIGIN = 0x70000000, LENGTH = 0x10000000  /* External Hyperbus/Xccelabus chip select 1 */
}

/* Added Oct 9, 2018 to go to correct reset vector. */
ENTRY(Reset_Handler)
PROVIDE( _start_SWAP = (((Reset_Handler) >> 24) | (((Reset_Handler) & 0x00FF0000) >> 8) | (((Reset_Handler) & 0x0000FF00) << 8) | ((Reset_Handler) << 24)));
PROVIDE_HIDDEN( _SLA_Size = _endimage - __end_header );
PROVIDE( _SLA_Size_SWAP = (((_SLA_Size) >> 24) | (((_SLA_Size) & 0x00FF0000) >> 8) | (((_SLA_Size) & 0x0000FF00) << 8) | ((_SLA_Size) << 24)));

SECTIONS {
     .sb_sla_header : ALIGN(4)
    {
       FILL(0xFF)
        KEEP(*(.sb_sla_header)) /* Header for ROM code */ 
         __end_header = . ;
        . = ALIGN(512);
    } > HEADER

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

    /* These sections allow code to be compiled/linked for HPB addresses, but reside in
     * flash until copied by code to the external HPB flash device
     */
    .hpb_cs0_section :
    {
	__hpb_cs0_start = ABSOLUTE(.);
        KEEP(*(.hpb_cs0_section*))
    } > HPB_CS0 AT>FLASH

    __load_start_hpb_cs0 = LOADADDR(.hpb_cs0_section);
    __load_length_hpb_cs0 = SIZEOF(.hpb_cs0_section);

    .hpb_cs1_section :
    {
    	__hpb_cs1_start = ABSOLUTE(.);
        KEEP(*(.hpb_cs1_section*))
    } > HPB_CS1 AT>FLASH

    __load_start_hpb_cs1 = LOADADDR(.hpb_cs1_section);
    __load_length_hpb_cs1 = SIZEOF(.hpb_cs1_section);

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
    _enddata = LOADADDR(.data)+SIZEOF(.data);

    .sb_sla_trailer : AT(_enddata)
    {
        KEEP(*(.sb_sla_trailer))
        /* Align image with 16 byte boundary to conform to flash encryption block size. */
        FILL(0xDEADC0DE);
        /* NOTE: The FILL and ALIGN will not work unless something is written to the section.  So, we use LONG. */
        LONG(0xDEADC0DE);
        . = ALIGN(16);
    }  > FLASH
    _endimage = LOADADDR(.sb_sla_trailer)+SIZEOF(.sb_sla_trailer);
    .sig :
    {
        KEEP(*(.sig))
        LONG(0xDEADBEEF);

    }  > FLASH
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
