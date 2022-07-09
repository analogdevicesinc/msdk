/*******************************************************************************
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
 * $Date: 2016-12-22 17:11:40 -0600 (Thu, 22 Dec 2016) $ 
 * $Revision: 25614 $
 *
 ******************************************************************************/

MEMORY {
    ROM        (rx) : ORIGIN = 0x00000000, LENGTH = 0x00020000 /* 128kB ROM */
    SPIX_FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x08000000 /* 128MB SPIX Flash */
    FLASH      (rx) : ORIGIN = 0x10000000, LENGTH = 0x00300000 /* 3MB Flash */
    SRAM0     (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000 /* 32kB SRAM0 */
    SRAM1     (rwx) : ORIGIN = 0x20008000, LENGTH = 0x00010000 /* 64kB SRAM1 */
    SRAM2     (rwx) : ORIGIN = 0x20018000, LENGTH = 0x00008000 /* 32kB SRAM2 */
    SRAM3     (rwx) : ORIGIN = 0x20020000, LENGTH = 0x00020000 /* 128kB SRAM3 */
    SRAM4     (rwx) : ORIGIN = 0x20040000, LENGTH = 0x00040000 /* 256kB SRAM4 */
    SRAM5     (rwx) : ORIGIN = 0x20080000, LENGTH = 0x00040000 /* 256kB SRAM5 */
    SRAM6     (rwx) : ORIGIN = 0x200C0000, LENGTH = 0x00040000 /* 256kB SRAM6 */
    HBUS      (rwx) : ORIGIN = 0x60000000, LENGTH = 0x20000000 /* 512MB Hyperbus */
    SPIX_DATA (rwx) : ORIGIN = 0x80000000, LENGTH = 0x20000000 /* 512MB SPIX Data */
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

    .rom_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.rom_code_section)
        _esran_code = .;
    } > ROM

    .spix_flash_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.spix_flash_code_section)
        _esran_code = .;
    } > SPIX_FLASH

    .flash_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.flash_code_section)
        _esran_code = .;
    } > FLASH

    .sram0_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram0_code_section)
        _esran_code = .;
    } > SRAM0

    .sram1_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram1_code_section)
        _esran_code = .;
    } > SRAM1

    .sram2_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram2_code_section)
        _esran_code = .;
    } > SRAM2

    .sram3_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram3_code_section)
        _esran_code = .;
    } > SRAM3

    .sram4_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram4_code_section)
        _esran_code = .;
    } > SRAM4

    .sram5_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram5_code_section)
        _esran_code = .;
    } > SRAM5

    .sram6_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram6_code_section)
        _esran_code = .;
    } > SRAM6

    .hbus_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.hbus_code_section)
        _esran_code = .;
    } > HBUS

    .spix_data_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.spix_data_code_section)
        _esran_code = .;
    } > SPIX_DATA

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
        *(.spix_config*)    /* SPIX configuration functions need to be run from SRAM0 */

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
    } > SRAM0 AT>FLASH
    __load_data = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /*read-write zero initialized data: uninitialzed global variable*/
        *(COMMON)
        _ebss = ALIGN(., 4);
    } > SRAM0

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM0) + LENGTH(SRAM0);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SRAM0

    .heap (COPY):
    {
        . = ALIGN(4);
        *(.heap*)
        __HeapLimit = ABSOLUTE(__StackLimit);
    } > SRAM0

    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
