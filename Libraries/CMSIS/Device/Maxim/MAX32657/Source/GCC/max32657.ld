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

MEMORY {
    ROM_S         (rx) : ORIGIN = 0x00000000, LENGTH = 0x00010000 /* 64kB secure ROM (ROM always secure) */
    FLASH         (rx) : ORIGIN = DEFINED(SECURE_LINK) ? 0x11000000 : 0x01000000, LENGTH = 0x00100000 /* 1MB Flash */
    FLASH_INFO_S  (rx) : ORIGIN = 0x12000000, LENGTH = 0x00010000 /* 16KB secure Flash Info */
    SRAM         (rwx) : ORIGIN = DEFINED(SECURE_LINK) ? 0x30000000 : 0x20000000, LENGTH = 0x00040000 /* 256KB SRAM */
}
/* Note(JC): "SECURE_LINK" is a symbol defined at link-time by Libraries/CMSIS/Device/Maxim/GCC/gcc.mk
 * It's only defined for when building secure binaries (which is the default).
 * When it's defined, the FLASH and SRAM memory addresses will use the secure ranges.  Otherwise, the
 * non-secure ranges will be used.  This mechanism allows secure and non-secure binaries to be linked
 * without having to maintain a separate copy of the linkerfile.
 */

SECTIONS {
    .rom :
    {
        KEEP(*(.rom_vector))
        *(.rom_handlers*)
    } > ROM_S

    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)    /* Program code */
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
    } > FLASH


    /*
     * Secure Gatway (SG) veneers.
     *  All SG veneers are placed in the special output section .gnu.sgstubs.
     */
    .gnu.sgstubs :
    {
        _sg_veneers = .;
        *(.gnu.sgstubs*)
        _esg_veneers = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    
    .rom_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.rom_code_section)
        _esran_code = .;
    } > ROM_S

    .flash_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.flash_code_section)
        _esran_code = .;
    } > FLASH

    .sram_code :
    {
        . = ALIGN(16);
        _sran_code = .;
        *(.sram_code_section)
        _esran_code = .;
    } > SRAM

    /* it's used for C++ exception handling      */
    /* we need to keep this to avoid overlapping */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    .data :
    {
        _data = ALIGN(., 4);
        *(vtable)
        *(.data*)           /*read-write initialized data: initialized global variable*/
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

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /*read-write zero initialized data: uninitialzed global variable*/
        _ebss = ALIGN(., 4);
    } > SRAM

    /**
     *  Stack Seal section is required for secure builds. Stack sealing protects
     *  secure stack from illegal access by non-secure code.
     */
    .stackseal (COPY):
    {
        . = ALIGN(8);
        __StackSeal = .;
        . = . + 8;
        . = ALIGN(8);
    } > SRAM

    /**
     *   Set stack top to end of RAM, and stack limit move down by
     *      size of stack_dummy section.
     */
    __StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);

    /**
     *  .stack_dummy section doesn't contains any symbols. It is only
     *  used for linker to calculate size of stack sections, and assign
     *  values to stack symbols later.
     */
    .stack_dummy (COPY):
    {
        KEEP(*(.stack*))
        *(.stackseal*)
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
