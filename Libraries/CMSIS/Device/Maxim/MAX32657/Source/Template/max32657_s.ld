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

/**
 * 1MB Total Physical Flash Size
 * 256KB Total Physical SRAM Size
 *      - SRAM0 (32KB)
 *      - SRAM1 (32KB)
 *      - SRAM2 (64KB)
 *      - SRAM3 (64KB)
 *      - SRAM4 (64KB)
 */

MEMORY {
    FLASH         (rx) : ORIGIN = $FLASH_ORIGIN_S$, LENGTH = $FLASH_SIZE_S$ /* User-selected Secure Flash */
    FLASH_NS      (rx) : ORIGIN = $FLASH_ORIGIN_NS$, LENGTH = $FLASH_SIZE_NS$ /* User-selected Non-Secure Flash */
    SRAM         (rwx) : ORIGIN = $SRAM_ORIGIN_S$, LENGTH = $SRAM_SIZE_S$ /* User-selected Secure SRAM */
    SRAM_NS      (rwx) : ORIGIN = $SRAM_ORIGIN_NS$, LENGTH = $SRAM_SIZE_NS$ /* User-selected Non-Secure SRAM */

    NSC_REGION   $perm$ : ORIGIN = $MEM_ORIGIN_NSC$, LENGTH = $MEM_SIZE_NSC$ /* User-selected Non-Secure Callable Region */
}

SECTIONS {
    .nonsecure_flash :
    {
        /* Non-Secure Code must be aligned with this memory's block size. */
        _nonsecure_start = ALIGN(., $MPC_BLOCK_SIZE$);
        KEEP(*nonsecure.o) /* Non-Secure program code */
        _nonsecure_end = ALIGN(., $MPC_BLOCK_SIZE$) - 1; /* -1 for last accessible address, not start of next block */
    } > $CODE_MEM_TYPE$_NS

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
    } > $CODE_MEM_TYPE$

    /*
     * Secure Gatway (SG) veneers.
     *  All SG veneers are placed in the special output section .gnu.sgstubs.
     *  Must be placed in Non-Secure Callable Region.
     */
    .gnu.sgstubs :
    {
        _sg_veneers = .;
        *(.gnu.sgstubs*)
        _esg_veneers = .;
    } > NSC_REGION

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > $CODE_MEM_TYPE$

    .flash_code :
    {
        . = ALIGN(16);
        _flash_code = .;
        *(.flash_code_section)
        _eflash_code = .;
    } > FLASH

    .sram_code :
    {
        . = ALIGN(16);
        _sram_code = .;
        *(.sram_code_section)
        _esram_code = .;
    } > SRAM

    /* it's used for C++ exception handling      */
    /* we need to keep this to avoid overlapping */
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > $CODE_MEM_TYPE$

    .data :
    {
        _data = ALIGN(., 4);
        *(vtable)
        *(.data*)           /* read-write initialized data: initialized global variable */
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
    } > SRAM AT>$CODE_MEM_TYPE$
    __load_data = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _bss = .;
        *(.bss*)     /* read-write zero initialized data: uninitialzed global variable */
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
        KEEP(*(.stackseal*))
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
