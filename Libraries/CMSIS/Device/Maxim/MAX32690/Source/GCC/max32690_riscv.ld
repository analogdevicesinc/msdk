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

FLASH_MEM_SIZE = _RISCV_FLASH_SIZE - _PAL_NVM_SIZE;
FLASH_MEM_ORIGIN = _RISCV_FLASH_ORIGIN - _PAL_NVM_SIZE;

SRAM_MEM_ORIGIN = _RISCV_SRAM_ORIGIN + _MAILBOX_SIZE*2;
SRAM_MEM_SIZE =  _RISCV_SRAM_SIZE - _MAILBOX_SIZE*2;

MAILBOX_ORIGIN = _RISCV_SRAM_ORIGIN;

MEMORY {
    FLASH      (rx) : ORIGIN = FLASH_MEM_ORIGIN, LENGTH = FLASH_MEM_SIZE
    PAL_NVM_DB (r)  : ORIGIN = FLASH_MEM_ORIGIN + FLASH_MEM_SIZE, LENGTH = _PAL_NVM_SIZE
    MAILBOX_0  (rw) : ORIGIN = MAILBOX_ORIGIN, LENGTH = _MAILBOX_SIZE
    MAILBOX_1  (rw) : ORIGIN = MAILBOX_ORIGIN + _MAILBOX_SIZE, LENGTH = _MAILBOX_SIZE
    SRAM      (rwx) : ORIGIN = SRAM_MEM_ORIGIN, LENGTH = SRAM_MEM_SIZE
}

SECTIONS {
    
    .text :
    {
        _text = ALIGN(.,13);
        KEEP(*(.isr_vector))
        KEEP(*(.c_iv))
        *(.text*)    /* program code */

        *(.srodata*) /* read-only data: "const" */
        *(.rodata*)  /* read-only data: "const" */

        KEEP(*(.init))
        KEEP(*(.fini))

        /* C++ Exception handling */
        KEEP(*(.eh_frame*))
        _etext = .;
    } > FLASH

    .mailbox_0 (NOLOAD) :
    {
        _mailbox_0 = ALIGN(., 4); 
        *(.mailbox_0*)           /*Shared memory section*/
        . = . + _MAILBOX_SIZE;
        _emailbox_0 = ALIGN(., 4);
    } > MAILBOX_0

    .mailbox_1 (NOLOAD) :
    {
        _mailbox_1 = ALIGN(., 4);
        *(.mailbox_1*)           /*Shared memory section*/
        . = . + _MAILBOX_SIZE;
        _emailbox_1 = ALIGN(., 4);
    } > MAILBOX_1

    /* short/global data section */
    .sdata :
    {
        _sdata = ALIGN(., 4);
        __sdata_start = .; 
        PROVIDE(__global_pointer$ = . + 0x800);        
        *(.sdata*)  /*read-write initialized data: initialized global variable*/

        _esdata = ALIGN(., 4);
        __sdata_end = .;
    
    } > SRAM AT>FLASH
    __load_sdata = LOADADDR(.sdata);

    .data :
    {
        _data = ALIGN(., 4);
        __data_start__ = . ;            /* Standard newlib definition. */
    
        *(.data*)           /*read-write initialized data: initialized global variable*/
        *(.spix_config*)    /* SPIX configuration functions need to be run from SRAM3 */

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
    } > SRAM AT>FLASH
    __load_data = LOADADDR(.data);

    .sbss :
    {
        . = ALIGN(4);
        __sbss_start = .;                 /* Standard newlib definition. */
    __sbss_start__ = .;                 /* Standard newlib definition. */
        _sbss = .;
        *(.sbss*)     /*read-write zero initialized data: uninitialzed global variable*/
        *(COMMON)
        _esbss = ALIGN(., 4);
        __sbss_end__ = .;                /* Standard newlib definition. */
        __sbss_end = .;                    /* Standard newlib definition. */
    } > SRAM

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
     } > SRAM

    .pal_nvm_db :
    {
    /* Align to the sector size */
    . = ALIGN(0x2000);
    __pal_nvm_db_start__ = .;
    . = . + _PAL_NVM_SIZE;
    __pal_nvm_db_end__ = .;
    } > PAL_NVM_DB
     

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

    /* init two words preceding const seg to address and length of segment */
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= _ebss, "region RAM overflowed with stack")
}
