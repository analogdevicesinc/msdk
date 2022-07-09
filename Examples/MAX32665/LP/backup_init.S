/* Backup_Init recovers the system from backup mode (without SRAM) 
 * and jumps to a location determined in BBGP1 (0x4000684C). 
 * This would probably be better done in Reset_Handler by checking 
 * for the 'magic values' set by ROM when recovering from backup
 * and jumping to BBGP1 if present, otherwise jumping to main
 */

    .syntax unified
    .arch armv7-m
    .text   
    .thumb
    .thumb_func
    .align 2
    .globl   Backup_Init
    .type    Backup_Init, %function
Backup_Init:
    ldr r0, =0x40006800
    ldr r1, [r0]
    orr r2, r1, #0x1
    str r2, [r0]

    ldr r0, =__StackTop
    mov sp, r0

    /* PreInit runs before any RAM initialization. Example usage: DDR setup, etc. */
    ldr     r0, =PreInit

/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __load_data: Where data sections are saved.
 *      _data /_edata: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__load_data
    ldr    r2, =_data
    ldr    r3, =_edata

#if 0
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
.BLC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .BLC0
#else
    subs    r3, r2
    ble    .BLC1
.BLC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .BLC0
.BLC1:
#endif

/*
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      _bss  : start of BSS section. Must align to 4
 *      _ebss : end of BSS section. Must align to 4
 */
    ldr r1, =_bss
    ldr r2, =_ebss

    movs    r0, 0
.BLC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .BLC2

    /* Perform system initialization after RAM initialization */
    ldr     r0, =SystemInit
    blx     r0

    /* This must be called to walk the constructor array for static C++ objects */
    /* Note: The linker file must have .data symbols for __X_array_start and __X_array_end */
    /*        where X is {preinit, init, fini}                                             */
    ldr     r0, =__libc_init_array
    blx     r0  

    /* Transfer control to users main program */
    ldr     r1, =0x4000684C
    ldr     r0, [r1]
    blx     r0 