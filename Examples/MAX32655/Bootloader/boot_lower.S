
    .syntax unified
    .arch armv7-m
    .text   
    .thumb
    .thumb_func
    .align 2


    .globl   Boot_Lower
    .type    Boot_Lower %function
/* Boot from the lower flash array */
Boot_Lower:

    ldr     r0,=0x10004000      /* Address for main flash image */
    ldr     r1,=0xE000ED08      /* Address for SCB_VTOR_REG */

    /* First 32-bit word in image is initial stack pointer */
    /* Second 32-bit word in image is Address of the Reset Handler */
    
    /* Set VTOR with vector table base address */
    str     r0,[r1]
    /* Load initial SP */
    ldr     sp,[r0]
    /* Load LR with address of the Reset Handler */
    ldr     lr,[r0,#4]
    /* Jump to Reset Handler. */
    bx      lr
