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

    .syntax unified
    .arch armv8-m.main

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ      Stack_Size, __STACK_SIZE
#else
    // Default stack size
    .equ      Stack_Size, 0x00001000
#endif
    .globl    __StackTop
    .globl    __StackLimit
#if IS_SECURE_ENVIRONMENT
    // __StackSeal defined in linker script
    .equ      __STACK_SEAL,  __StackSeal
    .globl    __StackSeal
#endif
__StackLimit:
    .space    Stack_Size
    .size     __StackLimit, . - __StackLimit
__StackTop:
    .size     __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ      Heap_Size, __HEAP_SIZE
#else
    // Default heap size
    .equ      Heap_Size, 0x00000C00
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    # TODO: Validate arch-specific handler layout
    .section .isr_vector
    .align 9 /* must be aligned to 512 byte boundary. VTOR requirement */
    .globl __isr_vector
__isr_vector:
    .long    __StackTop                 /* Top of Stack */
    .long    Reset_Handler              /* Reset Handler */
    .long    NMI_Handler                /* NMI Handler */
    .long    HardFault_Handler          /* Hard Fault Handler */
    .long    MemManage_Handler          /* MPU Fault Handler */
    .long    BusFault_Handler           /* Bus Fault Handler */
    .long    UsageFault_Handler         /* Usage Fault Handler */
    .long    SecureFault_Handler        /* Secure Fault Handler */
    .long    0                          /* Reserved */
    .long    0                          /* Reserved */
    .long    0                          /* Reserved */
    .long    SVC_Handler                /* SVCall Handler */
    .long    DebugMon_Handler           /* Debug Monitor Handler */
    .long    0                          /* Reserved */
    .long    PendSV_Handler             /* PendSV Handler */
    .long    SysTick_Handler            /* SysTick Handler */

    /* Device-specific Interrupts                                           */
    /*                                     CMSIS Interrupt Number           */
    /*                                     ||||          ||                 */
    /*                                     ||||  Offset  ||                 */
    /*                                     vvvv  vvvvvv  vv                 */

    .long ICE_IRQHandler                /* 0x10  0x0040  16: ICE Unlock */
    .long WDT_IRQHandler                /* 0x11  0x0044  17: Watchdog Timer */
    .long RTC_IRQHandler                /* 0x12  0x0048  18: RTC */
    .long TRNG_IRQHandler               /* 0x13  0x004C  19: True Random Number Generator */
    .long TMR0_IRQHandler               /* 0x14  0x0050  20: Timer 0 */
    .long TMR1_IRQHandler               /* 0x15  0x0054  21: Timer 1 */
    .long TMR2_IRQHandler               /* 0x16  0x0058  22: Timer 2 */
    .long TMR3_IRQHandler               /* 0x17  0x005C  23: Timer 3 */
    .long TMR4_IRQHandler               /* 0x18  0x0060  24: Timer 4 */
    .long TMR5_IRQHandler               /* 0x19  0x0064  25: Timer 5 */
    .long I3C_IRQHandler                /* 0x1A  0x0068  26: I3C */
    .long UART_IRQHandler               /* 0x1B  0x006C  27: UART */
    .long SPI_IRQHandler                /* 0x1C  0x0070  28: SPI */
    .long FLC_IRQHandler                /* 0x1D  0x0074  29: FLC */
    .long GPIO0_IRQHandler              /* 0x1E  0x0078  30: GPIO0 */
    .long RSV15_IRQHandler              /* 0x1F  0x007C  31: Reserved */
    .long DMA0_CH0_IRQHandler           /* 0x20  0x0080  32: DMA0 Channel 0 */
    .long DMA0_CH1_IRQHandler           /* 0x21  0x0084  33: DMA0 Channel 1 */
    .long DMA0_CH2_IRQHandler           /* 0x22  0x0088  34: DMA0 Channel 2 */
    .long DMA0_CH3_IRQHandler           /* 0x23  0x008C  35: DMA0 Channel 3 */
    .long DMA1_CH0_IRQHandler           /* 0x24  0x0090  36: DMA1 Channel 0 */
    .long DMA1_CH1_IRQHandler           /* 0x25  0x0094  37: DMA1 Channel 1 */
    .long DMA1_CH2_IRQHandler           /* 0x26  0x0098  38: DMA1 Channel 2 */
    .long DMA1_CH3_IRQHandler           /* 0x27  0x009C  39: DMA1 Channel 3 */
    .long WUT0_IRQHandler               /* 0x28  0x00A0  40: Wakeup Timer 0 */
    .long WUT1_IRQHandler               /* 0x29  0x00A4  41: Wakeup Timer 1 */
    .long GPIOWAKE_IRQHandler           /* 0x2A  0x00A8  42: GPIO Wakeup */
    .long CRC_IRQHandler                /* 0x2B  0x00AC  43: CRC */
    .long AES_IRQHandler                /* 0x2C  0x00B0  44: AES */
    .long ERFO_IRQHandler               /* 0x2D  0x00B4  45: ERFO Ready */
    .long BOOST_IRQHandler              /* 0x2E  0x00B8  46: Boost Controller */
    .long ECC_IRQHandler                /* 0x2F  0x00BC  47: ECC */
/* TODO(Bluetooth): Confirm BTLE IRQ Handler Names */
    .long BTLE_XXXX0_IRQHandler         /* 0x30  0x00C0  48: BTLE XXXX0 */
    .long BTLE_XXXX1_IRQHandler         /* 0x31  0x00C4  49: BTLE XXXX1 */
    .long BTLE_XXXX2_IRQHandler         /* 0x32  0x00C8  50: BTLE XXXX2 */
    .long BTLE_XXXX3_IRQHandler         /* 0x33  0x00CC  51: BTLE XXXX3 */
    .long BTLE_XXXX4_IRQHandler         /* 0x34  0x00D0  52: BTLE XXXX4 */
    .long BTLE_XXXX5_IRQHandler         /* 0x35  0x00D4  53: BTLE XXXX5 */
    .long BTLE_XXXX6_IRQHandler         /* 0x36  0x00D8  54: BTLE XXXX6 */
    .long BTLE_XXXX7_IRQHandler         /* 0x37  0x00DC  55: BTLE XXXX7 */
    .long BTLE_XXXX8_IRQHandler         /* 0x38  0x00E0  56: BTLE XXXX8 */
    .long BTLE_XXXX9_IRQHandler         /* 0x39  0x00E4  57: BTLE XXXX9 */
    .long BTLE_XXXXA_IRQHandler         /* 0x3A  0x00E8  58: BTLE XXXXA */
    .long BTLE_XXXXB_IRQHandler         /* 0x3B  0x00EC  59: BTLE XXXXB */
    .long BTLE_XXXXC_IRQHandler         /* 0x3C  0x00F0  60: BTLE XXXXC */
    .long BTLE_XXXXD_IRQHandler         /* 0x3D  0x00F4  61: BTLE XXXXD */
    .long BTLE_XXXXE_IRQHandler         /* 0x3E  0x00F8  62: BTLE XXXXE */
    .long RSV47_IRQHandler              /* 0x3F  0x00FC  63: Reserved */
    .long MPC_IRQHandler                /* 0x40  0x0100  64: MPC Combined (Secure) */
    .long PPC_IRQHandler                /* 0x44  0x0104  65: PPC Combined (Secure) */
    .long RSV50_IRQHandler              /* 0x48  0x0108  66: Reserved */
    .long RSV51_IRQHandler              /* 0x49  0x010C  67: Reserved */
    .long RSV52_IRQHandler              /* 0x4A  0x0110  68: Reserved */
    .long RSV53_IRQHandler              /* 0x4B  0x0114  69: Reserved */

    .text 	
    .thumb
    .thumb_func
    .align 2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    ldr r0, =__StackTop
    mov sp, r0

#if IS_SECURE_ENVIRONMENT
    /* Set limit on Main and Process SP */
    ldr r0, =__STACK_LIMIT
    msr msplim, r0
    msr psplim, r0

    /* Set up Stack Sealing - using predefined stack seal value */
    ldr r0, =__STACK_SEAL
    ldr r1, =0xFEF5EDA5U
    /* Store seal value twice as a double word for redundancy */
    strd r1, r1, [r0, #0]
#endif

    /* PreInit runs before any RAM initialization. Example usage: DDR setup, etc. */
    ldr     r0, =PreInit
    blx     r0
    cbnz    r0, .SKIPRAMINIT

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
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
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
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2

.SKIPRAMINIT:

    /* Perform system initialization after RAM initialization */
    ldr     r0, =SystemInit
    blx     r0

    /* This must be called to walk the constructor array for static C++ objects */
    /* Note: The linker file must have .data symbols for __X_array_start and __X_array_end */
    /*        where X is {preinit, init, fini}                                             */
    ldr     r0, =__libc_init_array
    blx     r0  

    /* Transfer control to user's main program */
    ldr     r0, =main
    blx     r0 

.SPIN:
    /* spin if main ever returns. */
    bl .SPIN

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SecureFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    Default_Handler

    /* Device-specific Interrupts                                                     */
    /*                                               CMSIS Interrupt Number           */
    /*                                               ||||          ||                 */
    /*                                               ||||  Offset  ||                 */
    /*                                               vvvv  vvvvvv  vv                 */

    def_irq_handler ICE_IRQHandler                /* 0x10  0x0040  16: ICE Unlock */
    def_irq_handler WDT_IRQHandler                /* 0x11  0x0044  17: Watchdog Timer */
    def_irq_handler RTC_IRQHandler                /* 0x12  0x0048  18: RTC */
    def_irq_handler TRNG_IRQHandler               /* 0x13  0x004C  19: True Random Number Generator */
    def_irq_handler TMR0_IRQHandler               /* 0x14  0x0050  20: Timer 0 */
    def_irq_handler TMR1_IRQHandler               /* 0x15  0x0054  21: Timer 1 */
    def_irq_handler TMR2_IRQHandler               /* 0x16  0x0058  22: Timer 2 */
    def_irq_handler TMR3_IRQHandler               /* 0x17  0x005C  23: Timer 3 */
    def_irq_handler TMR4_IRQHandler               /* 0x18  0x0060  24: Timer 4 */
    def_irq_handler TMR5_IRQHandler               /* 0x19  0x0064  25: Timer 5 */
    def_irq_handler I3C_IRQHandler                /* 0x1A  0x0068  26: I3C */
    def_irq_handler UART_IRQHandler               /* 0x1B  0x006C  27: UART */
    def_irq_handler SPI_IRQHandler                /* 0x1C  0x0070  28: SPI */
    def_irq_handler FLC_IRQHandler                /* 0x1D  0x0074  29: FLC */
    def_irq_handler GPIO0_IRQHandler              /* 0x1E  0x0078  30: GPIO0 */
    def_irq_handler RSV15_IRQHandler              /* 0x1F  0x007C  31: Reserved */
    def_irq_handler DMA0_CH0_IRQHandler           /* 0x20  0x0080  32: DMA0 Channel 0 */
    def_irq_handler DMA0_CH1_IRQHandler           /* 0x21  0x0084  33: DMA0 Channel 1 */
    def_irq_handler DMA0_CH2_IRQHandler           /* 0x22  0x0088  34: DMA0 Channel 2 */
    def_irq_handler DMA0_CH3_IRQHandler           /* 0x23  0x008C  35: DMA0 Channel 3 */
    def_irq_handler DMA1_CH0_IRQHandler           /* 0x24  0x0090  36: DMA1 Channel 0 */
    def_irq_handler DMA1_CH1_IRQHandler           /* 0x25  0x0094  37: DMA1 Channel 1 */
    def_irq_handler DMA1_CH2_IRQHandler           /* 0x26  0x0098  38: DMA1 Channel 2 */
    def_irq_handler DMA1_CH3_IRQHandler           /* 0x27  0x009C  39: DMA1 Channel 3 */
    def_irq_handler WUT0_IRQHandler               /* 0x28  0x00A0  40: Wakeup Timer 0 */
    def_irq_handler WUT1_IRQHandler               /* 0x29  0x00A4  41: Wakeup Timer 1 */
    def_irq_handler GPIOWAKE_IRQHandler           /* 0x2A  0x00A8  42: GPIO Wakeup */
    def_irq_handler CRC_IRQHandler                /* 0x2B  0x00AC  43: CRC */
    def_irq_handler AES_IRQHandler                /* 0x2C  0x00B0  44: AES */
    def_irq_handler ERFO_IRQHandler               /* 0x2D  0x00B4  45: ERFO Ready */
    def_irq_handler BOOST_IRQHandler              /* 0x2E  0x00B8  46: Boost Controller */
    def_irq_handler ECC_IRQHandler                /* 0x2F  0x00BC  47: ECC */
/* TODO(Bluetooth): Confirm BTLE IRQ Handler Names */
    def_irq_handler BTLE_XXXX0_IRQHandler         /* 0x30  0x00C0  48: BTLE XXXX0 */
    def_irq_handler BTLE_XXXX1_IRQHandler         /* 0x31  0x00C4  49: BTLE XXXX1 */
    def_irq_handler BTLE_XXXX2_IRQHandler         /* 0x32  0x00C8  50: BTLE XXXX2 */
    def_irq_handler BTLE_XXXX3_IRQHandler         /* 0x33  0x00CC  51: BTLE XXXX3 */
    def_irq_handler BTLE_XXXX4_IRQHandler         /* 0x34  0x00D0  52: BTLE XXXX4 */
    def_irq_handler BTLE_XXXX5_IRQHandler         /* 0x35  0x00D4  53: BTLE XXXX5 */
    def_irq_handler BTLE_XXXX6_IRQHandler         /* 0x36  0x00D8  54: BTLE XXXX6 */
    def_irq_handler BTLE_XXXX7_IRQHandler         /* 0x37  0x00DC  55: BTLE XXXX7 */
    def_irq_handler BTLE_XXXX8_IRQHandler         /* 0x38  0x00E0  56: BTLE XXXX8 */
    def_irq_handler BTLE_XXXX9_IRQHandler         /* 0x39  0x00E4  57: BTLE XXXX9 */
    def_irq_handler BTLE_XXXXA_IRQHandler         /* 0x3A  0x00E8  58: BTLE XXXXA */
    def_irq_handler BTLE_XXXXB_IRQHandler         /* 0x3B  0x00EC  59: BTLE XXXXB */
    def_irq_handler BTLE_XXXXC_IRQHandler         /* 0x3C  0x00F0  60: BTLE XXXXC */
    def_irq_handler BTLE_XXXXD_IRQHandler         /* 0x3D  0x00F4  61: BTLE XXXXD */
    def_irq_handler BTLE_XXXXE_IRQHandler         /* 0x3E  0x00F8  62: BTLE XXXXE */
    def_irq_handler RSV47_IRQHandler              /* 0x3F  0x00FC  63: Reserved */
    def_irq_handler MPC_IRQHandler                /* 0x40  0x0100  64: MPC Combined (Secure) */
    def_irq_handler PPC_IRQHandler                /* 0x44  0x0104  65: PPC Combined (Secure) */
    def_irq_handler RSV50_IRQHandler              /* 0x48  0x0108  66: Reserved */
    def_irq_handler RSV51_IRQHandler              /* 0x49  0x010C  67: Reserved */
    def_irq_handler RSV52_IRQHandler              /* 0x4A  0x0110  68: Reserved */
    def_irq_handler RSV53_IRQHandler              /* 0x4B  0x0114  69: Reserved */
    .end
