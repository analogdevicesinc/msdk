/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

    .syntax unified
    .arch armv7-m

    .section .stack
    .align 3
	
	/* Define SDMA_SIZE to enable the loading of the SDMA image */
#ifdef __MSR_SDMA_SIZE
    .global MSR_SDMA_SIZE
    .equ    MSR_SDMA_SIZE, __MSR_SDMA_SIZE - 512
#endif

#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0x00001000
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x00000C00
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


    .section .isr_vector
    .align 9
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* Device-specific Interrupts */
    .long PF_IRQHandler             /* 0x10  0x0040  16: Power Fail */
    .long WDT0_IRQHandler           /* 0x11  0x0044  17: Watchdog 0 */
    .long USB_IRQHandler            /* 0x12  0x0048  18: USB */
    .long RTC_IRQHandler            /* 0x13  0x004C  19: RTC */
    .long TRNG_IRQHandler           /* 0x14  0x0050  20: True Random Number Generator */
    .long TMR0_IRQHandler           /* 0x15  0x0054  21: Timer 0 */
    .long TMR1_IRQHandler           /* 0x16  0x0058  22: Timer 1 */
    .long TMR2_IRQHandler           /* 0x17  0x005C  23: Timer 2 */
    .long TMR3_IRQHandler           /* 0x18  0x0060  24: Timer 3*/
    .long TMR4_IRQHandler           /* 0x19  0x0064  25: Timer 4 */
    .long TMR5_IRQHandler           /* 0x1A  0x0068  26: Timer 5 */
    .long SC0_IRQHandler            /* 0x1B  0x006C  27: Smart Card 0 */
    .long RSV12_IRQHandler          /* 0x1C  0x0070  28: Reserved */
    .long I2C0_IRQHandler           /* 0x1D  0x0074  29: I2C0 */
    .long UART0_IRQHandler          /* 0x1E  0x0078  30: UART 0 */
    .long UART1_IRQHandler          /* 0x1F  0x007C  31: UART 1 */
    .long SPI0_IRQHandler           /* 0x20  0x0080  32: SPI0 */
    .long SPI1_IRQHandler           /* 0x21  0x0084  33: SPI1 */
    .long RSV18_IRQHandler          /* 0x22  0x0088  34: Reserved */
    .long SKB_IRQHandler            /* 0x23  0x008C  35: Secure Keyboard */
    .long ADC_IRQHandler            /* 0x24  0x0090  36: ADC */
    .long RSV21_IRQHandler          /* 0x25  0x0094  37: Reserved */
    .long MSRADC_IRQHandler         /* 0x26  0x0098  38: Magstripe DSP */
    .long RSV23_IRQHandler          /* 0x27  0x009C  39: Reserved */
    .long GPIO0_IRQHandler          /* 0x28  0x00A0  40: GPIO0 */
    .long GPIO1_IRQHandler          /* 0x29  0x00A4  41: GPIO1 */
    .long GPIO2_IRQHandler          /* 0x2A  0x00A8  42: GPIO2 */
    .long CRYPTO_IRQHandler         /* 0x2B  0x00AC  43: Crypto */
    .long DMA0_IRQHandler           /* 0x2C  0x00B0  44: DMA0 */
    .long DMA1_IRQHandler           /* 0x2D  0x00B4  45: DMA1 */
    .long DMA2_IRQHandler           /* 0x2E  0x00B8  46: DMA2 */
    .long DMA3_IRQHandler           /* 0x2F  0x00BC  47: DMA3 */
    .long RSV32_IRQHandler          /* 0x30  0x00C0  48: Reserved */
    .long RSV33_IRQHandler          /* 0x31  0x00C4  49: Reserved */
    .long UART2_IRQHandler          /* 0x32  0x00C8  50: UART 2 */
    .long RSV35_IRQHandler          /* 0x33  0x00CC  51: Reserved */
    .long I2C1_IRQHandler           /* 0x34  0x00D0  52: I2C1 */
    .long SC1_IRQHandler            /* 0x35  0x00D4  53: Smart Card 1 */
    .long SPIXFC_IRQHandler         /* 0x36  0x00D8  54: SPI execute in place */
    .long RSV39_IRQHandler          /* 0x37  0x00DC  55: Reserved */
    .long RSV40_IRQHandler          /* 0x38  0x00E0  56: Reserved */
    .long RSV41_IRQHandler          /* 0x39  0x00E4  57: Reserved */
    .long RSV42_IRQHandler          /* 0x3A  0x00E8  58: Reserved */
    .long RSV43_IRQHandler          /* 0x3B  0x00EC  59: Reserved */
    .long RSV44_IRQHandler          /* 0x3C  0x00F0  60: Reserved */
    .long RSV45_IRQHandler          /* 0x3D  0x00F4  61: Reserved */
    .long RSV46_IRQHandler          /* 0x3E  0x00F8  62: Reserved */
    .long RSV47_IRQHandler          /* 0x3F  0x00FC  63: Reserved */
    .long RSV48_IRQHandler          /* 0x40  0x0100  64: Reserved */
    .long RSV49_IRQHandler          /* 0x41  0x0104  65: Reserved */
    .long RSV50_IRQHandler          /* 0x42  0x0108  66: Reserved */
    .long RSV51_IRQHandler          /* 0x43  0x010C  67: Reserved */
    .long RSV52_IRQHandler          /* 0x44  0x0110  68: Reserved */
    .long RSV53_IRQHandler          /* 0x45  0x0114  69: Reserved */
    .long GPIOWAKE_IRQHandler       /* 0x46  0x0118  70: GPIO Wakeup */
    .long RSV55_IRQHandler          /* 0x47  0x011C  71: Reserved */
    .long SPI2_IRQHandler           /* 0x48  0x0120  72: SPI2 */
    .long WDT1_IRQHandler           /* 0x49  0x0124  73: Watchdog 1 */
    .long GPIO3_IRQHandler          /* 0x4A  0x0128  74: GPIO 3 */
    .long PT_IRQHandler             /* 0x4B  0x012C  75: Pulse train */
    .long RSV60_IRQHandler          /* 0x4C  0x0130  76: Reserved */
    .long RSV61_IRQHandler          /* 0x4D  0x0134  77: Reserved */
    .long RSV62_IRQHandler          /* 0x4E  0x0138  78: Reserved */
    .long RISCV_IRQHandler          /* 0x4F  0x013C  79: RISCV */
    .long RSV64_IRQHandler          /* 0x50  0x0140  80: Reserved */
    .long RSV65_IRQHandler          /* 0x51  0x0144  81: Reserved */
    .long RSV66_IRQHandler          /* 0x52  0x0148  82: Reserved */
    .long RSV67_IRQHandler          /* 0x53  0x014C  83: Reserved */
    .long DMA4_IRQHandler           /* 0x54  0x0150  84: DMA4 */
    .long DMA5_IRQHandler           /* 0x55  0x0154  85: DMA5 */
    .long DMA6_IRQHandler           /* 0x56  0x0158  86: DMA6 */
    .long DMA7_IRQHandler           /* 0x57  0x015C  87: DMA7 */
    .long DMA8_IRQHandler           /* 0x58  0x0160  88: DMA8 */
    .long DMA9_IRQHandler           /* 0x59  0x0164  89: DMA9 */
    .long DMA10_IRQHandler          /* 0x5A  0x0168  90: DMA10 */
    .long DMA11_IRQHandler          /* 0x5B  0x016C  91: DMA11 */
    .long DMA12_IRQHandler          /* 0x5C  0x0170  92: DMA12 */
    .long DMA13_IRQHandler          /* 0x5D  0x0174  93: DMA13 */
    .long DMA14_IRQHandler          /* 0x5E  0x0178  94: DMA14 */
    .long DMA15_IRQHandler          /* 0x5F  0x017C  95: DMA15 */
    .long USBDMA_IRQHandler         /* 0x60  0x0180  96: USBDMA */
    .long RSV81_IRQHandler          /* 0x61  0x0184  97: Reserved */
    .long ECC_IRQHandler            /* 0x62  0x0188  98: Error Correction */
    .long RSV83_IRQHandler          /* 0x63  0x018C  99: Reserved */
    .long RSV84_IRQHandler          /* 0x64  0x0190  100: Reserved */
    .long SCA_IRQHandler            /* 0x65  0x0194  101: SCA Crypto Accelerator */
    .long RSV86_IRQHandler          /* 0x66  0x0198  102: Reserved */
    .long RSV87_IRQHandler          /* 0x67  0x019C  103: Reserved */
    .long UART3_IRQHandler          /* 0x68  0x01A0  104: UART 3 */
    .long RSV89_IRQHandler          /* 0x69  0x01A4  105: Reserved */
    .long RSV90_IRQHandler          /* 0x6A  0x01A8  106: Reserved */
    .long RSV91_IRQHandler          /* 0x6B  0x01AC  107: Reserved */
    .long RSV92_IRQHandler          /* 0x6C  0x01B0  108: Reserved */
    .long HTMR0_IRQHandler          /* 0x6D  0x01B4  109: HTIMER 0 */
    .long HTMR1_IRQHandler          /* 0x6E  0x01B8  110: HTIMER 1 */
    .long RSV95_IRQHandler          /* 0x6F  0x01BC  111: Reserved */
    .long RSV96_IRQHandler          /* 0x70  0x01C0  112: Reserved */
    .long RSV97_IRQHandler          /* 0x71  0x01C4  113: Reserved */
    .long RSV98_IRQHandler          /* 0x72  0x01C8  114: Reserved */
    .long RSV99_IRQHandler          /* 0x73  0x01CC  115: Reserved */
    .long RSV100_IRQHandler         /* 0x74  0x01D0  116: Reserved */
    .long RSV101_IRQHandler         /* 0x75  0x01D4  117: Reserved */
    .long RSV102_IRQHandler         /* 0x76  0x01D8  118: Reserved */
    .long RSV103_IRQHandler         /* 0x77  0x01DC  119: Reserved */
    .long RSV104_IRQHandler         /* 0x78  0x01E0  120: Reserved */
    .long RSV105_IRQHandler         /* 0x79  0x01E4  121: Reserved */
    .long RSV106_IRQHandler         /* 0x7A  0x01E8  122: Reserved */
    .long RSV107_IRQHandler         /* 0x7B  0x01EC  123: Reserved */
    .long RSV108_IRQHandler         /* 0x7C  0x01F0  124: Reserved */
    .long RSV109_IRQHandler         /* 0x7D  0x01F4  125: Reserved */
    .long RSV110_IRQHandler         /* 0x7E  0x01F8  126: Reserved */
    .long RSV111_IRQHandler         /* 0x7F  0x01FC  127: Reserved */
    .text 	
    .thumb
    .thumb_func
    .align 2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    ldr r0, =__StackTop
    mov sp, r0

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

#ifdef __MSR_SDMA_SIZE
/*     Loop to copy data from read only SDMA memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __load_sdma: Where data sections are saved.
 *      _sdma /_esdma: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__load_msr_sdma
    ldr    r2, =_msr_sdma
    ldr    r3, =_emsr_sdma

#if 0
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
.LC0_sdma:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0_sdma
#else
    subs    r3, r2
    ble    .LC1_sdma
.LC0_sdma:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0_sdma
.LC1_sdma:
#endif
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
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    Default_Handler

    /* Device-specific Interrupts */
    def_irq_handler    PF_IRQHandler             /* 0x10  0x0040  16: Power Fail */
    def_irq_handler    WDT0_IRQHandler           /* 0x11  0x0044  17: Watchdog 0 */
    def_irq_handler    USB_IRQHandler            /* 0x12  0x0048  18: USB */
    def_irq_handler    RTC_IRQHandler            /* 0x13  0x004C  19: RTC */
    def_irq_handler    TRNG_IRQHandler           /* 0x14  0x0050  20: True Random Number Generator */
    def_irq_handler    TMR0_IRQHandler           /* 0x15  0x0054  21: Timer 0 */
    def_irq_handler    TMR1_IRQHandler           /* 0x16  0x0058  22: Timer 1 */
    def_irq_handler    TMR2_IRQHandler           /* 0x17  0x005C  23: Timer 2 */
    def_irq_handler    TMR3_IRQHandler           /* 0x18  0x0060  24: Timer 3 */
    def_irq_handler    TMR4_IRQHandler           /* 0x19  0x0064  25: Timer 4 */
    def_irq_handler    TMR5_IRQHandler           /* 0x1A  0x0068  26: Timer 5 */
    def_irq_handler    SC0_IRQHandler            /* 0x1B  0x006C  27: Smart Card 0 */
    def_irq_handler    RSV12_IRQHandler          /* 0x1C  0x0070  28: Reserved */
    def_irq_handler    I2C0_IRQHandler           /* 0x1D  0x0074  29: I2C0 */
    def_irq_handler    UART0_IRQHandler          /* 0x1E  0x0078  30: UART 0 */
    def_irq_handler    UART1_IRQHandler          /* 0x1F  0x007C  31: UART 1 */
    def_irq_handler    SPI0_IRQHandler           /* 0x20  0x0080  32: SPI0 */
    def_irq_handler    SPI1_IRQHandler           /* 0x21  0x0084  33: SPI1 */
    def_irq_handler    RSV18_IRQHandler          /* 0x22  0x0088  34: Reserved */
    def_irq_handler    SKB_IRQHandler            /* 0x23  0x008C  35: Secure Keyboard */
    def_irq_handler    ADC_IRQHandler            /* 0x24  0x0090  36: ADC */
    def_irq_handler    RSV21_IRQHandler          /* 0x25  0x0094  37: Reserved */
    def_irq_handler    MSRADC_IRQHandler         /* 0x26  0x0098  38: Magstripe DSP */
    def_irq_handler    RSV23_IRQHandler          /* 0x27  0x009C  39: Reserved */
    def_irq_handler    GPIO0_IRQHandler          /* 0x28  0x00A0  40: GPIO0 */
    def_irq_handler    GPIO1_IRQHandler          /* 0x29  0x00A4  41: GPIO1 */
    def_irq_handler    GPIO2_IRQHandler          /* 0x2A  0x00A8  42: GPIO2 */
    def_irq_handler    CRYPTO_IRQHandler         /* 0x2B  0x00AC  43: Crypto */
    def_irq_handler    DMA0_IRQHandler           /* 0x2C  0x00B0  44: DMA0 */
    def_irq_handler    DMA1_IRQHandler           /* 0x2D  0x00B4  45: DMA1 */
    def_irq_handler    DMA2_IRQHandler           /* 0x2E  0x00B8  46: DMA2 */
    def_irq_handler    DMA3_IRQHandler           /* 0x2F  0x00BC  47: DMA3 */
    def_irq_handler    RSV32_IRQHandler          /* 0x30  0x00C0  48: Reserved */
    def_irq_handler    RSV33_IRQHandler          /* 0x31  0x00C4  49: Reserved */
    def_irq_handler    UART2_IRQHandler          /* 0x32  0x00C8  50: UART 2 */
    def_irq_handler    RSV35_IRQHandler          /* 0x33  0x00CC  51: Reserved */
    def_irq_handler    I2C1_IRQHandler           /* 0x34  0x00D0  52: I2C1 */
    def_irq_handler    SC1_IRQHandler            /* 0x35  0x00D4  53: Smart Card 1 */
    def_irq_handler    SPIXFC_IRQHandler         /* 0x36  0x00D8  54: SPI execute in place */
    def_irq_handler    RSV39_IRQHandler          /* 0x37  0x00DC  55: Reserved */
    def_irq_handler    RSV40_IRQHandler          /* 0x38  0x00E0  56: Reserved */
    def_irq_handler    RSV41_IRQHandler          /* 0x39  0x00E4  57: Reserved */
    def_irq_handler    RSV42_IRQHandler          /* 0x3A  0x00E8  58: Reserved */
    def_irq_handler    RSV43_IRQHandler          /* 0x3B  0x00EC  59: Reserved */
    def_irq_handler    RSV44_IRQHandler          /* 0x3C  0x00F0  60: Reserved */
    def_irq_handler    RSV45_IRQHandler          /* 0x3D  0x00F4  61: Reserved */
    def_irq_handler    RSV46_IRQHandler          /* 0x3E  0x00F8  62: Reserved */
    def_irq_handler    RSV47_IRQHandler          /* 0x3F  0x00FC  63: Reserved */
    def_irq_handler    RSV48_IRQHandler          /* 0x40  0x0100  64: Reserved */
    def_irq_handler    RSV49_IRQHandler          /* 0x41  0x0104  65: Reserved */
    def_irq_handler    RSV50_IRQHandler          /* 0x42  0x0108  66: Reserved */
    def_irq_handler    RSV51_IRQHandler          /* 0x43  0x010C  67: Reserved */
    def_irq_handler    RSV52_IRQHandler          /* 0x44  0x0110  68: Reserved */
    def_irq_handler    RSV53_IRQHandler          /* 0x45  0x0114  69: Reserved */
    def_irq_handler    GPIOWAKE_IRQHandler       /* 0x46  0x0118  70: GPIO Wakeup */
    def_irq_handler    RSV55_IRQHandler          /* 0x47  0x011C  71: Reserved */
    def_irq_handler    SPI2_IRQHandler           /* 0x48  0x0120  72: SPI2 */
    def_irq_handler    WDT1_IRQHandler           /* 0x49  0x0124  73: Watchdog 1 */
    def_irq_handler    GPIO3_IRQHandler          /* 0x4A  0x0128  74: GPIO3 */
    def_irq_handler    PT_IRQHandler             /* 0x4B  0x012C  75: Pulse train */
    def_irq_handler    RSV60_IRQHandler          /* 0x4C  0x0130  76: Reserved */
    def_irq_handler    RSV61_IRQHandler          /* 0x4D  0x0134  77: Reserved */
    def_irq_handler    RSV62_IRQHandler          /* 0x4E  0x0138  78: Reserved */
    def_irq_handler    RISCV_IRQHandler          /* 0x4F  0x013C  79: RISCV */
    def_irq_handler    RSV64_IRQHandler          /* 0x50  0x0140  80: Reserved */
    def_irq_handler    RSV65_IRQHandler          /* 0x51  0x0144  81: Reserved */
    def_irq_handler    RSV66_IRQHandler          /* 0x52  0x0148  82: Reserved */
    def_irq_handler    RSV67_IRQHandler          /* 0x53  0x014C  83: Reserved */
    def_irq_handler    DMA4_IRQHandler           /* 0x54  0x0150  84: DMA4 */
    def_irq_handler    DMA5_IRQHandler           /* 0x55  0x0154  85: DMA5 */
    def_irq_handler    DMA6_IRQHandler           /* 0x56  0x0158  86: DMA6 */
    def_irq_handler    DMA7_IRQHandler           /* 0x57  0x015C  87: DMA7 */
    def_irq_handler    DMA8_IRQHandler           /* 0x58  0x0160  88: DMA8 */
    def_irq_handler    DMA9_IRQHandler           /* 0x59  0x0164  89: DMA9 */
    def_irq_handler    DMA10_IRQHandler          /* 0x5A  0x0168  90: DMA10 */
    def_irq_handler    DMA11_IRQHandler          /* 0x5B  0x016C  91: DMA11 */
    def_irq_handler    DMA12_IRQHandler          /* 0x5C  0x0170  92: DMA12 */
    def_irq_handler    DMA13_IRQHandler          /* 0x5D  0x0174  93: DMA13 */
    def_irq_handler    DMA14_IRQHandler          /* 0x5E  0x0178  94: DMA14 */
    def_irq_handler    DMA15_IRQHandler          /* 0x5F  0x017C  95: DMA15 */
    def_irq_handler    USBDMA_IRQHandler         /* 0x60  0x0180  96: USBDMA */
    def_irq_handler    RSV81_IRQHandler          /* 0x61  0x0184  97: Reserved */
    def_irq_handler    ECC_IRQHandler            /* 0x62  0x0188  98: Error Correction */
    def_irq_handler    RSV83_IRQHandler          /* 0x63  0x018C  99: Reserved */
    def_irq_handler    RSV84_IRQHandler          /* 0x64  0x0190  100: Reserved */
    def_irq_handler    SCA_IRQHandler            /* 0x65  0x0194  101: SCA Crypto Accelerator */
    def_irq_handler    RSV86_IRQHandler          /* 0x66  0x0198  102: Reserved */
    def_irq_handler    RSV87_IRQHandler          /* 0x67  0x019C  103: Reserved */
    def_irq_handler    UART3_IRQHandler          /* 0x68  0x01A0  104: UART 3 */
    def_irq_handler    RSV89_IRQHandler          /* 0x69  0x01A4  105: Reserved */
    def_irq_handler    RSV90_IRQHandler          /* 0x6A  0x01A8  106: Reserved */
    def_irq_handler    RSV91_IRQHandler          /* 0x6B  0x01AC  107: Reserved */
    def_irq_handler    RSV92_IRQHandler          /* 0x6C  0x01B0  108: Reserved */
    def_irq_handler    HTMR0_IRQHandler          /* 0x6D  0x01B4  109: HTIMER 0 */
    def_irq_handler    HTMR1_IRQHandler          /* 0x6E  0x01B8  110: HTIMER 1 */
    def_irq_handler    RSV95_IRQHandler          /* 0x6F  0x01BC  111: Reserved */
    def_irq_handler    RSV96_IRQHandler          /* 0x70  0x01C0  112: Reserved */
    def_irq_handler    RSV97_IRQHandler          /* 0x71  0x01C4  113: Reserved */
    def_irq_handler    RSV98_IRQHandler          /* 0x72  0x01C8  114: Reserved */
    def_irq_handler    RSV99_IRQHandler          /* 0x73  0x01CC  115: Reserved */
    def_irq_handler    RSV100_IRQHandler         /* 0x74  0x01D0  116: Reserved */
    def_irq_handler    RSV101_IRQHandler         /* 0x75  0x01D4  117: Reserved */
    def_irq_handler    RSV102_IRQHandler         /* 0x76  0x01D8  118: Reserved */
    def_irq_handler    RSV103_IRQHandler         /* 0x77  0x01DC  119: Reserved */
    def_irq_handler    RSV104_IRQHandler         /* 0x78  0x01E0  120: Reserved */
    def_irq_handler    RSV105_IRQHandler         /* 0x79  0x01E4  121: Reserved */
    def_irq_handler    RSV106_IRQHandler         /* 0x7A  0x01E8  122: Reserved */
    def_irq_handler    RSV107_IRQHandler         /* 0x7B  0x01EC  123: Reserved */
    def_irq_handler    RSV108_IRQHandler         /* 0x7C  0x01F0  124: Reserved */
    def_irq_handler    RSV109_IRQHandler         /* 0x7D  0x01F4  125: Reserved */
    def_irq_handler    RSV110_IRQHandler         /* 0x7E  0x01F8  126: Reserved */
    def_irq_handler    RSV111_IRQHandler         /* 0x7F  0x01FC  127: Reserved */
    .end
