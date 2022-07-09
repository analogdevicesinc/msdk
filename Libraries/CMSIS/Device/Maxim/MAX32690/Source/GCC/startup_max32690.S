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

    /* Symbols for the memory segments */
    .equ      _FLASH_SIZE, FLASH_SIZE
    .globl    _FLASH_SIZE
    .equ      _FLASH_ORIGIN, FLASH_ORIGIN
    .globl    _FLASH_ORIGIN
    .equ      _SRAM_SIZE, SRAM_SIZE
    .globl    _SRAM_SIZE
    .equ      _SRAM_ORIGIN, SRAM_ORIGIN
    .globl    _SRAM_ORIGIN

#ifdef RISCV_FLASH_SIZE
    .equ      _RISCV_FLASH_SIZE, RISCV_FLASH_SIZE
#else
    .equ    _RISCV_FLASH_SIZE, 0x0
#endif
    .globl    _RISCV_FLASH_SIZE
#ifdef RISCV_FLASH_ORIGIN
    .equ      _RISCV_FLASH_ORIGIN, RISCV_FLASH_ORIGIN
#else
    .equ    _RISCV_FLASH_ORIGIN, 0x0
#endif
    .globl    _RISCV_FLASH_ORIGIN
#ifdef RISCV_SRAM_SIZE
    .equ      _RISCV_SRAM_SIZE, RISCV_SRAM_SIZE
#else
    .equ    _RISCV_SRAM_SIZE, 0x0
#endif
    .globl    _RISCV_SRAM_SIZE
#ifdef RISCV_SRAM_ORIGIN
    .equ      _RISCV_SRAM_ORIGIN, RISCV_SRAM_ORIGIN
#else
    .equ    _RISCV_SRAM_ORIGIN, 0x0
#endif
    .globl    _RISCV_SRAM_ORIGIN

    /* Flash storage for PAL layer */
#ifdef PAL_NVM_SIZE
    .equ    _PAL_NVM_SIZE, PAL_NVM_SIZE
#else
    .equ    _PAL_NVM_SIZE, 0x0
#endif
    .globl    _PAL_NVM_SIZE

#ifdef MAILBOX_SIZE
    .equ    _MAILBOX_SIZE, MAILBOX_SIZE
#else
    .equ    _MAILBOX_SIZE, 0x0
#endif
    .globl    _MAILBOX_SIZE
    
    .section .stack
    .align 3
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
    .align 2
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
    .long PF_IRQHandler                /* 0x10  0x0040  16: Power Fail */
    .long WDT0_IRQHandler              /* 0x11  0x0044  17: Watchdog 0 */
    .long USB_IRQHandler               /* 0x12  0x0048  18: USB */
    .long RTC_IRQHandler               /* 0x13  0x004C  19: RTC */
    .long TRNG_IRQHandler              /* 0x14  0x0050  20: True Random Number Generator */
    .long TMR0_IRQHandler              /* 0x15  0x0054  21: Timer 0 */
    .long TMR1_IRQHandler              /* 0x16  0x0058  22: Timer 1 */
    .long TMR2_IRQHandler              /* 0x17  0x005C  23: Timer 2 */
    .long TMR3_IRQHandler              /* 0x18  0x0060  24: Timer 3 */
    .long TMR4_IRQHandler              /* 0x19  0x0064  25: Timer 4 */
    .long TMR5_IRQHandler              /* 0x1A  0x0068  26: Timer 5 */
    .long RSV11_IRQHandler             /* 0x1B  0x006C  27: Reserved */
    .long RSV12_IRQHandler             /* 0x1C  0x0070  28: Reserved */
    .long I2C0_IRQHandler              /* 0x1D  0x0074  29: I2C0 */
    .long UART0_IRQHandler             /* 0x1E  0x0078  30: UART 0 */
    .long UART1_IRQHandler             /* 0x1F  0x007C  31: UART 1 */
    .long SPI0_IRQHandler              /* 0x20  0x0080  32: SPI0 */
    .long SPI1_IRQHandler              /* 0x21  0x0084  33: SPI1 */
    .long SPI2_IRQHandler              /* 0x22  0x0088  34: SPI2 */
    .long RSV19_IRQHandler             /* 0x23  0x008C  35: Reserved */
    .long ADC_IRQHandler               /* 0x24  0x0090  36: ADC */
    .long RSV21_IRQHandler             /* 0x25  0x0094  37: Reserved */
    .long RSV22_IRQHandler             /* 0x26  0x0098  38: Reserved */
    .long FLC0_IRQHandler              /* 0x27  0x009C  39: Flash Controller 0 */
    .long GPIO0_IRQHandler             /* 0x28  0x00A0  40: GPIO0 */
    .long GPIO1_IRQHandler             /* 0x29  0x00A4  41: GPIO1 */
    .long GPIO2_IRQHandler             /* 0x2A  0x00A8  42: GPIO2 */
    .long CRYPTO_IRQHandler            /* 0x2B  0x00AC  43: Crypto */
    .long DMA0_IRQHandler              /* 0x2C  0x00B0  44: DMA0 */
    .long DMA1_IRQHandler              /* 0x2D  0x00B4  45: DMA1 */
    .long DMA2_IRQHandler              /* 0x2E  0x00B8  46: DMA2 */
    .long DMA3_IRQHandler              /* 0x2F  0x00BC  47: DMA3 */
    .long RSV32_IRQHandler             /* 0x30  0x00C0  48: Reserved */
    .long RSV33_IRQHandler             /* 0x31  0x00C4  49: Reserved */
    .long UART2_IRQHandler             /* 0x32  0x00C8  50: UART 2 */
    .long RSV35_IRQHandler             /* 0x33  0x00CC  51: Reserved */
    .long I2C1_IRQHandler              /* 0x34  0x00D0  52: I2C1 */
    .long RSV37_IRQHandler             /* 0x35  0x00D4  53: Reserved */
    .long SPIXC_IRQHandler             /* 0x36  0x00D8  54: SPI execute in place */
    .long BTLE_TX_DONE_IRQHandler      /* 0x37  0x00DC  55: BTLE TX Done */
    .long BTLE_RX_RCVD_IRQHandler      /* 0x38  0x00E0  56: BTLE RX Received */
    .long BTLE_RX_ENG_DET_IRQHandler   /* 0x39  0x00E4  57: BTLE RX Energy Detected */
    .long BTLE_SFD_DET_IRQHandler      /* 0x3A  0x00E8  58: BTLE SFD Detected */
    .long BTLE_SFD_TO_IRQHandler       /* 0x3B  0x00EC  59: BTLE SFD Timeout*/
    .long BTLE_GP_EVENT_IRQHandler     /* 0x3C  0x00F0  60: BTLE Timestamp*/
    .long BTLE_CFO_IRQHandler          /* 0x3D  0x00F4  61: BTLE CFO Done */
    .long BTLE_SIG_DET_IRQHandler      /* 0x3E  0x00F8  62: BTLE Signal Detected */
    .long BTLE_AGC_EVENT_IRQHandler    /* 0x3F  0x00FC  63: BTLE AGC Event */
    .long BTLE_RFFE_SPIM_IRQHandler    /* 0x40  0x0100  64: BTLE RFFE SPIM Done */
    .long BTLE_TX_AES_IRQHandler       /* 0x41  0x0104  65: BTLE TX AES Done */
    .long BTLE_RX_AES_IRQHandler       /* 0x42  0x0108  66: BTLE RX AES Done */
    .long BTLE_INV_APB_ADDR_IRQHandler /* 0x43  0x010C  67: BTLE Invalid APB Address*/
    .long BTLE_IQ_DATA_VALID_IRQHandler/* 0x44  0x0110  68: BTLE IQ Data Valid */
    .long WUT0_IRQHandler              /* 0x45  0x0114  69: Wake up timer 0 */
    .long GPIOWAKE_IRQHandler          /* 0x46  0x0118  70: GPIO Wakeup */
    .long RSV55_IRQHandler             /* 0x47  0x011C  71: Reserved */
    .long SPI3_IRQHandler              /* 0x48  0x0120  72: SPI3 */
    .long WDT1_IRQHandler              /* 0x49  0x0124  73: Watchdog 1 */
    .long GPIO3_IRQHandler             /* 0x4A  0x0128  74: GPIO3 */
    .long PT_IRQHandler                /* 0x4B  0x012C  75: Pulse Train */
    .long RSV60_IRQHandler             /* 0x4C  0x0130  76: Reserved */
    .long HPB_IRQHandler               /* 0x4D  0x0134  77: Hyperbus */
    .long I2C2_IRQHandler              /* 0x4E  0x0138  78: I2C 2 */
    .long RISCV_IRQHandler             /* 0x4F  0x013C  79: RISCV */
    .long RSV64_IRQHandler             /* 0x50  0x0140  80: Reserved */
    .long RSV65_IRQHandler             /* 0x51  0x0144  81: Reserved */
    .long RSV66_IRQHandler             /* 0x52  0x0148  82: Reserved*/
    .long OWM_IRQHandler               /* 0x53  0x014C  83: One Wire Master */
    .long DMA4_IRQHandler              /* 0x54  0x0150  84: DMA4 */
    .long DMA5_IRQHandler              /* 0x55  0x0154  85: DMA5 */
    .long DMA6_IRQHandler              /* 0x56  0x0158  86: DMA6 */
    .long DMA7_IRQHandler              /* 0x57  0x015C  87: DMA7 */
    .long DMA8_IRQHandler              /* 0x58  0x0160  88: DMA8 */
    .long DMA9_IRQHandler              /* 0x59  0x0164  89: DMA9 */
    .long DMA10_IRQHandler             /* 0x5A  0x0168  90: DMA10 */
    .long DMA11_IRQHandler             /* 0x5B  0x016C  91: DMA11 */
    .long DMA12_IRQHandler             /* 0x5C  0x0170  92: DMA12 */
    .long DMA13_IRQHandler             /* 0x5D  0x0174  93: DMA13 */
    .long DMA14_IRQHandler             /* 0x5E  0x0178  94: DMA14 */
    .long DMA15_IRQHandler             /* 0x5F  0x017C  95: DMA15 */
    .long USBDMA_IRQHandler            /* 0x60  0x0180  96: USB DMA */
    .long RSV81_IRQHandler             /* 0x61  0x0184  97: Reserved */
    .long ECC_IRQHandler               /* 0x62  0x0188  98: ECC */
    .long RSV83_IRQHandler             /* 0x63  0x018C  99: Reserved */
    .long RSV84_IRQHandler             /* 0x64  0x0190  100: Reserved */
    .long SCA_IRQHandler               /* 0x65  0x0194  101: SCA Crypto Accelerator */
    .long RSV86_IRQHandler             /* 0x66  0x0198  102: Reserved */
    .long FLC1_IRQHandler              /* 0x67  0x019C  103: Flash Controller 1 */
    .long UART3_IRQHandler             /* 0x68  0x01A0  104: UART 3 */
    .long RSV89_IRQHandler             /* 0x69  0x01A4  105: Reserved */
    .long RSV90_IRQHandler             /* 0x6A  0x01A8  106: Reserved */
    .long RSV91_IRQHandler             /* 0x6B  0x01AC  107: Reserved */
    .long RSV92_IRQHandler             /* 0x6C  0x01B0  108: Reserved */
    .long RSV93_IRQHandler             /* 0x6D  0x01B4  109: Reserved */
    .long RSV94_IRQHandler             /* 0x6E  0x01B8  110: Reserved */
    .long PUF_IRQHandler               /* 0x6F  0x01BC  111: Physically Uncloneable Function */
    .long RSV96_IRQHandler             /* 0x70  0x01C0  112: Reserved */
    .long RSV97_IRQHandler             /* 0x71  0x01C4  113: Reserved */
    .long RSV98_IRQHandler             /* 0x72  0x01C8  114: Reserved */
    .long I2S_IRQHandler               /* 0x73  0x01CC  115: I2S */    
    .long RSV100_IRQHandler            /* 0x74  0x01D0  116: Reserved */
    .long RSV101_IRQHandler            /* 0x75  0x01D4  117: Reserved */
    .long RSV102_IRQHandler            /* 0x76  0x01D8  118: Reserved */
    .long LPCMP_IRQHandler             /* 0x77  0x01DC  119: LPCMP */    
    .long RSV104_IRQHandler            /* 0x78  0x01E0  120: Reserved */
    .long SPI4_IRQHandler              /* 0x79  0x01E4  121: SPI4 */
    .long RSV106_IRQHandler            /* 0x7A  0x01E8  122: Reserved */
    .long CAN0_IRQHandler              /* 0x7B  0x01EC  123: CAN0 */    
    .long CAN1_IRQHandler              /* 0x7C  0x01F0  124: CAN1 */
    .long WUT1_IRQHandler              /* 0x7D  0x01F4  125: Wake up timer 1 */ 
    .long RSV111_IRQHandler            /* 0x7E  0x01F8  126: Reserved */
    .long RSV112_IRQHandler            /* 0x7F  0x01FC  127: Reserved */ 
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
    def_irq_handler PF_IRQHandler                /* 0x10  0x0040  16: Power Fail */
    def_irq_handler WDT0_IRQHandler              /* 0x11  0x0044  17: Watchdog 0 */
    def_irq_handler USB_IRQHandler               /* 0x12  0x0048  18: USB */
    def_irq_handler RTC_IRQHandler               /* 0x13  0x004C  19: RTC */
    def_irq_handler TRNG_IRQHandler              /* 0x14  0x0050  20: True Random Number Generator */
    def_irq_handler TMR0_IRQHandler              /* 0x15  0x0054  21: Timer 0 */
    def_irq_handler TMR1_IRQHandler              /* 0x16  0x0058  22: Timer 1 */
    def_irq_handler TMR2_IRQHandler              /* 0x17  0x005C  23: Timer 2 */
    def_irq_handler TMR3_IRQHandler              /* 0x18  0x0060  24: Timer 3 */
    def_irq_handler TMR4_IRQHandler              /* 0x19  0x0064  25: Timer 4 */
    def_irq_handler TMR5_IRQHandler              /* 0x1A  0x0068  26: Timer 5 */
    def_irq_handler RSV11_IRQHandler             /* 0x1B  0x006C  27: Reserved */
    def_irq_handler RSV12_IRQHandler             /* 0x1C  0x0070  28: Reserved */
    def_irq_handler I2C0_IRQHandler              /* 0x1D  0x0074  29: I2C0 */
    def_irq_handler UART0_IRQHandler             /* 0x1E  0x0078  30: UART 0 */
    def_irq_handler UART1_IRQHandler             /* 0x1F  0x007C  31: UART 1 */
    def_irq_handler SPI0_IRQHandler              /* 0x20  0x0080  32: SPI0 */
    def_irq_handler SPI1_IRQHandler              /* 0x21  0x0084  33: SPI1 */
    def_irq_handler SPI2_IRQHandler              /* 0x22  0x0088  34: SPI2 */
    def_irq_handler RSV19_IRQHandler             /* 0x23  0x008C  35: Reserved */
    def_irq_handler ADC_IRQHandler               /* 0x24  0x0090  36: ADC */
    def_irq_handler RSV21_IRQHandler             /* 0x25  0x0094  37: Reserved */
    def_irq_handler RSV22_IRQHandler             /* 0x26  0x0098  38: Reserved */
    def_irq_handler FLC0_IRQHandler              /* 0x27  0x009C  39: Flash Controller 0 */
    def_irq_handler GPIO0_IRQHandler             /* 0x28  0x00A0  40: GPIO0 */
    def_irq_handler GPIO1_IRQHandler             /* 0x29  0x00A4  41: GPIO1 */
    def_irq_handler GPIO2_IRQHandler             /* 0x2A  0x00A8  42: GPIO2 */
    def_irq_handler CRYPTO_IRQHandler            /* 0x2B  0x00AC  43: Crypto */
    def_irq_handler DMA0_IRQHandler              /* 0x2C  0x00B0  44: DMA0 */
    def_irq_handler DMA1_IRQHandler              /* 0x2D  0x00B4  45: DMA1 */
    def_irq_handler DMA2_IRQHandler              /* 0x2E  0x00B8  46: DMA2 */
    def_irq_handler DMA3_IRQHandler              /* 0x2F  0x00BC  47: DMA3 */
    def_irq_handler RSV32_IRQHandler             /* 0x30  0x00C0  48: Reserved */
    def_irq_handler RSV33_IRQHandler             /* 0x31  0x00C4  49: Reserved */
    def_irq_handler UART2_IRQHandler             /* 0x32  0x00C8  50: UART 2 */
    def_irq_handler RSV35_IRQHandler             /* 0x33  0x00CC  51: Reserved */
    def_irq_handler I2C1_IRQHandler              /* 0x34  0x00D0  52: I2C1 */
    def_irq_handler RSV37_IRQHandler             /* 0x35  0x00D4  53: Reserved */
    def_irq_handler SPIXC_IRQHandler             /* 0x36  0x00D8  54: SPI execute in place */
    def_irq_handler BTLE_TX_DONE_IRQHandler      /* 0x37  0x00DC  55: BTLE TX Done */
    def_irq_handler BTLE_RX_RCVD_IRQHandler      /* 0x38  0x00E0  56: BTLE RX Received */
    def_irq_handler BTLE_RX_ENG_DET_IRQHandler   /* 0x39  0x00E4  57: BTLE RX Energy Detected */
    def_irq_handler BTLE_SFD_DET_IRQHandler      /* 0x3A  0x00E8  58: BTLE SFD Detected */
    def_irq_handler BTLE_SFD_TO_IRQHandler       /* 0x3B  0x00EC  59: BTLE SFD Timeout*/
    def_irq_handler BTLE_GP_EVENT_IRQHandler     /* 0x3C  0x00F0  60: BTLE Timestamp*/
    def_irq_handler BTLE_CFO_IRQHandler          /* 0x3D  0x00F4  61: BTLE CFO Done */
    def_irq_handler BTLE_SIG_DET_IRQHandler      /* 0x3E  0x00F8  62: BTLE Signal Detected */
    def_irq_handler BTLE_AGC_EVENT_IRQHandler    /* 0x3F  0x00FC  63: BTLE AGC Event */
    def_irq_handler BTLE_RFFE_SPIM_IRQHandler    /* 0x40  0x0100  64: BTLE RFFE SPIM Done */
    def_irq_handler BTLE_TX_AES_IRQHandler       /* 0x41  0x0104  65: BTLE TX AES Done */
    def_irq_handler BTLE_RX_AES_IRQHandler       /* 0x42  0x0108  66: BTLE RX AES Done */
    def_irq_handler BTLE_INV_APB_ADDR_IRQHandler /* 0x43  0x010C  67: BTLE Invalid APB Address*/
    def_irq_handler BTLE_IQ_DATA_VALID_IRQHandler/* 0x44  0x0110  68: BTLE IQ Data Valid */
    def_irq_handler WUT0_IRQHandler              /* 0x45  0x0114  69: Wake up timer 0 */
    def_irq_handler GPIOWAKE_IRQHandler          /* 0x46  0x0118  70: GPIO Wakeup */
    def_irq_handler RSV55_IRQHandler             /* 0x47  0x011C  71: Reserved */
    def_irq_handler SPI3_IRQHandler              /* 0x48  0x0120  72: SPI3 */
    def_irq_handler WDT1_IRQHandler              /* 0x49  0x0124  73: Watchdog 1 */
    def_irq_handler GPIO3_IRQHandler             /* 0x4A  0x0128  74: GPIO3 */
    def_irq_handler PT_IRQHandler                /* 0x4B  0x012C  75: Pulse Train */
    def_irq_handler RSV60_IRQHandler             /* 0x4C  0x0130  76: Reserved */
    def_irq_handler HPB_IRQHandler               /* 0x4D  0x0134  77: Hyperbus */
    def_irq_handler I2C2_IRQHandler              /* 0x4E  0x0138  78: I2C 2 */
    def_irq_handler RISCV_IRQHandler             /* 0x4F  0x013C  79: RISCV */
    def_irq_handler RSV64_IRQHandler             /* 0x50  0x0140  80: Reserved */
    def_irq_handler RSV65_IRQHandler             /* 0x51  0x0144  81: Reserved */
    def_irq_handler RSV66_IRQHandler             /* 0x52  0x0148  82: Reserved*/
    def_irq_handler OWM_IRQHandler               /* 0x53  0x014C  83: One Wire Master */
    def_irq_handler DMA4_IRQHandler              /* 0x54  0x0150  84: DMA4 */
    def_irq_handler DMA5_IRQHandler              /* 0x55  0x0154  85: DMA5 */
    def_irq_handler DMA6_IRQHandler              /* 0x56  0x0158  86: DMA6 */
    def_irq_handler DMA7_IRQHandler              /* 0x57  0x015C  87: DMA7 */
    def_irq_handler DMA8_IRQHandler              /* 0x58  0x0160  88: DMA8 */
    def_irq_handler DMA9_IRQHandler              /* 0x59  0x0164  89: DMA9 */
    def_irq_handler DMA10_IRQHandler             /* 0x5A  0x0168  90: DMA10 */
    def_irq_handler DMA11_IRQHandler             /* 0x5B  0x016C  91: DMA11 */
    def_irq_handler DMA12_IRQHandler             /* 0x5C  0x0170  92: DMA12 */
    def_irq_handler DMA13_IRQHandler             /* 0x5D  0x0174  93: DMA13 */
    def_irq_handler DMA14_IRQHandler             /* 0x5E  0x0178  94: DMA14 */
    def_irq_handler DMA15_IRQHandler             /* 0x5F  0x017C  95: DMA15 */
    def_irq_handler USBDMA_IRQHandler            /* 0x60  0x0180  96: USB DMA */
    def_irq_handler RSV81_IRQHandler             /* 0x61  0x0184  97: Reserved */
    def_irq_handler ECC_IRQHandler               /* 0x62  0x0188  98: ECC */
    def_irq_handler RSV83_IRQHandler             /* 0x63  0x018C  99: Reserved */
    def_irq_handler RSV84_IRQHandler             /* 0x64  0x0190  100: Reserved */
    def_irq_handler SCA_IRQHandler               /* 0x65  0x0194  101: SCA Crypto Accelerator */
    def_irq_handler RSV86_IRQHandler             /* 0x66  0x0198  102: Reserved */
    def_irq_handler FLC1_IRQHandler              /* 0x67  0x019C  103: Flash Controller 1 */
    def_irq_handler UART3_IRQHandler             /* 0x68  0x01A0  104: UART 3 */
    def_irq_handler RSV89_IRQHandler             /* 0x69  0x01A4  105: Reserved */
    def_irq_handler RSV90_IRQHandler             /* 0x6A  0x01A8  106: Reserved */
    def_irq_handler RSV91_IRQHandler             /* 0x6B  0x01AC  107: Reserved */
    def_irq_handler RSV92_IRQHandler             /* 0x6C  0x01B0  108: Reserved */
    def_irq_handler RSV93_IRQHandler             /* 0x6D  0x01B4  109: Reserved */
    def_irq_handler RSV94_IRQHandler             /* 0x6E  0x01B8  110: Reserved */
    def_irq_handler PUF_IRQHandler               /* 0x6F  0x01BC  111: Physically Uncloneable Function */
    def_irq_handler RSV96_IRQHandler             /* 0x70  0x01C0  112: Reserved */
    def_irq_handler RSV97_IRQHandler             /* 0x71  0x01C4  113: Reserved */
    def_irq_handler RSV98_IRQHandler             /* 0x72  0x01C8  114: Reserved */
    def_irq_handler I2S_IRQHandler               /* 0x73  0x01CC  115: I2S */    
    def_irq_handler RSV100_IRQHandler            /* 0x74  0x01D0  116: Reserved */
    def_irq_handler RSV101_IRQHandler            /* 0x75  0x01D4  117: Reserved */
    def_irq_handler RSV102_IRQHandler            /* 0x76  0x01D8  118: Reserved */
    def_irq_handler LPCMP_IRQHandler             /* 0x77  0x01DC  119: LPCMP */    
    def_irq_handler RSV104_IRQHandler            /* 0x78  0x01E0  120: Reserved */
    def_irq_handler SPI4_IRQHandler              /* 0x79  0x01E4  121: SPI4 */
    def_irq_handler RSV106_IRQHandler            /* 0x7A  0x01E8  122: Reserved */
    def_irq_handler CAN0_IRQHandler              /* 0x7B  0x01EC  123: CAN0 */    
    def_irq_handler CAN1_IRQHandler              /* 0x7C  0x01F0  124: CAN1 */
    def_irq_handler WUT1_IRQHandler              /* 0x7D  0x01F4  125: Wake up timer 1 */ 
    def_irq_handler RSV111_IRQHandler            /* 0x7E  0x01F8  126: Reserved */
    def_irq_handler RSV112_IRQHandler            /* 0x7F  0x01FC  127: Reserved */     
    .end