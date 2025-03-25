;*******************************************************************************
;* Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
;*
;* Permission is hereby granted, free of charge, to any person obtaining a
;* copy of this software and associated documentation files (the "Software"),
;* to deal in the Software without restriction, including without limitation
;* the rights to use, copy, modify, merge, publish, distribute, sublicense,
;* and/or sell copies of the Software, and to permit persons to whom the
;* Software is furnished to do so, subject to the following conditions:
;*
;* The above copyright notice and this permission notice shall be included
;* in all copies or substantial portions of the Software.
;*
;* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
;* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
;* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
;* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
;* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
;* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
;* OTHER DEALINGS IN THE SOFTWARE.
;*
;* Except as contained in this notice, the name of Maxim Integrated
;* Products, Inc. shall not be used except as stated in the Maxim Integrated
;* Products, Inc. Branding Policy.
;*
;* The mere transfer of this software does not imply any licenses
;* of trade secrets, proprietary technology, copyrights, patents,
;* trademarks, maskwork rights, or any other form of intellectual
;* property whatsoever. Maxim Integrated Products, Inc. retains all
;* ownership rights.
;*
;* Description        : MAX32655 device vector table for IAR EWARM toolchain.
;*                      - Sets the initial SP
;*                      - Sets the initial PC == _iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR
;*                        address, all set as PUBWEAK. User may override any ISR
;*                        defined as PUBWEAK.
;*                      - Branches to main in the C library (which eventually
;*                        calls SystemInit() and main()).
;*                      After Reset the Cortex-M4 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* $Date: 2019-07-30 14:08:36 -0500 (Tue, 30 Jul 2019) $
;* $Revision: 45136 $
;*
;*******************************************************************************
    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3)

    SECTION .intvec:CODE:NOROOT(2)

    EXTERN  __iar_program_start
    EXTERN  SysTick_Handler
    EXTERN  __ICFEDIT_intvec_start__
    PUBLIC  __vector_table
    PUBLIC  __vector_table_modify
    PUBLIC  __Vectors
    PUBLIC  __Vectors_End
    PUBLIC  __Vectors_Size
    PUBLIC  __isr_vector

    DATA
__vector_table
    DCD     sfe(CSTACK)
    DCD    Reset_Handler         ; Reset Handler 
    DCD    NMI_Handler           ; NMI Handler 
    DCD    HardFault_Handler     ; Hard Fault Handler 
    DCD    MemManage_Handler     ; MPU Fault Handler 
    DCD    BusFault_Handler      ; Bus Fault Handler 
    DCD    UsageFault_Handler    ; Usage Fault Handler 
    DCD    0                     ; Reserved 
    DCD    0                     ; Reserved 
    DCD    0                     ; Reserved 
    DCD    0                     ; Reserved 
    DCD    SVC_Handler           ; SVCall Handler 
    DCD    DebugMon_Handler      ; Debug Montor Handler
    DCD    0                     ; Reserved 
    DCD    PendSV_Handler        ; PendSV Handler 
    DCD    SysTick_Handler       ; SysTick Handler 
__vector_table_modify
    ; Device-specific Interrupts 
    DCD    PF_IRQHandler                 ; 0x10  0x0040  16: Power Fail 
    DCD    WDT0_IRQHandler               ; 0x11  0x0044  17: Watchdog 0 
    DCD    RSV02_IRQHandler              ; 0x12  0x0048  18: Reserved 
    DCD    RTC_IRQHandler                ; 0x13  0x004C  19: RTC 
    DCD    TRNG_IRQHandler               ; 0x14  0x0050  20: True Random Number Generator 
    DCD    TMR0_IRQHandler               ; 0x15  0x0054  21: Timer 0 
    DCD    TMR1_IRQHandler               ; 0x16  0x0058  22: Timer 1 
    DCD    TMR2_IRQHandler               ; 0x17  0x005C  23: Timer 2 
    DCD    TMR3_IRQHandler               ; 0x18  0x0060  24: Timer 3
    DCD    TMR4_IRQHandler               ; 0x19  0x0064  25: Timer 4
    DCD    TMR5_IRQHandler               ; 0x1A  0x0068  26: Timer 5 
    DCD    RSV11_IRQHandler              ; 0x1B  0x006C  27: Reserved 
    DCD    RSV12_IRQHandler              ; 0x1C  0x0070  28: Reserved 
    DCD    I2C0_IRQHandler               ; 0x1D  0x0074  29: I2C0
    DCD    UART0_IRQHandler              ; 0x1E  0x0078  30: UART 0 
    DCD    UART1_IRQHandler              ; 0x1F  0x007C  31: UART 1 
    DCD    SPI1_IRQHandler               ; 0x20  0x0080  32: SPI1
    DCD    RSV17_IRQHandler              ; 0x21  0x0084  33: Reserved */ 
    DCD    RSV18_IRQHandler              ; 0x22  0x0088  34: Reserved */ 
    DCD    RSV19_IRQHandler              ; 0x23  0x008C  35: Reserved 
    DCD    ADC_IRQHandler                ; 0x24  0x0090  36: ADC
    DCD    RSV21_IRQHandler              ; 0x25  0x0094  37: Reserved 
    DCD    RSV22_IRQHandler              ; 0x26  0x0098  38: Reserved
    DCD    FLC0_IRQHandler               ; 0x27  0x009C  39: Flash Controller 0
    DCD    GPIO0_IRQHandler              ; 0x28  0x00A0  40: GPIO0 
    DCD    GPIO1_IRQHandler              ; 0x29  0x00A4  41: GPIO1 
    DCD    GPIO2_IRQHandler              ; 0x2A  0x00A8  42: GPIO2
    DCD    RSV27_IRQHandler              ; 0x2B  0x00AC  43: Reserved
    DCD    DMA0_IRQHandler               ; 0x2C  0x00B0  44: DMA0 
    DCD    DMA1_IRQHandler               ; 0x2D  0x00B4  45: DMA1 
    DCD    DMA2_IRQHandler               ; 0x2E  0x00B8  46: DMA2 
    DCD    DMA3_IRQHandler               ; 0x2F  0x00BC  47: DMA3 
    DCD    RSV32_IRQHandler              ; 0x30  0x00C0  48: Reserved 
    DCD    RSV33_IRQHandler              ; 0x31  0x00C4  49: Reserved 
    DCD    UART2_IRQHandler              ; 0x32  0x00C8  50: UART 2 
    DCD    RSV35_IRQHandler              ; 0x33  0x00CC  51: Reserved 
    DCD    I2C1_IRQHandler               ; 0x34  0x00D0  52: I2C1 
    DCD    RSV37_IRQHandler              ; 0x35  0x00D4  53: Reserved 
    DCD    RSV38_IRQHandler              ; 0x36  0x00D8  54: Reserved 
    DCD    BTLE_TX_DONE_IRQHandler       ; 0x37  0x00DC  55: BTLE_TX_DONE_IRQHandler 
    DCD    BTLE_RX_RCVD_IRQHandler       ; 0x38  0x00E0  56: BTLE_RX_RCVD_IRQHandler 
    DCD    BTLE_RX_ENG_DET_IRQHandler    ; 0x39  0x00E4  57: BTLE_RX_ENG_DET_IRQHandler 
    DCD    BTLE_SFD_DET_IRQHandler       ; 0x3A  0x00E8  58: BTLE_SFD_DET_IRQHandler 
    DCD    BTLE_SFD_TO_IRQHandler        ; 0x3B  0x00EC  59: BTLE_SFD_TO_IRQHandler
    DCD    BTLE_GP_EVENT_IRQHandler      ; 0x3C  0x00F0  60: BTLE_GP_EVENT_IRQHandler
    DCD    BTLE_CFO_IRQHandler           ; 0x3D  0x00F4  61: BTLE_CFO_IRQHandler 
    DCD    BTLE_SIG_DET_IRQHandler       ; 0x3E  0x00F8  62: BTLE_SIG_DET_IRQHandler 
    DCD    BTLE_AGC_EVENT_IRQHandler     ; 0x3F  0x00FC  63: BTLE_AGC_EVENT_IRQHandler 
    DCD    BTLE_RFFE_SPIM_IRQHandler     ; 0x40  0x0100  64: BTLE_RFFE_SPIM_IRQHandler  
    DCD    BTLE_TX_AES_IRQHandler        ; 0x41  0x0104  65: BTLE_TX_AES_IRQHandler 
    DCD    BTLE_RX_AES_IRQHandler        ; 0x42  0x0108  66: BTLE_RX_AES_IRQHandler 
    DCD    BTLE_INV_APB_ADDR_IRQHandler  ; 0x43  0x010C  67: BTLE_INV_APB_ADDR_IRQHandler
    DCD    BTLE_IQ_DATA_VALID_IRQHandler ; 0x44  0x0110  68: BTLE_IQ_DATA_VALID_IRQHandler
    DCD    WUT_IRQHandler                ; 0x45  0x0114  69: Wake up Timer 
    DCD    GPIOWAKE_IRQHandler           ; 0x46  0x0118  70: GPIO Wakeup
    DCD    RSV55_IRQHandler              ; 0x47  0x011C  71: Reserved 
    DCD    SPI0_IRQHandler               ; 0x48  0x0120  72: SPI 0 
    DCD    WDT1_IRQHandler               ; 0x49  0x0124  73: Watchdog 1 
    DCD    RSV57_IRQHandler              ; 0x4A  0x0128  74: Reserved 
    DCD    PT_IRQHandler                 ; 0x4B  0x012C  75: Pulse Train
    DCD    RSV59_IRQHandler              ; 0x4C  0x0130  76: Reserved 
    DCD    RSV61_IRQHandler              ; 0x4D  0x0134  77: Reserved 
    DCD    I2C2_IRQHandler               ; 0x4E  0x0138  78: I2C 2 
    DCD    SDMA2_IRQHandler              ; 0x4F  0x013C  79: SDMA
    DCD    RSV64_IRQHandler              ; 0x50  0x0140  80: Reserved
    DCD    RSV65_IRQHandler              ; 0x51  0x0144  81: Reserved 
    DCD    RSV66_IRQHandler              ; 0x52  0x0148  82: Reserved 
    DCD    OWM_IRQHandler                ; 0x53  0x014C  83: OWM
    DCD    RSV68_IRQHandler              ; 0x54  0x0150  84: Reserved 
    DCD    RSV69_IRQHandler              ; 0x55  0x0154  85: Reserved 
    DCD    RSV70_IRQHandler              ; 0x56  0x0158  86: Reserved 
    DCD    RSV71_IRQHandler              ; 0x57  0x015C  87: Reserved 
    DCD    RSV72_IRQHandler              ; 0x58  0x0160  88: Reserved 
    DCD    RSV73_IRQHandler              ; 0x59  0x0164  89: Reserved 
    DCD    RSV74_IRQHandler              ; 0x5A  0x0168  90: Reserved
    DCD    RSV75_IRQHandler              ; 0x5B  0x016C  91: Reserved
    DCD    RSV76_IRQHandler              ; 0x5C  0x0170  92: Reserved 
    DCD    RSV77_IRQHandler              ; 0x5D  0x0174  93: Reserved 
    DCD    RSV78_IRQHandler              ; 0x5E  0x0178  94: Reserved 
    DCD    RSV79_IRQHandler              ; 0x5F  0x017C  95: Reserved 
    DCD    RSV80_IRQHandler              ; 0x60  0x0180  96: Reserved 
    DCD    RSV81_IRQHandler              ; 0x61  0x0184  97: Reserved
    DCD    ECC_IRQHandler                ; 0x62  0x0188  98: ECC
    DCD    DVS_IRQHandler                ; 0x63  0x018C  99: DVS 
    DCD    SIMO_IRQHandler               ; 0x64 0x0190   100: SIMO
    DCD    RSV85_IRQHandler              ; 0x65  0x0194  101: Reserved
    DCD    RSV86_IRQHandler              ; 0x66  0x0198  102: Reserved
    DCD    RSV87_IRQHandler              ; 0x67  0x019C  103: Reserved
    DCD    UART3_IRQHandler              ; 0x68  0x01A0  104: UART 3 
    DCD    RSV89_IRQHandler              ; 0x69  0x01A4  105: Reserved 
    DCD    RSV90_IRQHandler              ; 0x6A  0x01A8  106: Reserved 
    DCD    RSV91_IRQHandler              ; 0x6B  0x01AC  107: Reserved 
    DCD    RSV92_IRQHandler              ; 0x6C  0x01B0  108: Reserved  
    DCD    RSV93_IRQHandler              ; 0x6D  0x01B1  109: Reserved 
    DCD    RSV94_IRQHandler              ; 0x6E  0x01B2  110: Reserved
    DCD    RSV95_IRQHandler              ; 0x6D  0x01B1  111: Reserved 
    DCD    RSV96_IRQHandler              ; 0x6E  0x01B2  112: Reserved
    DCD    AES_IRQHandler                ; 0x6D  0x01B1  113: AES  
    DCD    CRC_IRQHandler                ; 0x6E  0x01B2  114: CRC
    DCD    I2S_IRQHandler                ; 0x6E  0x01B2  115: I2S


__Vectors_End
__isr_vector    EQU   __vector_table
__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors



    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
    THUMB
    PUBWEAK Reset_Handler
Reset_Handler



; SystemInit will be called as part of the __iar_program_start flow prior to
; the call to main. It needs to be called after static data/variable initialization
; which occurs during the __iar_program_start definition.
; see project file cmain.s for the IAR specific startup flow and call to SystemInit
        ; Set the initial stack pointer
        LDR        R0, =__vector_table 
        LDR        R1, [R0]
        MOV        SP, R1
        ; jump into IAR program flow
        LDR        R0, =__iar_program_start
        BX         R0

def_irq_handler MACRO handler_name
        PUBWEAK handler_name
        Section .text:CODE:REORDER:NOROOT(1)
handler_name
        B handler_name
        ENDM

    def_irq_handler    NMI_Handler                   ; NMI Handler 
    def_irq_handler    HardFault_Handler             ; Hard Fault Handler 
    def_irq_handler    MemManage_Handler             ; MPU Fault Handler 
    def_irq_handler    BusFault_Handler              ; Bus Fault Handler 
    def_irq_handler    UsageFault_Handler            ; Usage Fault Handler 
    def_irq_handler    SVC_Handler                   ; SVCall Handler
    def_irq_handler    DebugMon_Handler              ; DebugMon Handler 
    def_irq_handler    PendSV_Handler                ; PendSV Handler 
//    def_irq_handler    SysTick_Handler               ; SysTick Handler
    def_irq_handler    PF_IRQHandler                 ; 0x10  0x0040  16: Power Fail 
    def_irq_handler    WDT0_IRQHandler               ; 0x11  0x0044  17: Watchdog 0 
    def_irq_handler    RSV02_IRQHandler              ; 0x12  0x0048  18: Reserved 
    def_irq_handler    RTC_IRQHandler                ; 0x13  0x004C  19: RTC 
    def_irq_handler    TRNG_IRQHandler               ; 0x14  0x0050  20: True Random Number Generator 
    def_irq_handler    TMR0_IRQHandler               ; 0x15  0x0054  21: Timer 0 
    def_irq_handler    TMR1_IRQHandler               ; 0x16  0x0058  22: Timer 1 
    def_irq_handler    TMR2_IRQHandler               ; 0x17  0x005C  23: Timer 2 
    def_irq_handler    TMR3_IRQHandler               ; 0x18  0x0060  24: Timer 3
    def_irq_handler    TMR4_IRQHandler               ; 0x19  0x0064  25: Timer 4
    def_irq_handler    TMR5_IRQHandler               ; 0x1A  0x0068  26: Timer 5 
    def_irq_handler    RSV11_IRQHandler              ; 0x1B  0x006C  27: Reserved 
    def_irq_handler    RSV12_IRQHandler              ; 0x1C  0x0070  28: Reserved 
    def_irq_handler    I2C0_IRQHandler               ; 0x1D  0x0074  29: I2C0
    def_irq_handler    UART0_IRQHandler              ; 0x1E  0x0078  30: UART 0 
    def_irq_handler    UART1_IRQHandler              ; 0x1F  0x007C  31: UART 1 
    def_irq_handler    SPI1_IRQHandler               ; 0x20  0x0080  32: SPI 1
    def_irq_handler    RSV17_IRQHandler              ; 0x21  0x0084  33: Reserved 
    def_irq_handler    RSV18_IRQHandler              ; 0x22  0x0088  34: Reserved 
    def_irq_handler    RSV19_IRQHandler              ; 0x23  0x008C  35: Reserved 
    def_irq_handler    ADC_IRQHandler                ; 0x24  0x0090  36: ADC 
    def_irq_handler    RSV21_IRQHandler              ; 0x25  0x0094  37: Reserved 
    def_irq_handler    RSV22_IRQHandler              ; 0x26  0x0098  38: Reserved
    def_irq_handler    FLC0_IRQHandler               ; 0x27  0x009C  39: Flash Controller 0
    def_irq_handler    GPIO0_IRQHandler              ; 0x28  0x00A0  40: GPIO0 
    def_irq_handler    GPIO1_IRQHandler              ; 0x29  0x00A4  41: GPIO1 
    def_irq_handler    GPIO2_IRQHandler              ; 0x2A  0x00A8  42: GPIO2
    def_irq_handler    RSV27_IRQHandler              ; 0x2B  0x00AC  43: Reserved
    def_irq_handler    DMA0_IRQHandler               ; 0x2C  0x00B0  44: DMA0 
    def_irq_handler    DMA1_IRQHandler               ; 0x2D  0x00B4  45: DMA1 
    def_irq_handler    DMA2_IRQHandler               ; 0x2E  0x00B8  46: DMA2 
    def_irq_handler    DMA3_IRQHandler               ; 0x2F  0x00BC  47: DMA3 
    def_irq_handler    RSV32_IRQHandler              ; 0x30  0x00C0  48: Reserved 
    def_irq_handler    RSV33_IRQHandler              ; 0x31  0x00C4  49: Reserved 
    def_irq_handler    UART2_IRQHandler              ; 0x32  0x00C8  50: UART 2 
    def_irq_handler    RSV35_IRQHandler              ; 0x33  0x00CC  51: Reserved 
    def_irq_handler    I2C1_IRQHandler               ; 0x34  0x00D0  52: I2C1 
    def_irq_handler    RSV37_IRQHandler              ; 0x35  0x00D4  53: Reserved 
    def_irq_handler    RSV38_IRQHandler              ; 0x36  0x00D8  54: Reserved 
    def_irq_handler    BTLE_TX_DONE_IRQHandler       ; 0x37  0x00DC  55: BTLE TX Don 
    def_irq_handler    BTLE_RX_RCVD_IRQHandler       ; 0x38  0x00E0  56: BTLE RX Received 
    def_irq_handler    BTLE_RX_ENG_DET_IRQHandler    ; 0x39  0x00E4  57: BTLE RX Energy Detected 
    def_irq_handler    BTLE_SFD_DET_IRQHandler       ; 0x3A  0x00E8  58: BTLE SFD Detected  
    def_irq_handler    BTLE_SFD_TO_IRQHandler        ; 0x3B  0x00EC  59: BTLE SFD Timeout
    def_irq_handler    BTLE_GP_EVENT_IRQHandler      ; 0x3C  0x00F0  60: BTLE Timestampd
    def_irq_handler    BTLE_CFO_IRQHandler           ; 0x3D  0x00F4  61: BTLE CFO Done  
    def_irq_handler    BTLE_SIG_DET_IRQHandler       ; 0x3E  0x00F8  62: BTLE Signal Detected 
    def_irq_handler    BTLE_AGC_EVENT_IRQHandler     ; 0x3F  0x00FC  63: BTLE AGC Event 
    def_irq_handler    BTLE_RFFE_SPIM_IRQHandler     ; 0x40  0x0100  64: BTLE RFFE SPIM Done
    def_irq_handler    BTLE_TX_AES_IRQHandler        ; 0x41  0x0104  65: BTLE TX AES Done 
    def_irq_handler    BTLE_RX_AES_IRQHandler        ; 0x42  0x0108  66: BTLE RX AES Done 
    def_irq_handler    BTLE_INV_APB_ADDR_IRQHandler  ; 0x43  0x010C  67: BTLE Invalid APB Address
    def_irq_handler    BTLE_IQ_DATA_VALID_IRQHandler ; 0x44  0x0110  68: BTLE IQ Data Valid
    def_irq_handler    WUT_IRQHandler                ; 0x45  0x0114  69: Reserved 
    def_irq_handler    GPIOWAKE_IRQHandler           ; 0x46  0x0118  70: GPIO Wakeup
    def_irq_handler    RSV55_IRQHandler              ; 0x47  0x011C  71: Reserved 
    def_irq_handler    SPI0_IRQHandler               ; 0x48  0x0120  72: SPI0 
    def_irq_handler    WDT1_IRQHandler               ; 0x49  0x0124  73: Watchdog 1 
    def_irq_handler    RSV57_IRQHandler              ; 0x4A  0x0128  74: Reserved 
    def_irq_handler    PT_IRQHandler                 ; 0x4B  0x012C  75: Pulse Train
    def_irq_handler    RSV59_IRQHandler              ; 0x4C  0x0130  76: Reserved 
    def_irq_handler    RSV61_IRQHandler              ; 0x4D  0x0134  77: Reserved 
    def_irq_handler    I2C2_IRQHandler               ; 0x4E  0x0138  78: I2C 2 
    def_irq_handler    SDMA2_IRQHandler              ; 0x4F  0x013C  79: Smart DMA 2
    def_irq_handler    RSV64_IRQHandler              ; 0x50  0x0140  80: Reserved
    def_irq_handler    RSV65_IRQHandler              ; 0x51  0x0144  81: Reserved 
    def_irq_handler    RSV66_IRQHandler              ; 0x52  0x0148  82: Reserved 
    def_irq_handler    OWM_IRQHandler                ; 0x53  0x014C  83: One Wire Master
    def_irq_handler    RSV68_IRQHandler              ; 0x54  0x0150  84: Reserved 
    def_irq_handler    RSV69_IRQHandler              ; 0x55  0x0154  85: Reserved 
    def_irq_handler    RSV70_IRQHandler              ; 0x56  0x0158  86: Reserved 
    def_irq_handler    RSV71_IRQHandler              ; 0x57  0x015C  87: Reserved 
    def_irq_handler    RSV72_IRQHandler              ; 0x58  0x0160  88: Reserved 
    def_irq_handler    RSV73_IRQHandler              ; 0x59  0x0164  89: Reserved 
    def_irq_handler    RSV74_IRQHandler              ; 0x5A  0x0168  90: Reserved 
    def_irq_handler    RSV75_IRQHandler              ; 0x5B  0x016C  91: Reserved 
    def_irq_handler    RSV76_IRQHandler              ; 0x5C  0x0170  92: Reserved 
    def_irq_handler    RSV77_IRQHandler              ; 0x5D  0x0174  93: Reserved 
    def_irq_handler    RSV78_IRQHandler              ; 0x5E  0x0178  94: Reserved 
    def_irq_handler    RSV79_IRQHandler              ; 0x5F  0x017C  95: Reserved 
    def_irq_handler    RSV80_IRQHandler              ; 0x60  0x0180  96: Reserved 
    def_irq_handler    RSV81_IRQHandler              ; 0x61  0x0184  97: Reserved
    def_irq_handler    ECC_IRQHandler                ; 0x62  0x0188  98: ECC 
    def_irq_handler    DVS_IRQHandler                ; 0x63  0x018C  99: DVS 
    def_irq_handler    SIMO_IRQHandler               ; 0x64 0x0190   100: SIMO
    def_irq_handler    RSV85_IRQHandler              ; 0x65  0x0194  101: Reserved
    def_irq_handler    RSV86_IRQHandler              ; 0x66  0x0198  102: Reserved
    def_irq_handler    RSV87_IRQHandler              ; 0x67  0x019C  103: Reserved
    def_irq_handler    UART3_IRQHandler              ; 0x68  0x01A0  104: UART 3 
    def_irq_handler    RSV89_IRQHandler              ; 0x69  0x01A4  105: Reserved 
    def_irq_handler    RSV90_IRQHandler              ; 0x6A  0x01A8  106: Reserved
    def_irq_handler    RSV91_IRQHandler              ; 0x6B  0x01AC  107: Reserved 
    def_irq_handler    RSV92_IRQHandler              ; 0x6C  0x01B0  108: Reserved  
    def_irq_handler    RSV93_IRQHandler              ; 0x6D  0x01B1  109: Reserved 
    def_irq_handler    RSV94_IRQHandler              ; 0x6E  0x01B2  110: Reserved
    def_irq_handler    RSV95_IRQHandler              ; 0x6F  0x01B1  111: Reserved 
    def_irq_handler    RSV96_IRQHandler              ; 0x70  0x01B2  112: Reserved
    def_irq_handler    AES_IRQHandler                ; 0x71  0x01B1  113: AES  
    def_irq_handler    CRC_IRQHandler                ; 0x72  0x01B2  114: CRC
    def_irq_handler    I2S_IRQHandler                ; 0x73  0x01B2  115: I2S
    END
