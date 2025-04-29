;******************************************************************************
;
; Copyright (C) 202-2023 Maxim Integrated Products, Inc. (now owned by 
; Analog Devices, Inc.),
; Copyright (C) 2023-2025 Analog Devices, Inc.
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.
;
;******************************************************************************

Stack_Size      EQU     0x00001000

    AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp    ; ARMCC: name is set to work with MicroLib


Heap_Size       EQU     0x00001000

    AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

    PRESERVE8
    THUMB


; Vector Table Mapped to Address 0 at Reset

    AREA    RESET, DATA, READONLY
    EXPORT  __StackTop
    EXPORT  __isr_vector
    EXPORT  __isr_vector_End
    EXPORT  __isr_vector_Size
    IMPORT  SysTick_Handler

__isr_vector
    DCD     __initial_sp              ; Top of Stack
    DCD     Reset_Handler             ; Reset Handler
    DCD     NMI_Handler               ; NMI Handler
    DCD     HardFault_Handler         ; Hard Fault Handler
    DCD     MemManage_Handler         ; MPU Fault Handler
    DCD     BusFault_Handler          ; Bus Fault Handler
    DCD     UsageFault_Handler        ; Usage Fault Handler
    DCD     0                         ; Reserved
    DCD     0                         ; Reserved
    DCD     0                         ; Reserved
    DCD     0                         ; Reserved
    DCD     SVC_Handler               ; SVCall Handler
    DCD     DebugMon_Handler          ; Debug Monitor Handler
    DCD     0                         ; Reserved
    DCD     PendSV_Handler            ; PendSV Handler
    DCD     SysTick_Handler           ; SysTick Handler

    ; Device-specific Interrupts
    DCD     PF_IRQHandler             ; 0x10  0x0040  16: Power Fail 
    DCD     WDT0_IRQHandler           ; 0x11  0x0044  17: Watchdog 0 
    DCD     USB_IRQHandler            ; 0x12  0x0048  18: USB 
    DCD     RTC_IRQHandler            ; 0x13  0x004C  19: RTC 
    DCD     TRNG_IRQHandler           ; 0x14  0x0050  20: True Random Number Generator 
    DCD     TMR0_IRQHandler           ; 0x15  0x0054  21: Timer 0 
    DCD     TMR1_IRQHandler           ; 0x16  0x0058  22: Timer 1 
    DCD     TMR2_IRQHandler           ; 0x17  0x005C  23: Timer 2 
    DCD     TMR3_IRQHandler           ; 0x18  0x0060  24: Timer 3 
    DCD     TMR4_IRQHandler           ; 0x19  0x0064  25: Timer 4 (LP) 
    DCD     TMR5_IRQHandler           ; 0x1A  0x0068  26: Timer 5 (LP) 
    DCD     RSV11_IRQHandler          ; 0x1B  0x006C  27: Reserved 
    DCD     RSV12_IRQHandler          ; 0x1C  0x0070  28: Reserved 
    DCD     I2C0_IRQHandler           ; 0x1D  0x0074  29: I2C0 
    DCD     UART0_IRQHandler          ; 0x1E  0x0078  30: UART 0 
    DCD     UART1_IRQHandler          ; 0x1F  0x007C  31: UART 1 
    DCD     SPI1_IRQHandler           ; 0x20  0x0080  32: SPI1 
    DCD     RSV17_IRQHandler          ; 0x21  0x0084  33: Reserved 
    DCD     RSV18_IRQHandler          ; 0x22  0x0088  34: Reserved 
    DCD     RSV19_IRQHandler          ; 0x23  0x008C  35: Reserved 
    DCD     ADC_IRQHandler            ; 0x24  0x0090  36: ADC 
    DCD     RSV21_IRQHandler          ; 0x25  0x0094  37: Reserved 
    DCD     RSV22_IRQHandler          ; 0x26  0x0098  38: Reserved 
    DCD     FLC0_IRQHandler           ; 0x27  0x009C  39: Flash Controller 
    DCD     GPIO0_IRQHandler          ; 0x28  0x00A0  40: GPIO0 
    DCD     GPIO1_IRQHandler          ; 0x29  0x00A4  41: GPIO1 
    DCD     GPIO2_IRQHandler          ; 0x2A  0x00A8  42: GPIO2 (LP) 
    DCD     RSV27_IRQHandler          ; 0x2B  0x00AC  43: Reserved 
    DCD     DMA0_IRQHandler           ; 0x2C  0x00B0  44: DMA0 
    DCD     DMA1_IRQHandler           ; 0x2D  0x00B4  45: DMA1 
    DCD     DMA2_IRQHandler           ; 0x2E  0x00B8  46: DMA2 
    DCD     DMA3_IRQHandler           ; 0x2F  0x00BC  47: DMA3 
    DCD     RSV32_IRQHandler          ; 0x30  0x00C0  48: Reserved 
    DCD     RSV33_IRQHandler          ; 0x31  0x00C4  49: Reserved 
    DCD     UART2_IRQHandler          ; 0x32  0x00C8  50: UART 2 
    DCD     RSV35_IRQHandler          ; 0x33  0x00CC  51: Reserved 
    DCD     I2C1_IRQHandler           ; 0x34  0x00D0  52: I2C1 
    DCD     RSV37_IRQHandler          ; 0x35  0x00D4  53: Reserved 
    DCD     RSV38_IRQHandler          ; 0x36  0x00D8  54: Reserved 
    DCD     RSV39_IRQHandler          ; 0x37  0x00DC  55: Reserved 
    DCD     RSV40_IRQHandler          ; 0x38  0x00E0  56: Reserved 
    DCD     RSV41_IRQHandler          ; 0x39  0x00E4  57: Reserved 
    DCD     RSV42_IRQHandler          ; 0x3A  0x00E8  58: Reserved 
    DCD     RSV43_IRQHandler          ; 0x3B  0x00EC  59: Reserved 
    DCD     RSV44_IRQHandler          ; 0x3C  0x00F0  60: Reserved 
    DCD     RSV45_IRQHandler          ; 0x3D  0x00F4  61: Reserved 
    DCD     RSV46_IRQHandler          ; 0x3E  0x00F8  62: Reserved 
    DCD     RSV47_IRQHandler          ; 0x3F  0x00FC  63: Reserved 
    DCD     RSV48_IRQHandler          ; 0x40  0x0100  64: Reserved 
    DCD     RSV49_IRQHandler          ; 0x41  0x0104  65: Reserved 
    DCD     RSV50_IRQHandler          ; 0x42  0x0108  66: Reserved 
    DCD     RSV51_IRQHandler          ; 0x43  0x010C  67: Reserved 
    DCD     RSV52_IRQHandler          ; 0x44  0x0110  68: Reserved 
    DCD     WUT_IRQHandler            ; 0x45  0x0114  69: Wakeup Timer 
    DCD     GPIOWAKE_IRQHandler       ; 0x46  0x0118  70: GPIO and AIN Wakeup 
    DCD     RSV55_IRQHandler          ; 0x47  0x011C  71: Reserved 
    DCD     SPI0_IRQHandler           ; 0x48  0x0120  72: SPI0 
    DCD     WDT1_IRQHandler           ; 0x49  0x0124  73: LP Watchdog 
    DCD     RSV58_IRQHandler          ; 0x4A  0x0128  74: Reserved 
    DCD     PT_IRQHandler             ; 0x4B  0x012C  75: Pulse Train 
    DCD     RSV60_IRQHandler          ; 0x4C  0x0130  76: Reserved 
    DCD     RSV61_IRQHandler          ; 0x4D  0x0134  77: Reserved 
    DCD     I2C2_IRQHandler           ; 0x4E  0x0138  78: I2C2 
    DCD     RISCV_IRQHandler          ; 0x4F  0x013C  79: RISC-V 
    DCD     RSV64_IRQHandler          ; 0x50  0x0140  80: Reserved 
    DCD     RSV65_IRQHandler          ; 0x51  0x0144  81: Reserved 
    DCD     SDHC_IRQHandler           ; 0x52  0x0148  82: SDHC/SDIO 
    DCD     OWM_IRQHandler            ; 0x53  0x014C  83: One Wire Master 
    DCD     RSV68_IRQHandler          ; 0x54  0x0150  84: Reserved 
    DCD     RSV69_IRQHandler          ; 0x55  0x0154  85: Reserved 
    DCD     RSV70_IRQHandler          ; 0x56  0x0158  86: Reserved 
    DCD     RSV71_IRQHandler          ; 0x57  0x015C  87: Reserved 
    DCD     RSV72_IRQHandler          ; 0x58  0x0160  88: Reserved 
    DCD     RSV73_IRQHandler          ; 0x59  0x0164  89: Reserved 
    DCD     RSV74_IRQHandler          ; 0x5A  0x0168  90: Reserved 
    DCD     RSV75_IRQHandler          ; 0x5B  0x016C  91: Reserved 
    DCD     RSV76_IRQHandler          ; 0x5C  0x0170  92: Reserved 
    DCD     RSV77_IRQHandler          ; 0x5D  0x0174  93: Reserved 
    DCD     RSV78_IRQHandler          ; 0x5E  0x0178  94: Reserved 
    DCD     RSV79_IRQHandler          ; 0x5F  0x017C  95: Reserved 
    DCD     RSV80_IRQHandler          ; 0x60  0x0180  96: Reserved
    DCD     RSV81_IRQHandler          ; 0x61  0x0184  97: Reserved 
    DCD     ECC_IRQHandler            ; 0x62  0x0188  98: ECC 
    DCD     DVS_IRQHandler            ; 0x63  0x018C  99: DVS 
    DCD     SIMO_IRQHandler           ; 0x64  0x0190 100: SIMO 
    DCD     RSV85_IRQHandler          ; 0x65  0x0194 101: Reserved 
    DCD     RSV86_IRQHandler          ; 0x66  0x0198 102: Reserved 
    DCD     RSV87_IRQHandler          ; 0x67  0x019C 103: Reserved 
    DCD     UART3_IRQHandler          ; 0x68  0x01A0 104: UART 3 (LP) 
    DCD     RSV89_IRQHandler          ; 0x69  0x01A4 105: Reserved 
    DCD     RSV90_IRQHandler          ; 0x6A  0x01A8 106: Reserved 
    DCD     PCIF_IRQHandler           ; 0x6B  0x01AC 107: PCIF (Camera) 
    DCD     RSV92_IRQHandler          ; 0x6C  0x01B0 108: Reserved 
    DCD     RSV93_IRQHandler          ; 0x6D  0x01B4 109: Reserved 
    DCD     RSV94_IRQHandler          ; 0x6E  0x01B8 110: Reserved 
    DCD     RSV95_IRQHandler          ; 0x6F  0x01BC 111: Reserved 
    DCD     RSV96_IRQHandler          ; 0x70  0x01C0 112: Reserved 
    DCD     AES_IRQHandler            ; 0x71  0x01C4 113: AES 
    DCD     CRC_IRQHandler            ; 0x72  0x01C8 114: CRC 
    DCD     I2S_IRQHandler            ; 0x73  0x01CC 115: I2S 
    DCD     CNN_FIFO_IRQHandler       ; 0x74  0x01D0 116: CNN FIFO 
    DCD     CNN_IRQHandler            ; 0x75  0x01D4 117: CNN 
    DCD     RSV102_IRQHandler         ; 0x76  0x01D8 118: Reserved 
    DCD     LPCMP_IRQHandler          ; 0x77  0x01Dc 119: LP Comparator 
    DCD     CSI2_IRQHandler           ; 0x78  0x01E0 120: CSI2 
__isr_vector_End

__isr_vector_Size       EQU     __isr_vector_End - __isr_vector
__StackTop              EQU     __initial_sp

    AREA    |.text|, CODE, READONLY				

Reset_Handler   PROC
    EXPORT Reset_Handler                    [WEAK]
    IMPORT __main
    IMPORT SystemInit
    IMPORT PreInit
    LDR     R0, =PreInit
    BLX     R0
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0, =__main
    BX      R0

__SPIN          
    WFI
    BL __SPIN
    ENDP

; Dummy Exception Handlers (infinite loops which can be modified)
                            
NMI_Handler         PROC    
                    EXPORT NMI_Handler              [WEAK]
                    B       NMI_Handler
                    ENDP

HardFault_Handler   PROC    
                    EXPORT HardFault_Handler        [WEAK]
                    B       HardFault_Handler
                    ENDP

MemManage_Handler   PROC    
                    EXPORT MemManage_Handler        [WEAK]
                    B       MemManage_Handler
                    ENDP

BusFault_Handler    PROC    
                    EXPORT BusFault_Handler         [WEAK]
                    B       BusFault_Handler
                    ENDP

UsageFault_Handler  PROC    
                    EXPORT UsageFault_Handler       [WEAK]
                    B       UsageFault_Handler
                    ENDP

SVC_Handler         PROC    
                    EXPORT SVC_Handler              [WEAK]
                    B       SVC_Handler
                    ENDP

DebugMon_Handler    PROC    
                    EXPORT DebugMon_Handler         [WEAK]
                    B       DebugMon_Handler
                    ENDP

PendSV_Handler      PROC    
                    EXPORT PendSV_Handler           [WEAK]
                    B       PendSV_Handler
                    ENDP

; SysTick_Handler weakly defined in mxc_delay.c

Default_Handler PROC
    EXPORT  PF_IRQHandler       [WEAK]    ; 0x10  0x0040  16: Power Fail
    EXPORT  WDT0_IRQHandler     [WEAK]    ; 0x11  0x0044  17: Watchdog 0
    EXPORT  USB_IRQHandler      [WEAK]    ; 0x12  0x0048  18: USB
    EXPORT  RTC_IRQHandler      [WEAK]    ; 0x13  0x004C  19: RTC
    EXPORT  TRNG_IRQHandler     [WEAK]    ; 0x14  0x0050  20: True Random Number Generator
    EXPORT  TMR0_IRQHandler     [WEAK]    ; 0x15  0x0054  21: Timer 0
    EXPORT  TMR1_IRQHandler     [WEAK]    ; 0x16  0x0058  22: Timer 1
    EXPORT  TMR2_IRQHandler     [WEAK]    ; 0x17  0x005C  23: Timer 2
    EXPORT  TMR3_IRQHandler     [WEAK]    ; 0x18  0x0060  24: Timer 3
    EXPORT  TMR4_IRQHandler     [WEAK]    ; 0x19  0x0064  25: Timer 4
    EXPORT  TMR5_IRQHandler     [WEAK]    ; 0x1A  0x0068  26: Timer 5
    EXPORT  RSV11_IRQHandler    [WEAK]    ; 0x1B  0x006C  27: Reserved
    EXPORT  RSV12_IRQHandler    [WEAK]    ; 0x1C  0x0070  28: Reserved
    EXPORT  I2C0_IRQHandler     [WEAK]    ; 0x1D  0x0074  29: I2C0
    EXPORT  UART0_IRQHandler    [WEAK]    ; 0x1E  0x0078  30: UART 0
    EXPORT  UART1_IRQHandler    [WEAK]    ; 0x1F  0x007C  31: UART 1
    EXPORT  SPI1_IRQHandler     [WEAK]    ; 0x20  0x0080  32: SPI1
    EXPORT  RSV17_IRQHandler    [WEAK]    ; 0x21  0x0084  33: Reserved
    EXPORT  RSV18_IRQHandler    [WEAK]    ; 0x22  0x0088  34: Reserved
    EXPORT  RSV19_IRQHandler    [WEAK]    ; 0x23  0x008C  35: Reserved
    EXPORT  ADC_IRQHandler      [WEAK]    ; 0x24  0x0090  36: ADC
    EXPORT  RSV21_IRQHandler    [WEAK]    ; 0x25  0x0094  37: Reserved
    EXPORT  RSV22_IRQHandler    [WEAK]    ; 0x26  0x0098  38: Reserved
    EXPORT  FLC0_IRQHandler     [WEAK]    ; 0x27  0x009C  39: Flash Controller 0
    IMPORT  GPIO0_IRQHandler              ; 0x28  0x00A0  40: GPIO0                 DEFINED IN board.c
    IMPORT  GPIO1_IRQHandler              ; 0x29  0x00A4  41: GPIO1                 DEFINED IN board.c
    IMPORT  GPIO2_IRQHandler              ; 0x2A  0x00A8  42: GPIO2 (LP)            DEFINED IN board.c
    EXPORT  RSV27_IRQHandler    [WEAK]    ; 0x2B  0x00AC  43: Reserved
    EXPORT  DMA0_IRQHandler     [WEAK]    ; 0x2C  0x00B0  44: DMA0
    EXPORT  DMA1_IRQHandler     [WEAK]    ; 0x2D  0x00B4  45: DMA1
    EXPORT  DMA2_IRQHandler     [WEAK]    ; 0x2E  0x00B8  46: DMA2
    EXPORT  DMA3_IRQHandler     [WEAK]    ; 0x2F  0x00BC  47: DMA3
    EXPORT  RSV32_IRQHandler    [WEAK]    ; 0x30  0x00C0  48: Reserved
    EXPORT  RSV33_IRQHandler    [WEAK]    ; 0x31  0x00C4  49: Reserved
    EXPORT  UART2_IRQHandler    [WEAK]    ; 0x32  0x00C8  50: UART 2
    EXPORT  RSV35_IRQHandler    [WEAK]    ; 0x33  0x00CC  51: Reserved
    EXPORT  I2C1_IRQHandler     [WEAK]    ; 0x34  0x00D0  52: I2C1
    EXPORT  RSV37_IRQHandler    [WEAK]    ; 0x35  0x00D4  53: Reserved
    EXPORT  RSV38_IRQHandler    [WEAK]    ; 0x36  0x00D8  54: Reserved
    EXPORT  RSV39_IRQHandler    [WEAK]    ; 0x37  0x00DC  55: Reserved
    EXPORT  RSV40_IRQHandler    [WEAK]    ; 0x38  0x00E0  56: Reserved
    EXPORT  RSV41_IRQHandler    [WEAK]    ; 0x39  0x00E4  57: Reserved
    EXPORT  RSV42_IRQHandler    [WEAK]    ; 0x3A  0x00E8  58: Reserved
    EXPORT  RSV43_IRQHandler    [WEAK]    ; 0x3B  0x00EC  59: Reserved
    EXPORT  RSV44_IRQHandler    [WEAK]    ; 0x3C  0x00F0  60: Reserved
    EXPORT  RSV45_IRQHandler    [WEAK]    ; 0x3D  0x00F4  61: Reserved
    EXPORT  RSV46_IRQHandler    [WEAK]    ; 0x3E  0x00F8  62: Reserved
    EXPORT  RSV47_IRQHandler    [WEAK]    ; 0x3F  0x00FC  63: Reserved
    EXPORT  RSV48_IRQHandler    [WEAK]    ; 0x40  0x0100  64: Reserved
    EXPORT  RSV49_IRQHandler    [WEAK]    ; 0x41  0x0104  65: Reserved
    EXPORT  RSV50_IRQHandler    [WEAK]    ; 0x42  0x0108  66: Reserved
    EXPORT  RSV51_IRQHandler    [WEAK]    ; 0x43  0x010C  67: Reserved
    EXPORT  RSV52_IRQHandler    [WEAK]    ; 0x44  0x0110  68: Reserved
    EXPORT  WUT_IRQHandler      [WEAK]    ; 0x45  0x0114  69: Wake-up Timer
    EXPORT  GPIOWAKE_IRQHandler [WEAK]    ; 0x46  0x0118  70: GPIO and Analog In Wakeup
    EXPORT  RSV55_IRQHandler    [WEAK]    ; 0x47  0x011C  71: Reserved
    EXPORT  SPI0_IRQHandler     [WEAK]    ; 0x48  0x0120  72: SPI0
    EXPORT  WDT1_IRQHandler     [WEAK]    ; 0x49  0x0124  73: Watchdog 1
    EXPORT  RSV58_IRQHandler    [WEAK]    ; 0x4A  0x0128  74: Reserved
    EXPORT  PT_IRQHandler       [WEAK]    ; 0x4B  0x012C  75: Pulse train
    EXPORT  RSV60_IRQHandler    [WEAK]    ; 0x4C  0x0130  76: Reserved
    EXPORT  RSV61_IRQHandler    [WEAK]    ; 0x4D  0x0134  77: Reserved
    EXPORT  I2C2_IRQHandler     [WEAK]    ; 0x4E  0x0138  78: I2C 1
    EXPORT  RISCV_IRQHandler    [WEAK]    ; 0x4F  0x013C  79: RISC-V Core Interrupt
    EXPORT  RSV64_IRQHandler    [WEAK]    ; 0x50  0x0140  80: Reserved
    EXPORT  RSV65_IRQHandler    [WEAK]    ; 0x51  0x0144  81: Reserved
    EXPORT  SDHC_IRQHandler     [WEAK]    ; 0x52  0x0148  82: SDHC
    EXPORT  OWM_IRQHandler      [WEAK]    ; 0x53  0x014C  83: One Wire Master
    EXPORT  RSV68_IRQHandler    [WEAK]    ; 0x54  0x0150  84: Reserved
    EXPORT  RSV69_IRQHandler    [WEAK]    ; 0x55  0x0154  85: Reserved
    EXPORT  RSV70_IRQHandler    [WEAK]    ; 0x56  0x0158  86: Reserved
    EXPORT  RSV71_IRQHandler    [WEAK]    ; 0x57  0x015C  87: Reserved
    EXPORT  RSV72_IRQHandler    [WEAK]    ; 0x58  0x0160  88: Reserved
    EXPORT  RSV73_IRQHandler    [WEAK]    ; 0x59  0x0164  89: Reserved
    EXPORT  RSV74_IRQHandler    [WEAK]    ; 0x5A  0x0168  90: Reserved
    EXPORT  RSV75_IRQHandler    [WEAK]    ; 0x5B  0x016C  91: Reserved
    EXPORT  RSV76_IRQHandler    [WEAK]    ; 0x5C  0x0170  92: Reserved
    EXPORT  RSV77_IRQHandler    [WEAK]    ; 0x5D  0x0174  93: Reserved
    EXPORT  RSV78_IRQHandler    [WEAK]    ; 0x5E  0x0178  94: Reserved
    EXPORT  RSV79_IRQHandler    [WEAK]    ; 0x5F  0x017C  95: Reserved
    EXPORT  RSV80_IRQHandler    [WEAK]    ; 0x60  0x0180  96: Reserved
    EXPORT  RSV81_IRQHandler    [WEAK]    ; 0x61  0x0184  97: Reserved
    EXPORT  ECC_IRQHandler      [WEAK]    ; 0x62  0x0188  98: Error Correction
    EXPORT  DVS_IRQHandler      [WEAK]    ; 0x63  0x018C  99: DVS Controller
    EXPORT  SIMO_IRQHandler     [WEAK]    ; 0x64  0x0190  100: SIMO Controller
    EXPORT  RSV85_IRQHandler    [WEAK]    ; 0x65  0x0194  101: Reserved
    EXPORT  RSV86_IRQHandler    [WEAK]    ; 0x66  0x0198  102: Reserved
    EXPORT  RSV87_IRQHandler    [WEAK]    ; 0x67  0x019C  103: Reserved
    EXPORT  UART3_IRQHandler    [WEAK]    ; 0x68  0x01A0  104: UART 3 (LP)
    EXPORT  RSV89_IRQHandler    [WEAK]    ; 0x69  0x01A4  105: UART 4
    EXPORT  RSV90_IRQHandler    [WEAK]    ; 0x6A  0x01A8  106: UART 5
    EXPORT  PCIF_IRQHandler     [WEAK]    ; 0x6B  0x01AC  107: Camera IF
    EXPORT  RSV92_IRQHandler    [WEAK]    ; 0x6C  0x01B0  108: Reserved
    EXPORT  RSV93_IRQHandler    [WEAK]    ; 0x6D  0x01B4  109: Reserved
    EXPORT  RSV94_IRQHandler    [WEAK]    ; 0x6E  0x01B8  110: Reserved
    EXPORT  RSV95_IRQHandler    [WEAK]    ; 0x6F  0x01BC  111: Reserved
    EXPORT  RSV96_IRQHandler    [WEAK]    ; 0x70  0x01C0  112: Reserved
    EXPORT  AES_IRQHandler      [WEAK]    ; 0x71  0x01C4  113: AES
    EXPORT  CRC_IRQHandler      [WEAK]    ; 0x72  0x01C8  114: CRC
    EXPORT  I2S_IRQHandler      [WEAK]    ; 0x73  0x01CC  115: I2S
    EXPORT  CNN_FIFO_IRQHandler [WEAK]    ; 0x74  0x01D0  116: CNN FIFO
    EXPORT  CNN_IRQHandler      [WEAK]    ; 0x75  0x01D4  117: CNN
    EXPORT  RSV102_IRQHandler   [WEAK]    ; 0x76  0x01D8  118: Reserved
    EXPORT  LPCMP_IRQHandler    [WEAK]    ; 0x77  0x01DC  119: LP Comparator
    EXPORT  CSI2_IRQHandler     [WEAK]    ; 0x78  0x01DE  120: CSI2 APB

;*******************************************************************************
; Default handler implementations
;*******************************************************************************
PF_IRQHandler
WDT0_IRQHandler
RSV02_IRQHandler
RTC_IRQHandler
TRNG_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
TMR5_IRQHandler
RSV11_IRQHandler
RSV12_IRQHandler
I2C0_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
SPI1_IRQHandler
RSV17_IRQHandler
RSV18_IRQHandler
RSV19_IRQHandler
ADC_IRQHandler
RSV21_IRQHandler
RSV22_IRQHandler
FLC0_IRQHandler
; GPIO0_IRQHandler - Defined in board.c
; GPIO1_IRQHandler - Defined in board.c
; GPIO2_IRQHandler - Defined in board.c
RSV27_IRQHandler
DMA0_IRQHandler
DMA1_IRQHandler
DMA2_IRQHandler
DMA3_IRQHandler
RSV32_IRQHandler
RSV33_IRQHandler
UART2_IRQHandler
RSV35_IRQHandler
I2C1_IRQHandler
RSV37_IRQHandler
RSV38_IRQHandler
RSV39_IRQHandler
RSV40_IRQHandler
RSV41_IRQHandler
RSV42_IRQHandler
RSV43_IRQHandler
RSV44_IRQHandler
RSV45_IRQHandler
RSV46_IRQHandler
RSV47_IRQHandler
RSV48_IRQHandler
RSV49_IRQHandler
RSV50_IRQHandler
RSV51_IRQHandler
RSV52_IRQHandler
WUT_IRQHandler
GPIOWAKE_IRQHandler
RSV55_IRQHandler
SPI0_IRQHandler
WDT1_IRQHandler
RSV58_IRQHandler
PT_IRQHandler
RSV60_IRQHandler
RSV61_IRQHandler
I2C2_IRQHandler
RISCV_IRQHandler
RSV64_IRQHandler
RSV65_IRQHandler
RSV66_IRQHandler
OWM_IRQHandler
RSV68_IRQHandler
RSV69_IRQHandler
RSV70_IRQHandler
RSV71_IRQHandler
RSV72_IRQHandler
RSV73_IRQHandler
RSV74_IRQHandler
RSV75_IRQHandler
RSV76_IRQHandler
RSV77_IRQHandler
RSV78_IRQHandler
RSV79_IRQHandler
RSV80_IRQHandler
RSV81_IRQHandler
ECC_IRQHandler
DVS_IRQHandler
SIMO_IRQHandler
RSV85_IRQHandler
RSV86_IRQHandler
RSV87_IRQHandler
UART3_IRQHandler
RSV89_IRQHandler
RSV90_IRQHandler
PCIF_IRQHandler
RSV92_IRQHandler
RSV93_IRQHandler
RSV94_IRQHandler
RSV95_IRQHandler
RSV96_IRQHandler
AES_IRQHandler
CRC_IRQHandler
I2S_IRQHandler
CNN_FIFO_IRQHandler
CNN_IRQHandler
RSV102_IRQHandler
LPCMP_IRQHandler
CSI2_IRQHandler
    B .
    ; Adds 2-bytes padding to align user stack and heap initialization below.
    ALIGN
    ENDP


;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
    IF      :DEF:__MICROLIB

        ; Both are the same, but __initial_sp is required for microlib.
        EXPORT  __initial_sp 
        EXPORT  __StackTop

        EXPORT  __heap_base
        EXPORT  __heap_limit

    ELSE

        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC

        LDR     R0, =Heap_Mem
        LDR     R1, =(Stack_Mem + Stack_Size)
        LDR     R2, =(Heap_Mem + Heap_Size)
        LDR     R3, =Stack_Mem
        BX      LR
        ENDP

    ENDIF

    END

