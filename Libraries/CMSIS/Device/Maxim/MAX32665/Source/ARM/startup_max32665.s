;*****************************************************************************
;
; Copyright (C) 2025 Analog Devices, Inc.
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
;*****************************************************************************

; To map FreeRTOS function names to their CMSIS equivalents add following lines to FreeRTOSConfig.h
; #define vPortSVCHandler SVC_Handler
; #define xPortPendSVHandler PendSV_Handler
; #define xPortSysTickHandler SysTick_Handler
; *------- <<< Use Configuration Wizard in Context Menu to Modify Stack Size and Heap Size. >>> ----

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp                                        ; ARMCC: name is set to work with MicroLib

; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000800

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
                EXPORT  __isr_vector
                IMPORT  SysTick_Handler
                ; IMPORT  WUT_IRQHandler
                ; Core Level - CM4
                                                ; Most names are to help the FreeRTOS port.

__isr_vector    DCD     __initial_sp            ; Top of Stack
                DCD     Reset_Handler           ; Reset Handler
                DCD     NMI_Handler             ; NMI Handler
                DCD     HardFault_Handler       ; Hard Fault Handler
                DCD     MemManage_Handler       ; MPU Fault Handler
                DCD     BusFault_Handler        ; Bus Fault Handler
                DCD     UsageFault_Handler      ; Usage Fault Handler
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     0                       ; Reserved
                DCD     SVC_Handler             ; SVCall Handler
                DCD     DebugMon_Handler        ; Debug Monitor Handler
                DCD     0                       ; Reserved				
                DCD     PendSV_Handler          ; PendSV Handler
                DCD     SysTick_Handler         ; SysTick Handler

                ; MAX32665 Device-specific Interrupts
                DCD     PF_IRQHandler                 ; 0x10  0x0040  16: Power Fail
                DCD     WDT0_IRQHandler               ; 0x11  0x0044  17: Watchdog 0
                DCD     USB_IRQHandler                ; 0x12  0x0048  18: USB
                DCD     RTC_IRQHandler                ; 0x13  0x004C  19: RTC
                DCD     TRNG_IRQHandler               ; 0x14  0x0050  20: True Random Number Gener
                DCD     TMR0_IRQHandler               ; 0x15  0x0054  21: Timer 0
                DCD     TMR1_IRQHandler               ; 0x16  0x0058  22: Timer 1
                DCD     TMR2_IRQHandler               ; 0x17  0x005C  23: Timer 2
                DCD     TMR3_IRQHandler               ; 0x18  0x0060  24: Timer 3
                DCD     TMR4_IRQHandler               ; 0x19  0x0064  25: Timer 4
                DCD     TMR5_IRQHandler               ; 0x1A  0x0068  26: Timer 5
                DCD     RSV11_IRQHandler              ; 0x1B  0x006C  27: Reserved
                DCD     RSV12_IRQHandler              ; 0x1C  0x0070  28: Reserved
                DCD     I2C0_IRQHandler               ; 0x1D  0x0074  29: I2C0
                DCD     UART0_IRQHandler              ; 0x1E  0x0078  30: UART 0
                DCD     UART1_IRQHandler              ; 0x1F  0x007C  31: UART 1
                DCD     SPI1_IRQHandler               ; 0x20  0x0080  32: SPI1
                DCD     SPI2_IRQHandler               ; 0x21  0x0084  33: SPI2
                DCD     RSV18_IRQHandler              ; 0x22  0x0088  34: Reserved
                DCD     RSV19_IRQHandler              ; 0x23  0x008C  35: Reserved
                DCD     ADC_IRQHandler                ; 0x24  0x0090  36: ADC
                DCD     RSV21_IRQHandler              ; 0x25  0x0094  37: Reserved
                DCD     RSV22_IRQHandler              ; 0x26  0x0098  38: Reserved
                DCD     FLC0_IRQHandler               ; 0x27  0x009C  39: Flash Controller 0
                DCD     GPIO0_IRQHandler              ; 0x28  0x00A0  40: GPIO0
                DCD     GPIO1_IRQHandler              ; 0x29  0x00A4  41: GPIO1
                DCD     RSV26_IRQHandler              ; 0x2A  0x00A8  42: Reserved
                DCD     TPU_IRQHandler                ; 0x2B  0x00AC  43: TPU
                DCD     DMA0_IRQHandler               ; 0x2C  0x00B0  44: DMA0
                DCD     DMA1_IRQHandler               ; 0x2D  0x00B4  45: DMA1
                DCD     DMA2_IRQHandler               ; 0x2E  0x00B8  46: DMA2
                DCD     DMA3_IRQHandler               ; 0x2F  0x00BC  47: DMA3
                DCD     RSV32_IRQHandler              ; 0x30  0x00C0  48: Reserved
                DCD     RSV33_IRQHandler              ; 0x31  0x00C4  49: Reserved
                DCD     UART2_IRQHandler              ; 0x32  0x00C8  50: UART 2
                DCD     RSV35_IRQHandler              ; 0x33  0x00CC  51: Reserved
                DCD     I2C1_IRQHandler               ; 0x34  0x00D0  52: I2C1
                DCD     RSV37_IRQHandler              ; 0x35  0x00D4  53: Reserved
                DCD     SPIXFC_IRQHandler              ; 0x36  0x00D8  54: SPI execute in place
                DCD     BTLE_TX_DONE_IRQHandler       ; 0x37  0x00DC  55: BTLE TX Done
                DCD     BTLE_RX_RCVD_IRQHandler       ; 0x38  0x00E0  56: BTLE RX Recived
                DCD     BTLE_RX_ENG_DET_IRQHandler    ; 0x39  0x00E4  57: BTLE RX Energy Dectected 
                DCD     BTLE_SFD_DET_IRQHandler       ; 0x3A  0x00E8  58: BTLE SFD Detected
                DCD     BTLE_SFD_TO_IRQHandler        ; 0x3B  0x00EC  59: BTLE SFD Timeout
                DCD     BTLE_GP_EVENT_IRQHandler      ; 0x3C  0x00F0  60: BTLE Timestamp
                DCD     BTLE_CFO_IRQHandler           ; 0x3D  0x00F4  61: BTLE CFO Done
                DCD     BTLE_SIG_DET_IRQHandler       ; 0x3E  0x00F8  62: BTLE Signal Detected
                DCD     BTLE_AGC_EVENT_IRQHandler     ; 0x3F  0x00FC  63: BTLE AGC Event
                DCD     BTLE_RFFE_SPIM_IRQHandler     ; 0x40  0x0100  64: BTLE RFFE SPIM Done
                DCD     BTLE_TX_AES_IRQHandler        ; 0x41  0x0104  65: BTLE TX AES Done
                DCD     BTLE_RX_AES_IRQHandler        ; 0x42  0x0108  66: BTLE RX AES Done
                DCD     BTLE_INV_APB_ADDR_IRQHandler  ; 0x43  0x010C  67: BTLE Invalid APB Address
                DCD     BTLE_IQ_DATA_VALID_IRQHandler ; 0x44  0x0110  68: BTLE IQ Data Valid
                DCD     WUT_IRQHandler                ; 0x45  0x0114  69: WUT Wakeup
                DCD     GPIOWAKE_IRQHandler           ; 0x46  0x0118  70: GPIO Wakeup
                DCD     RSV55_IRQHandler              ; 0x47  0x011C  71: Reserved
                DCD     SPI0_IRQHandler               ; 0x48  0x0120  72: SPI AHB
                DCD     WDT1_IRQHandler               ; 0x49  0x0124  73: Watchdog 1
                DCD     RSV58_IRQHandler              ; 0x4A  0x0128  74: Reserved
                DCD     PT_IRQHandler                 ; 0x4B  0x012C  75: Pulse train
                DCD     SDMA0_IRQHandler               ; 0x4C  0x0130  76: Smart DMA 0s
                DCD     RSV61_IRQHandler              ; 0x4D  0x0134  77: Reserved
                DCD     I2C2_IRQHandler               ; 0x4E  0x0138  78: I2C 1
                DCD     RSV63_IRQHandler              ; 0x4F  0x013C  79: Reserved
                DCD     RSV64_IRQHandler              ; 0x50  0x0140  80: Reserved
                DCD     RSV65_IRQHandler              ; 0x51  0x0144  81: Reserved
                DCD     SDHC_IRQHandler               ; 0x52  0x0148  82: SDIO/SDHC
                DCD     OWM_IRQHandler                ; 0x53  0x014C  83: One Wire Master
                DCD     DMA4_IRQHandler               ; 0x54  0x0150  84: DMA4
                DCD     DMA5_IRQHandler               ; 0x55  0x0154  85: DMA5
                DCD     DMA6_IRQHandler               ; 0x56  0x0158  86: DMA6
                DCD     DMA7_IRQHandler               ; 0x57  0x015C  87: DMA7
                DCD     DMA8_IRQHandler               ; 0x58  0x0160  88: DMA8
                DCD     DMA9_IRQHandler               ; 0x59  0x0164  89: DMA9
                DCD     DMA10_IRQHandler              ; 0x5A  0x0168  90: DMA10
                DCD     DMA11_IRQHandler              ; 0x5B  0x016C  91: DMA11
                DCD     DMA12_IRQHandler              ; 0x5C  0x0170  92: DMA12
                DCD     DMA13_IRQHandler              ; 0x5D  0x0174  93: DMA13
                DCD     DMA14_IRQHandler              ; 0x5E  0x0178  94: DMA14
                DCD     DMA15_IRQHandler              ; 0x5F  0x017C  95: DMA15
                DCD     USBDMA_IRQHandler             ; 0x60  0x0180  96: USBDMA
                DCD     WDT2_IRQHandler               ; 0x61  0x0184  97: Watchdog Timer 2 
                DCD     ECC_IRQHandler                ; 0x62  0x0188  98: Error Correction 
                DCD     DVS_IRQHandler                ; 0x63  0x018C  99: DVS Controller 
                DCD     SIMO_IRQHandler               ; 0x64  0x0190  100: SIMO Controller 
                DCD     SCA_IRQHandler                ; 0x65  0x0194  101: SCA 
                DCD     AUDIO_IRQHandler              ; 0x66  0x0198  102: Audio subsystem 
                DCD     FLC1_IRQHandler               ; 0x67  0x019C  103: Flash Control 1 
                DCD     RSV88_IRQHandler              ; 0x68  0x01A0  104: UART 3 
                DCD     RSV89_IRQHandler              ; 0x69  0x01A4  105: UART 4 
                DCD     RSV90_IRQHandler              ; 0x6A  0x01A8  106: UART 5 
                DCD     RSV91_IRQHandler              ; 0x6B  0x01AC  107: Camera IF 
                DCD     RSV92_IRQHandler              ; 0x6C  0x01B0  108: I3C 
                DCD     HTMR0_IRQHandler              ; 0x6D  0x01B4  109: HTimer0 
                DCD     HTMR1_IRQHandler              ; 0x6E  0x01B8  110: HTimer1 
__isr_vector_end

__isr_vector_Size       EQU     __isr_vector_end - __isr_vector
__Vectors       EQU     __isr_vector
__Vectors_End   EQU     __isr_vector_end
__Vectors_Size  EQU     __isr_vector_Size

                AREA    |.text|, CODE, READONLY

Reset_Handler   PROC
                EXPORT  Reset_Handler           [WEAK]
                IMPORT  PreInit
                IMPORT  __main

                LDR     R0, =PreInit
                BLX     R0
                ;
                ; SystemInit is called as part of $Sub$$main(void) after RAM initialization, see sub_main.c
                ;
                LDR     R0, =__main
                BX      R0
__SPIN
                WFI
                BL __SPIN
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler             [WEAK]
                BL      NMI_Handler
                ENDP

HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler       [WEAK]
                B       .
                ENDP

MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler       [WEAK]
                B       .
                ENDP

BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler        [WEAK]
                B       .
                ENDP

UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler      [WEAK]
                B       .
                ENDP

SVC_Handler\
                PROC
                EXPORT  SVC_Handler             [WEAK]
                B       .
                ENDP

DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler        [WEAK]
                B       .
                ENDP

PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler          [WEAK]
                B       .
                ENDP

Default_Handler\
                PROC
                ; MAX32665 Interrupts
                EXPORT  PF_IRQHandler                 [WEAK]  ; 0x10  0x0040  16: Power Fail
                EXPORT  WDT0_IRQHandler               [WEAK]  ; 0x11  0x0044  17: Watchdog 0
                EXPORT  USB_IRQHandler                [WEAK]  ; 0x12  0x0048  18: USB
                EXPORT  RTC_IRQHandler                [WEAK]  ; 0x13  0x004C  19: RTC
                EXPORT  TRNG_IRQHandler               [WEAK]  ; 0x14  0x0050  20: True Random Number Gener
                EXPORT  TMR0_IRQHandler               [WEAK]  ; 0x15  0x0054  21: Timer 0
                EXPORT  TMR1_IRQHandler               [WEAK]  ; 0x16  0x0058  22: Timer 1
                EXPORT  TMR2_IRQHandler               [WEAK]  ; 0x17  0x005C  23: Timer 2
                EXPORT  TMR3_IRQHandler               [WEAK]  ; 0x18  0x0060  24: Timer 3
                EXPORT  TMR4_IRQHandler               [WEAK]  ; 0x19  0x0064  25: Timer 4
                EXPORT  TMR5_IRQHandler               [WEAK]  ; 0x1A  0x0068  26: Timer 5
                EXPORT  RSV11_IRQHandler              [WEAK]  ; 0x1B  0x006C  27: Reserved
                EXPORT  RSV12_IRQHandler              [WEAK]  ; 0x1C  0x0070  28: Reserved
                EXPORT  I2C0_IRQHandler               [WEAK]  ; 0x1D  0x0074  29: I2C0
                EXPORT  UART0_IRQHandler              [WEAK]  ; 0x1E  0x0078  30: UART 0
                EXPORT  UART1_IRQHandler              [WEAK]  ; 0x1F  0x007C  31: UART 1
                EXPORT  SPI1_IRQHandler               [WEAK]  ; 0x20  0x0080  32: SPI1
                EXPORT  SPI2_IRQHandler               [WEAK]  ; 0x21  0x0084  33: SPI2
                EXPORT  RSV18_IRQHandler              [WEAK]  ; 0x22  0x0088  34: Reserved
                EXPORT  RSV19_IRQHandler              [WEAK]  ; 0x23  0x008C  35: Reserved
                EXPORT  ADC_IRQHandler                [WEAK]  ; 0x24  0x0090  36: ADC
                EXPORT  RSV21_IRQHandler              [WEAK]  ; 0x25  0x0094  37: Reserved
                EXPORT  RSV22_IRQHandler              [WEAK]  ; 0x26  0x0098  38: Reserved
                EXPORT  FLC0_IRQHandler               [WEAK]  ; 0x27  0x009C  39: Flash Controller 0
                EXPORT  GPIO0_IRQHandler              [WEAK]  ; 0x28  0x00A0  40: GPIO0
                EXPORT  GPIO1_IRQHandler              [WEAK]  ; 0x29  0x00A4  41: GPIO1
                EXPORT  RSV26_IRQHandler              [WEAK]  ; 0x2A  0x00A8  42: Reserved
                EXPORT  TPU_IRQHandler                [WEAK]  ; 0x2B  0x00AC  43: TPU
                EXPORT  DMA0_IRQHandler               [WEAK]  ; 0x2C  0x00B0  44: DMA0
                EXPORT  DMA1_IRQHandler               [WEAK]  ; 0x2D  0x00B4  45: DMA1
                EXPORT  DMA2_IRQHandler               [WEAK]  ; 0x2E  0x00B8  46: DMA2
                EXPORT  DMA3_IRQHandler               [WEAK]  ; 0x2F  0x00BC  47: DMA3
                EXPORT  RSV32_IRQHandler              [WEAK]  ; 0x30  0x00C0  48: Reserved
                EXPORT  RSV33_IRQHandler              [WEAK]  ; 0x31  0x00C4  49: Reserved
                EXPORT  UART2_IRQHandler              [WEAK]  ; 0x32  0x00C8  50: UART 2
                EXPORT  RSV35_IRQHandler              [WEAK]  ; 0x33  0x00CC  51: Reserved
                EXPORT  I2C1_IRQHandler               [WEAK]  ; 0x34  0x00D0  52: I2C1
                EXPORT  RSV37_IRQHandler              [WEAK]  ; 0x35  0x00D4  53: Reserved
                EXPORT  SPIXFC_IRQHandler              [WEAK]  ; 0x36  0x00D8  54: SPI execute in place
                EXPORT  BTLE_TX_DONE_IRQHandler       [WEAK]  ; 0x37  0x00DC  55: BTLE TX Done
                EXPORT  BTLE_RX_RCVD_IRQHandler       [WEAK]  ; 0x38  0x00E0  56: BTLE RX Recived
                EXPORT  BTLE_RX_ENG_DET_IRQHandler    [WEAK]  ; 0x39  0x00E4  57: BTLE RX Energy Dectected 
                EXPORT  BTLE_SFD_DET_IRQHandler       [WEAK]  ; 0x3A  0x00E8  58: BTLE SFD Detected
                EXPORT  BTLE_SFD_TO_IRQHandler        [WEAK]  ; 0x3B  0x00EC  59: BTLE SFD Timeout
                EXPORT  BTLE_GP_EVENT_IRQHandler      [WEAK]  ; 0x3C  0x00F0  60: BTLE Timestamp
                EXPORT  BTLE_CFO_IRQHandler           [WEAK]  ; 0x3D  0x00F4  61: BTLE CFO Done
                EXPORT  BTLE_SIG_DET_IRQHandler       [WEAK]  ; 0x3E  0x00F8  62: BTLE Signal Detected
                EXPORT  BTLE_AGC_EVENT_IRQHandler     [WEAK]  ; 0x3F  0x00FC  63: BTLE AGC Event
                EXPORT  BTLE_RFFE_SPIM_IRQHandler     [WEAK]  ; 0x40  0x0100  64: BTLE RFFE SPIM Done
                EXPORT  BTLE_TX_AES_IRQHandler        [WEAK]  ; 0x41  0x0104  65: BTLE TX AES Done
                EXPORT  BTLE_RX_AES_IRQHandler        [WEAK]  ; 0x42  0x0108  66: BTLE RX AES Done
                EXPORT  BTLE_INV_APB_ADDR_IRQHandler  [WEAK]  ; 0x43  0x010C  67: BTLE Invalid APB Address
                EXPORT  BTLE_IQ_DATA_VALID_IRQHandler [WEAK]  ; 0x44  0x0110  68: BTLE IQ Data Valid
                EXPORT  WUT_IRQHandler                [WEAK]  ; 0x45  0x0114  69: WUT Wakeup
                EXPORT  GPIOWAKE_IRQHandler           [WEAK]  ; 0x46  0x0118  70: GPIO Wakeup
                EXPORT  RSV55_IRQHandler              [WEAK]  ; 0x47  0x011C  71: Reserved
                EXPORT  SPI0_IRQHandler               [WEAK]  ; 0x48  0x0120  72: SPI0 AHB
                EXPORT  WDT1_IRQHandler               [WEAK]  ; 0x49  0x0124  73: Watchdog 1
                EXPORT  RSV58_IRQHandler              [WEAK]  ; 0x4A  0x0128  74: Reserved
                EXPORT  PT_IRQHandler                 [WEAK]  ; 0x4B  0x012C  75: Pulse train
                EXPORT  SDMA0_IRQHandler               [WEAK]  ; 0x4C  0x0130  76: Smart DMA 0s
                EXPORT  RSV61_IRQHandler              [WEAK]  ; 0x4D  0x0134  77: Reserved
                EXPORT  I2C2_IRQHandler               [WEAK]  ; 0x4E  0x0138  78: I2C 1
                EXPORT  RSV63_IRQHandler              [WEAK]  ; 0x4F  0x013C  79: Reserved
                EXPORT  RSV64_IRQHandler              [WEAK]  ; 0x50  0x0140  80: Reserved
                EXPORT  RSV65_IRQHandler              [WEAK]  ; 0x51  0x0144  81: Reserved
                EXPORT  SDHC_IRQHandler               [WEAK]  ; 0x52  0x0148  82: SDIO/SDHC
                EXPORT  OWM_IRQHandler                [WEAK]  ; 0x53  0x014C  83: One Wire Master
                EXPORT  DMA4_IRQHandler               [WEAK]  ; 0x54  0x0150  84: DMA4
                EXPORT  DMA5_IRQHandler               [WEAK]  ; 0x55  0x0154  85: DMA5
                EXPORT  DMA6_IRQHandler               [WEAK]  ; 0x56  0x0158  86: DMA6
                EXPORT  DMA7_IRQHandler               [WEAK]  ; 0x57  0x015C  87: DMA7
                EXPORT  DMA8_IRQHandler               [WEAK]  ; 0x58  0x0160  88: DMA8
                EXPORT  DMA9_IRQHandler               [WEAK]  ; 0x59  0x0164  89: DMA9
                EXPORT  DMA10_IRQHandler              [WEAK]  ; 0x5A  0x0168  90: DMA10
                EXPORT  DMA11_IRQHandler              [WEAK]  ; 0x5B  0x016C  91: DMA11
                EXPORT  DMA12_IRQHandler              [WEAK]  ; 0x5C  0x0170  92: DMA12
                EXPORT  DMA13_IRQHandler              [WEAK]  ; 0x5D  0x0174  93: DMA13
                EXPORT  DMA14_IRQHandler              [WEAK]  ; 0x5E  0x0178  94: DMA14
                EXPORT  DMA15_IRQHandler              [WEAK]  ; 0x5F  0x017C  95: DMA15
                EXPORT  USBDMA_IRQHandler             [WEAK]  ; 0x60  0x0180  96: USBDMA
                EXPORT  WDT2_IRQHandler               [WEAK]  ; 0x61  0x0184  97: Watchdog Timer 2 
                EXPORT  ECC_IRQHandler                [WEAK]  ; 0x62  0x0188  98: Error Correction 
                EXPORT  DVS_IRQHandler                [WEAK]  ; 0x63  0x018C  99: DVS Controller 
                EXPORT  SIMO_IRQHandler               [WEAK]  ; 0x64 0x0190  100: SIMO Controller 
                EXPORT  SCA_IRQHandler                [WEAK]  ; 0x65  0x0194  101: SCA 
                EXPORT  AUDIO_IRQHandler              [WEAK]  ; 0x66  0x0198  102: Audio subsystem 
                EXPORT  FLC1_IRQHandler               [WEAK]  ; 0x67  0x019C  103: Flash Control 1 
                EXPORT  RSV88_IRQHandler              [WEAK]  ; 0x68  0x01A0  104: UART 3 
                EXPORT  RSV89_IRQHandler              [WEAK]  ; 0x69  0x01A4  105: UART 4 
                EXPORT  RSV90_IRQHandler              [WEAK]  ; 0x6A  0x01A8  106: UART 5 
                EXPORT  RSV91_IRQHandler              [WEAK]  ; 0x6B  0x01AC  107: Camera IF 
                EXPORT  RSV92_IRQHandler              [WEAK]  ; 0x6C  0x01B0  108: I3C 
                EXPORT  HTMR0_IRQHandler              [WEAK]  ; 0x6D  0x01B4  109: HTimer0 
                EXPORT  HTMR1_IRQHandler              [WEAK]  ; 0x6E  0x01B8  110: HTimer1 


;*******************************************************************************
; Default handler implementations
;*******************************************************************************
PF_IRQHandler
WDT0_IRQHandler
USB_IRQHandler   
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
SPI2_IRQHandler
RSV18_IRQHandler 
RSV19_IRQHandler 
ADC_IRQHandler   
RSV21_IRQHandler 
RSV22_IRQHandler 
FLC0_IRQHandler 
GPIO0_IRQHandler 
GPIO1_IRQHandler 
RSV26_IRQHandler 
TPU_IRQHandler    
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
SPIXFC_IRQHandler
BTLE_TX_DONE_IRQHandler      
BTLE_RX_RCVD_IRQHandler      
BTLE_RX_ENG_DET_IRQHandler   
BTLE_SFD_DET_IRQHandler      
BTLE_SFD_TO_IRQHandler       
BTLE_GP_EVENT_IRQHandler     
BTLE_CFO_IRQHandler
BTLE_SIG_DET_IRQHandler      
BTLE_AGC_EVENT_IRQHandler    
BTLE_RFFE_SPIM_IRQHandler    
BTLE_TX_AES_IRQHandler      
BTLE_RX_AES_IRQHandler       
BTLE_INV_APB_ADDR_IRQHandler 
BTLE_IQ_DATA_VALID_IRQHandler 
WUT_IRQHandler 
GPIOWAKE_IRQHandler
RSV55_IRQHandler 
SPI0_IRQHandler
WDT1_IRQHandler  
RSV58_IRQHandler 
PT_IRQHandler   
SDMA0_IRQHandler  
RSV61_IRQHandler 
I2C2_IRQHandler  
RSV63_IRQHandler 
RSV64_IRQHandler 
RSV65_IRQHandler 
SDHC_IRQHandler  
OWM_IRQHandler   
DMA4_IRQHandler  
DMA5_IRQHandler  
DMA6_IRQHandler  
DMA7_IRQHandler  
DMA8_IRQHandler  
DMA9_IRQHandler  
DMA10_IRQHandler 
DMA11_IRQHandler 
DMA12_IRQHandler 
DMA13_IRQHandler 
DMA14_IRQHandler 
DMA15_IRQHandler 
USBDMA_IRQHandler 
WDT2_IRQHandler 
ECC_IRQHandler
DVS_IRQHandler 
SIMO_IRQHandler
SCA_IRQHandler 
AUDIO_IRQHandler  
FLC1_IRQHandler 
RSV88_IRQHandler  
RSV89_IRQHandler  
RSV90_IRQHandler
RSV91_IRQHandler
RSV92_IRQHandler
HTMR0_IRQHandler
HTMR1_IRQHandler    

                B       .
                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
        IF      :DEF:__MICROLIB
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

        ELSE    ; Not using Microlib
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

;*******************************************************************************
; Set up the initial stack and heap
;*******************************************************************************
__user_initial_stackheap\
                PROC

                LDR     R0, =  Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF

                END

;*******************************************************************************
; End of file.
;*******************************************************************************
