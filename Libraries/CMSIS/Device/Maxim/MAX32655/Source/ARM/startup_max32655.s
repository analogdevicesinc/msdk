;******************************************************************************
;
; Copyright (C) 2018-2023 Maxim Integrated Products, Inc. (now owned by 
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
__StackTop


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

__isr_vector    DCD     __StackTop                ; Top of Stack
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
                DCD Fake16_IRQHandler         ; 16:01 -- No IRQ assignments yet
                DCD Fake17_IRQHandler         ; 17:02 -- No IRQ assignments yet
                DCD Fake18_IRQHandler         ; 18:03 -- No IRQ assignments yet
__isr_vector_End

__isr_vector_Size       EQU  __isr_vector_End - __isr_vector

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
                              
NMI_Handler             PROC    
                        EXPORT NMI_Handler              [WEAK]
                        B       NMI_Handler
                        ENDP

HardFault_Handler       PROC    
                        EXPORT HardFault_Handler        [WEAK]
                        B       HardFault_Handler
                        ENDP

MemManage_Handler       PROC    
                        EXPORT MemManage_Handler        [WEAK]
                        B       MemManage_Handler
                        ENDP

BusFault_Handler        PROC    
                        EXPORT BusFault_Handler         [WEAK]
                        B       BusFault_Handler
                        ENDP

UsageFault_Handler      PROC    
                        EXPORT UsageFault_Handler       [WEAK]
                        B       UsageFault_Handler
                        ENDP

SVC_Handler             PROC    
                        EXPORT SVC_Handler              [WEAK]
                        B       SVC_Handler
                        ENDP

DebugMon_Handler        PROC    
                        EXPORT DebugMon_Handler         [WEAK]
                        B       DebugMon_Handler
                        ENDP

PendSV_Handler          PROC    
                        EXPORT PendSV_Handler           [WEAK]
                        B       PendSV_Handler
                        ENDP

SysTick_Handler         PROC    
                        EXPORT SysTick_Handler          [WEAK]
                        B       SysTick_Handler
                        ENDP

Default_Handler         PROC

        ; Device-specific Interrupts
        EXPORT Fake16_IRQHandler                [WEAK] ; 16:01 -- No IRQ assignments yet
        EXPORT Fake17_IRQHandler                [WEAK] ; 17:02 -- No IRQ assignments yet
        EXPORT Fake18_IRQHandler                [WEAK] ; 18:03 -- No IRQ assignments yet

                      
Fake16_IRQHandler
Fake17_IRQHandler
Fake18_IRQHandler

        B .
        ENDP


;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
        IF      :DEF:__MICROLIB

        EXPORT  __StackTop
        EXPORT  __heap_base
        EXPORT  __heap_limit

        ELSE

        IMPORT  __use_two_region_memory
        EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC

        LDR     R0, =  Heap_Mem
        LDR     R1, =(Stack_Mem + Stack_Size)
        LDR     R2, = (Heap_Mem +  Heap_Size)
        LDR     R3, = Stack_Mem
        BX      LR
		ENDP

        ALIGN

        ENDIF

        END


