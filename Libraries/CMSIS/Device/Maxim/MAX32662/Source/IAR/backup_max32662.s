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
;* Description        : MAX32665 device vector table for IAR EWARM toolchain.
;*                      - Sets the initial SP
;*                      - Sets the initial PC == _iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR
;*                        address, all set as PUBWEAK. User may override any ISR
;*                        defined as PUBWEAK.
;*                      - Branches to main in the C library (which eventually
;*                        calls SystemInit() and main()).
;*                      After Reset the Cortex-M4 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* $Date: 2018-12-13 17:49:15 -0600 (Thu, 13 Dec 2018) $
;* $Revision: 39895 $
;*
;*******************************************************************************
     ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Backup Handler to get the part up to operating mode
;; This function is intended to be used with zero state retention
;; The part will behave as if a reset occured
;;
    SECTION .text:CODE:REORDER:NOROOT(2)
    THUMB
    PUBWEAK Backup_Handler
Backup_Handler
        ; Wait for VregB to reach its setpoint
        LDR        R0, =0x40004400
buck_wait:
        LDR        R1, [R0, #64]
        LSLS       R1, R1, #30
        BPL.N      buck_wait
        ; Switch back to VcoreB
        LDR        R0, =0x40006C00
        LDR        R1, [R0, #16]
        BIC.W     R1, R1, #6
        STR        R1, [R0, #16]
        ; Set a flag in MXC_PWRSEQ->gp1
        LDR        R0, =0x4000684C
        LDR        R1, =0x424B5550
        LDR        R2, [R0]
        STR        R1, [R0]
        ; Jump to the entry point
        BX         R2
    END