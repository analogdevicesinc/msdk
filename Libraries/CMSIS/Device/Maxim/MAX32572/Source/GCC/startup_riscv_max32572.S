// Copyright 2017 ETH Zurich and University of Bologna.
// Portions Copyright (C) 2023 Analog Devices, Inc.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

/* Stack */
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

/* Heap */
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


/* unused IRQs sent to RSV0_IRQHandler */
    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
  jal x0, Reset_Handler         /* vector 0 */
  jal x0, illegal_insn_handler  /* 1 */
  jal x0, ecall_insn_handler    /* 2 */
  jal x0, HardFault_IRQHandler  /* 3 */
  jal x0, CM4_IRQHandler        /* 4    CM4 from CPU0 */
  jal x0, RSV0_IRQHandler       /* 5 */
  jal x0, RSV0_IRQHandler       /* 6 */
  jal x0, WDT0_IRQHandler       /* 7    WWDT0 */

  jal x0, WDT1_IRQHandler     /* vector 8    WWDT1 */
  jal x0, RTC_IRQHandler      /* 9    RTC */
  jal x0, MSRADC_IRQHandler   /* 10   MSR ADC9 */
  jal x0, RSV0_IRQHandler     /* 11 */
  jal x0, TMR0_IRQHandler     /* 12   TMR0 */
  jal x0, TMR1_IRQHandler     /* 13   TMR1 */
  jal x0, TMR2_IRQHandler     /* 14   TMR2 */
  jal x0, TMR3_IRQHandler     /* 15   TMR3 */

  jal x0, TMR4_IRQHandler     /* vector 16   TMR4 */
  jal x0, TMR5_IRQHandler     /* 17   MSR TMR5 */
  jal x0, RSV0_IRQHandler     /* 18 */
  jal x0, RSV0_IRQHandler     /* 19 */
  jal x0, GPIO0_IRQHandler    /* 20   GPIO0 */
  jal x0, GPIO1_IRQHandler    /* 21   GPIO1 */
  jal x0, RSV0_IRQHandler     /* 22 */
  jal x0, RSV0_IRQHandler     /* 23 */

  jal x0, I2C0_IRQHandler     /* vector 24   I2C0 */
  jal x0, SPI0_IRQHandler     /* 25   SPI0 */
  jal x0, UART0_IRQHandler    /* 26   UART0 */
  jal x0, I2C1_IRQHandler     /* 27   I2C1 */
  jal x0, SPI1_IRQHandler     /* 28   SPI1 */
  jal x0, UART1_IRQHandler    /* 29   UART1 */
  jal x0, RSV0_IRQHandler     /* 30 */
  jal x0, RSV0_IRQHandler     /* 31 */

  jal x0, UART2_IRQHandler    /* 32   UART2 */
  jal x0, SPI3_IRQHandler     /* 33   SPI3 */
  jal x0, UART3_IRQHandler    /* 34   UART3 */
  jal x0, RSV0_IRQHandler     /* 35 */
  jal x0, RSV0_IRQHandler     /* 36 */
  jal x0, RSV0_IRQHandler     /* 37 */
  jal x0, TRNG_IRQHandler     /* 38   TRNG */
  jal x0, CRYPTO_IRQHandler   /* 39   CRYPTO */

  jal x0, SCA_IRQHandler      /* vector 40   SCA Accelerator */
  jal x0, SCARD0_IRQHandler   /* 41   SCARD0 */
  jal x0, SCARD1_IRQHandler   /* 42   SCARD1 */
  jal x0, SKPD_IRQHandler     /* 43   Secure Keypad */
  jal x0, ADC_IRQHandler      /* 44   ADC10 */
  jal x0, PT_IRQHandler       /* 45   Pulse Train */
  jal x0, RSV0_IRQHandler     /* 46 */
  jal x0, RSV0_IRQHandler     /* 47 */

  jal x0, RSV0_IRQHandler     /* vector 48 */
  jal x0, RSV0_IRQHandler     /* 49 */
  jal x0, HTMR0_IRQHandler    /* 50   HTIMER0 */
  jal x0, HTMR1_IRQHandler    /* 51   HTIMER1 */
  jal x0, SPIXIP_IRQHandler   /* 52   SPI XIP */
  jal x0, RSV0_IRQHandler     /* 53 */
  jal x0, RSV0_IRQHandler     /* 54 */
  jal x0, DMA0_IRQHandler     /* 55   DMA0 */

  jal x0, DMA1_IRQHandler     /* vector 56   DMA1 */
  jal x0, DMA2_IRQHandler     /* 57   DMA2 */
  jal x0, DMA3_IRQHandler     /* 58   DMA3 */
  jal x0, DMA4_IRQHandler     /* 59   DMA4 */
  jal x0, DMA5_IRQHandler     /* 60   DMA5 */
  jal x0, DMA6_IRQHandler     /* 61   DMA6 */
  jal x0, DMA7_IRQHandler     /* 62   DMA7 */
  jal x0, DMA815_IRQHandler   /* 63   DMA8-DMA15 */

/* Main code */
  .section .text
  .align 2
  .globl   Reset_Handler
  .type    Reset_Handler, %function
Reset_Handler:

  /* set start of ISR table in mvtec, with vectored mode */
  la    t0, (__isr_vector + 1)
  csrrw x0, mtvec, t0

  /* Initialize Stack Pointer */
  la    sp, __StackTop

  /* Initialize Global Pointer */
.option push
.option norelax
  la    gp, __global_pointer$
.option pop

  /* PreInit runs before any RAM initialization. Example usage: SPID setup, etc */
  call  PreInit
  bnez  a0, SKIPRAMINIT

_start:
  .global _start

/*     Loopssel to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __load_data: Where data sections are saved.
 *      _data /_edata: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

zero_sbss:
  la   t0, _sbss
  la   t1, _esbss
  bge  t0, t1, zero_sbss_loop_end

zero_sbss_loop:
  sw   x0, 0(t0)
  addi t0, t0, 4
  blt  t0, t1, zero_sbss_loop

zero_sbss_loop_end:
  la   t0, __load_sdata
  la   t1, _sdata
  la   t2, _esdata
  bge  t1, t2, init_sdata_loop_end

/* MLB - There must be a way to speed this up... */
init_sdata_loop:
  lw   t3, 0(t0)
  sw   t3, 0(t1)
  addi t0, t0, 4
  addi t1, t1, 4
  blt  t1, t2, init_sdata_loop
init_sdata_loop_end:

/* MLB - There must be a way to speed this up... */
  la   t0, __load_data
  la   t1, _data
  la   t2, _edata
  bge  t1, t2,init_data_loop_end

init_data_loop:
  lw   t3, 0(t0)
  sw   t3, 0(t1)
  addi t0, t0, 4
  addi t1, t1, 4
  blt  t1, t2,init_data_loop
init_data_loop_end:

zero_bss:
  la   t0, _bss
  la   t1, _ebss
  bge  t0, t1, zero_bss_loop_end

zero_bss_loop:
  sw   x0, 0(t0)
  addi t0, t0, 4
  blt  t0, t1, zero_bss_loop

zero_bss_loop_end:
SKIPRAMINIT:

  /* Perform System initialization after RAM initialization */
  call SystemInit

  /* Run global initialization functions */
  call __libc_init_array

main_entry:

  /* jump to main program entry point */
  call main

.SPIN:
    /* spin if main ever returns. */
  jal .SPIN

/* =================================================== [ exceptions ] === */
/* This section has to be down here, since we have to disable rvc for it  */
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loopssel. They can be
 *    overwritten by other handlers */
/* HACK NI - move up macro to be used in ISR vector table*/
    .macro    def_irq_handler    handler_name
    .align 2
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
  jal    .
    .size    \handler_name, . - \handler_name
    .endm

  /* trap non-existing IRQ vectors in RSV0_IRQHandler */
  def_irq_handler RSV0_IRQHandler

#  def_irq_handler Reset_Handler         /* vector 0 */
  def_irq_handler illegal_insn_handler  /* 1 */
  def_irq_handler ecall_insn_handler    /* 2 */
  def_irq_handler HardFault_IRQHandler  /* 3 */
  def_irq_handler CM4_IRQHandler        /* 4    CM4 from CPU0 */
#  def_irq_handler RSV0_IRQHandler       /* 5 */
#  def_irq_handler RSV0_IRQHandler       /* 6 */
  def_irq_handler WDT0_IRQHandler       /* 7    WWDT0 */

  def_irq_handler WDT1_IRQHandler     /* vector 8    WWDT1 */
  def_irq_handler RTC_IRQHandler      /* 9    RTC */
  def_irq_handler MSRADC_IRQHandler   /* 10   MSR ADC9 */
#  def_irq_handler RSV0_IRQHandler     /* 11 */
  def_irq_handler TMR0_IRQHandler     /* 12   TMR0 */
  def_irq_handler TMR1_IRQHandler     /* 13   TMR1 */
  def_irq_handler TMR2_IRQHandler     /* 14   TMR2 */
  def_irq_handler TMR3_IRQHandler     /* 15   TMR3 */

  def_irq_handler TMR4_IRQHandler     /* vector 16   TMR4 */
  def_irq_handler TMR5_IRQHandler     /* 17   MSR TMR5 */
#  def_irq_handler RSV0_IRQHandler     /* 18 */
#  def_irq_handler RSV0_IRQHandler     /* 19 */
  def_irq_handler GPIO0_IRQHandler    /* 20   GPIO0 */
  def_irq_handler GPIO1_IRQHandler    /* 21   GPIO1 */
#  def_irq_handler RSV0_IRQHandler     /* 22 */
#  def_irq_handler RSV0_IRQHandler     /* 23 */

  def_irq_handler I2C0_IRQHandler     /* vector 24   I2C0 */
  def_irq_handler SPI0_IRQHandler     /* 25   SPI0 */
  def_irq_handler UART0_IRQHandler    /* 26   UART0 */
  def_irq_handler I2C1_IRQHandler     /* 27   I2C1 */
  def_irq_handler SPI1_IRQHandler     /* 28   SPI1 */
  def_irq_handler UART1_IRQHandler    /* 29   UART1 */
#  def_irq_handler RSV0_IRQHandler     /* 30 */
#  def_irq_handler RSV0_IRQHandler     /* 31 */

  def_irq_handler UART2_IRQHandler    /* 32   UART2 */
  def_irq_handler SPI3_IRQHandler     /* 33   SPI3 */
  def_irq_handler UART3_IRQHandler    /* 34   UART3 */
#  def_irq_handler RSV0_IRQHandler     /* 35 */
#  def_irq_handler RSV0_IRQHandler     /* 36 */
#  def_irq_handler RSV0_IRQHandler     /* 37 */
  def_irq_handler TRNG_IRQHandler     /* 38   TRNG */
  def_irq_handler CRYPTO_IRQHandler   /* 39   CRYPTO */

  def_irq_handler SCA_IRQHandler      /* vector 40   SCA Accelerator */
  def_irq_handler SCARD0_IRQHandler   /* 41   SCARD0 */
  def_irq_handler SCARD1_IRQHandler   /* 42   SCARD1 */
  def_irq_handler SKPD_IRQHandler     /* 43   Secure Keypad */
  def_irq_handler ADC_IRQHandler      /* 44   ADC10 */
  def_irq_handler PT_IRQHandler       /* 45   Pulse Train */
#  def_irq_handler RSV0_IRQHandler     /* 46 */
#  def_irq_handler RSV0_IRQHandler     /* 47 */

#  def_irq_handler RSV0_IRQHandler     /* vector 48 */
#  def_irq_handler RSV0_IRQHandler     /* 49 */
  def_irq_handler HTMR0_IRQHandler    /* 50   HTIMER0 */
  def_irq_handler HTMR1_IRQHandler    /* 51   HTIMER1 */
  def_irq_handler SPIXIP_IRQHandler   /* 52   SPI XIP */
#  def_irq_handler RSV0_IRQHandler     /* 53 */
#  def_irq_handler RSV0_IRQHandler     /* 54 */
  def_irq_handler DMA0_IRQHandler     /* 55   DMA0 */

  def_irq_handler DMA1_IRQHandler     /* vector 56   DMA1 */
  def_irq_handler DMA2_IRQHandler     /* 57   DMA2 */
  def_irq_handler DMA3_IRQHandler     /* 58   DMA3 */
  def_irq_handler DMA4_IRQHandler     /* 59   DMA4 */
  def_irq_handler DMA5_IRQHandler     /* 60   DMA5 */
  def_irq_handler DMA6_IRQHandler     /* 61   DMA6 */
  def_irq_handler DMA7_IRQHandler     /* 62   DMA7 */
  def_irq_handler DMA815_IRQHandler   /* 63   DMA8-DMA15 */

  .global _init
  .global _fini
_init:
_fini:
  # These don't have to do anything since we use init_array/fini_array.
  ret 
