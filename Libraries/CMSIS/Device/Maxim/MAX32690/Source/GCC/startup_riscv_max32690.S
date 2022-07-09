// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include "pulpino.h"

#define EXCEPTION_STACK_SIZE 96

    /* Symbols for the memory segments */
    .equ      _RISCV_FLASH_SIZE, RISCV_FLASH_SIZE
    .globl    _RISCV_FLASH_SIZE
    .equ      _RISCV_FLASH_ORIGIN, RISCV_FLASH_ORIGIN
    .globl    _RISCV_FLASH_ORIGIN
    .equ      _RISCV_SRAM_SIZE, RISCV_SRAM_SIZE
    .globl    _RISCV_SRAM_SIZE
    .equ      _RISCV_SRAM_ORIGIN, RISCV_SRAM_ORIGIN
    .globl    _RISCV_SRAM_ORIGIN

/* ========================================================= [ entry ] === */
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

   .section .sram7_code_section,"a",%progbits
   .align 2
   .globl __ram_startup
__ram_startup:
/* long jump ... if section is in different memory segment */
  lui   t0, %hi(__isr_vector)
  addi  t0, t0, %lo(__isr_vector)
  jalr  t0

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
  jal x0, Reset_Handler
  jal x0, illegal_insn_handler
  jal x0, ecall_insn_handler
  jal x0, HardFault_IRQHandler    
  jal x0, PF_IRQHandler       
  jal x0, WDT0_IRQHandler     
  jal x0, GPIOWAKE_IRQHandler 
  jal x0, RTC_IRQHandler      
  jal x0, TMR0_IRQHandler     
  jal x0, TMR1_IRQHandler     
  jal x0, TMR2_IRQHandler     
  jal x0, TMR3_IRQHandler     
  jal x0, TMR4_IRQHandler     
  jal x0, TMR5_IRQHandler     
  jal x0, I2C0_IRQHandler     
  jal x0, UART0_IRQHandler    
  jal x0, CM4_IRQHandler     
  jal x0, I2C1_IRQHandler     
  jal x0, UART1_IRQHandler    
  jal x0, UART2_IRQHandler     
  jal x0, I2C2_IRQHandler     
  jal x0, UART3_IRQHandler    
  jal x0, SPI0_IRQHandler     
  jal x0, WUT0_IRQHandler 
  jal x0, FLC1_IRQHandler     
  jal x0, GPIO0_IRQHandler    
  jal x0, GPIO1_IRQHandler    
  jal x0, GPIO3_IRQHandler    
  jal x0, DMA0_IRQHandler     
  jal x0, DMA1_IRQHandler     
  jal x0, DMA2_IRQHandler     
  jal x0, DMA3_IRQHandler 
  jal x0, BTLE_TX_DONE_IRQHandler 
  jal x0, BTLE_RX_RCVD_IRQHandler  
  jal x0, BTLE_RX_ENG_DET_IRQHandler 
  jal x0, BTLE_SFD_DET_IRQHandler  
  jal x0, BTLE_SFD_TO_IRQHandler
  jal x0, BTLE_GP_EVENT_IRQHandler 
  jal x0, BTLE_CFO_IRQHandler 
  jal x0, BTLE_SIG_DET_IRQHandler 
  jal x0, BTLE_AGC_EVENT_IRQHandler 
  jal x0, BTLE_RFFE_SPIM_IRQHandler 
  jal x0, BTLE_TX_AES_IRQHandler 
  jal x0, BTLE_RX_AES_IRQHandler 
  jal x0, BTLE_INV_APB_ADDR_IRQHandler 
  jal x0, BTLE_IQ_DATA_VALID_IRQHandler 
  jal x0, DMA4_15_IRQHandler 
  jal x0, TRNG_IRQHandler 
  jal x0, WDT1_IRQHandler 
  jal x0, RSV49_IRQHandler 
  jal x0, RSV50_IRQHandler 
  jal x0, WUT1_IRQHandler 
  jal x0, PT_IRQHandler 
  jal x0, ADC_IRQHandler 
  jal x0, OWM_IRQHandler 
  jal x0, I2S_IRQHandler
  jal x0, RSV56_IRQHandler
  jal x0, RSV57_IRQHandler
  jal x0, CAN0_IRQHandler
  jal x0, RSV59_IRQHandler
  jal x0, GPIO2_IRQHandler
  jal x0, SPI1_IRQHandler
  jal x0, SPI2_IRQHandler
  jal x0, CAN1_IRQHandler 

  .section .text
    .align 2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:

  /* set start of flash in mvtec, with vectored mode */
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

  /* def_irq_handler Reset_Handler */
  def_irq_handler illegal_insn_handler
  def_irq_handler ecall_insn_handler
  def_irq_handler HardFault_IRQHandler    
  def_irq_handler PF_IRQHandler       
  def_irq_handler WDT0_IRQHandler     
  def_irq_handler GPIOWAKE_IRQHandler 
  def_irq_handler RTC_IRQHandler      
  def_irq_handler TMR0_IRQHandler     
  def_irq_handler TMR1_IRQHandler     
  def_irq_handler TMR2_IRQHandler     
  def_irq_handler TMR3_IRQHandler     
  def_irq_handler TMR4_IRQHandler     
  def_irq_handler TMR5_IRQHandler     
  def_irq_handler I2C0_IRQHandler     
  def_irq_handler UART0_IRQHandler    
  def_irq_handler CM4_IRQHandler     
  def_irq_handler I2C1_IRQHandler     
  def_irq_handler UART1_IRQHandler    
  def_irq_handler UART2_IRQHandler     
  def_irq_handler I2C2_IRQHandler     
  def_irq_handler UART3_IRQHandler    
  def_irq_handler SPI0_IRQHandler     
  def_irq_handler WUT0_IRQHandler 
  def_irq_handler FLC1_IRQHandler     
  def_irq_handler GPIO0_IRQHandler    
  def_irq_handler GPIO1_IRQHandler    
  def_irq_handler GPIO3_IRQHandler    
  def_irq_handler DMA0_IRQHandler     
  def_irq_handler DMA1_IRQHandler     
  def_irq_handler DMA2_IRQHandler     
  def_irq_handler DMA3_IRQHandler 
  def_irq_handler BTLE_TX_DONE_IRQHandler 
  def_irq_handler BTLE_RX_RCVD_IRQHandler  
  def_irq_handler BTLE_RX_ENG_DET_IRQHandler 
  def_irq_handler BTLE_SFD_DET_IRQHandler  
  def_irq_handler BTLE_SFD_TO_IRQHandler
  def_irq_handler BTLE_GP_EVENT_IRQHandler 
  def_irq_handler BTLE_CFO_IRQHandler 
  def_irq_handler BTLE_SIG_DET_IRQHandler 
  def_irq_handler BTLE_AGC_EVENT_IRQHandler 
  def_irq_handler BTLE_RFFE_SPIM_IRQHandler 
  def_irq_handler BTLE_TX_AES_IRQHandler 
  def_irq_handler BTLE_RX_AES_IRQHandler 
  def_irq_handler BTLE_INV_APB_ADDR_IRQHandler 
  def_irq_handler BTLE_IQ_DATA_VALID_IRQHandler 
  def_irq_handler DMA4_15_IRQHandler 
  def_irq_handler TRNG_IRQHandler 
  def_irq_handler WDT1_IRQHandler 
  def_irq_handler RSV49_IRQHandler 
  def_irq_handler RSV50_IRQHandler 
  def_irq_handler WUT1_IRQHandler 
  def_irq_handler PT_IRQHandler 
  def_irq_handler ADC_IRQHandler 
  def_irq_handler OWM_IRQHandler 
  def_irq_handler I2S_IRQHandler
  def_irq_handler RSV56_IRQHandler
  def_irq_handler RSV57_IRQHandler
  def_irq_handler CAN0_IRQHandler
  def_irq_handler RSV59_IRQHandler
  def_irq_handler GPIO2_IRQHandler 
  def_irq_handler SPI1_IRQHandler
  def_irq_handler SPI2_IRQHandler
  def_irq_handler CAN1_IRQHandler 


  .global _init
  .global _fini
_init:
_fini:
  # These don't have to do anything since we use init_array/fini_array.
  ret 
