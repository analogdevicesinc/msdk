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
    .globl __FlashStart
__isr_vector:
  jal x0, Reset_Handler
  jal x0, illegal_insn_handler
  jal x0, ecall_insn_handler
  jal x0, HardFault_IRQHandler    
  jal x0, RTC_IRQHandler       
  jal x0, I2C0_IRQHandler     
  jal x0, I2C1_IRQHandler 
  jal x0, UART0_IRQHandler      
  jal x0, UART1_IRQHandler     
  jal x0, SPI0_IRQHandler     
  jal x0, SPI1_IRQHandler     
  jal x0, GPIO0_IRQHandler     
  jal x0, GPIO1_IRQHandler     
  jal x0, GPIO2_IRQHandler     
  jal x0, MSRADC_IRQHandler     
  jal x0, CM4_IRQHandler    
  jal x0, DMA_MEM2MEM_IRQHandler     
  jal x0, DMA_SPI0_RX_IRQHandler     
  jal x0, DMA_SPI1_RX_IRQHandler 
  jal x0, DMA_SPI2_RX_IRQHandler    
  jal x0, DMA_UART0_RX_IRQHandler     
  jal x0, DMA_UART1_RX_IRQHandler     
  jal x0, DMA_SC0_RX_IRQHandler    
  jal x0, DMA_I2C0_RX_IRQHandler     
  jal x0, DMA_I2C1_RX_IRQHandler 
  jal x0, DMA_ADC_RX_IRQHandler     
  jal x0, DMA_I2C2_RX_IRQHandler    
  jal x0, DMA_MSRADC_IRQHandler    
  jal x0, RSV28_IRQHandler    
  jal x0, DMA_PCIF_RX_IRQHandler     
  jal x0, DMA_UART2_RX_IRQHandler     
  jal x0, DMA_SPI3_RX_IRQHandler     
  jal x0, RSV32_IRQHandler 
  jal x0, DMA_USBEP1_RX_IRQHandler  
  jal x0, DMA_USBEP2_RX_IRQHandler 
  jal x0, DMA_USBEP3_RX_IRQHandler  
  jal x0, DMA_USBEP4_RX_IRQHandler
  jal x0, DMA_USBEP5_RX_IRQHandler 
  jal x0, DMA_USBEP6_RX_IRQHandler 
  jal x0, DMA_USBEP7_RX_IRQHandler 
  jal x0, DMA_USBEP8_RX_IRQHandler 
  jal x0, DMA_USBEP9_RX_IRQHandler 
  jal x0, DMA_USBEP10_RX_IRQHandler 
  jal x0, DMA_USBEP11_RX_IRQHandler 
  jal x0, DMA_UART3_RX_IRQHandler 
  jal x0, DMA_UART4_RX_IRQHandler 
  jal x0, DMA_UART5_RX_IRQHandler 
  jal x0, DMA_SC1_RX_IRQHandler 
  jal x0, RSV48_IRQHandler 
  jal x0, DMA_SPI0_TX_IRQHandler 
  jal x0, DMA_SPI1_TX_IRQHandler 
  jal x0, DMA_SPI2_TX_IRQHandler 
  jal x0, DMA_UART0_TX_IRQHandler 
  jal x0, DMA_UART1_TX_IRQHandler 
  jal x0, DMA_SC0_TX_IRQHandler 
  jal x0, DMA_I2C0_TX_IRQHandler
  jal x0, DMA_I2C1_TX_IRQHandler
  jal x0, RSV57_IRQHandler
  jal x0, DMA_I2C2_TX_IRQHandler
  jal x0, RSV59_IRQHandler
  jal x0, RSV60_IRQHandler
  jal x0, RSV61_IRQHandler
  jal x0, DMA_UART2_TX_IRQHandler
  jal x0, DMA_SPI3_TX_IRQHandler 

  .section .text
    .align 2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:

  /* set start of flash in mvtec, with vectored mode */
  la    t0, (__FlashStart + 1)
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
  def_irq_handler RTC_IRQHandler       
  def_irq_handler I2C0_IRQHandler     
  def_irq_handler I2C1_IRQHandler 
  def_irq_handler UART0_IRQHandler      
  def_irq_handler UART1_IRQHandler     
  def_irq_handler SPI0_IRQHandler     
  def_irq_handler SPI1_IRQHandler     
  def_irq_handler GPIO0_IRQHandler     
  def_irq_handler GPIO1_IRQHandler     
  def_irq_handler GPIO2_IRQHandler     
  def_irq_handler MSRADC_IRQHandler     
  def_irq_handler CM4_IRQHandler    
  def_irq_handler DMA_MEM2MEM_IRQHandler     
  def_irq_handler DMA_SPI0_RX_IRQHandler     
  def_irq_handler DMA_SPI1_RX_IRQHandler    
  def_irq_handler DMA_SPI2_RX_IRQHandler     
  def_irq_handler DMA_UART0_RX_IRQHandler     
  def_irq_handler DMA_UART1_RX_IRQHandler    
  def_irq_handler DMA_SC0_RX_IRQHandler     
  def_irq_handler DMA_I2C0_RX_IRQHandler 
  def_irq_handler DMA_I2C1_RX_IRQHandler     
  def_irq_handler DMA_ADC_RX_IRQHandler    
  def_irq_handler DMA_I2C2_RX_IRQHandler    
  def_irq_handler DMA_MSRADC_IRQHandler    
  def_irq_handler RSV28_IRQHandler     
  def_irq_handler DMA_PCIF_RX_IRQHandler     
  def_irq_handler DMA_UART2_RX_IRQHandler     
  def_irq_handler DMA_SPI3_RX_IRQHandler 
  def_irq_handler RSV32_IRQHandler 
  def_irq_handler DMA_USBEP1_RX_IRQHandler  
  def_irq_handler DMA_USBEP2_RX_IRQHandler 
  def_irq_handler DMA_USBEP3_RX_IRQHandler  
  def_irq_handler DMA_USBEP4_RX_IRQHandler
  def_irq_handler DMA_USBEP5_RX_IRQHandler 
  def_irq_handler DMA_USBEP6_RX_IRQHandler 
  def_irq_handler DMA_USBEP7_RX_IRQHandler 
  def_irq_handler DMA_USBEP8_RX_IRQHandler 
  def_irq_handler DMA_USBEP9_RX_IRQHandler 
  def_irq_handler DMA_USBEP10_RX_IRQHandler 
  def_irq_handler DMA_USBEP11_RX_IRQHandler 
  def_irq_handler DMA_UART3_RX_IRQHandler 
  def_irq_handler DMA_UART4_RX_IRQHandler 
  def_irq_handler DMA_UART5_RX_IRQHandler 
  def_irq_handler DMA_SCI1_RX_IRQHandler 
  def_irq_handler RSV48_IRQHandler 
  def_irq_handler DMA_SPI0_TX_IRQHandler 
  def_irq_handler DMA_SPI1_TX_IRQHandler 
  def_irq_handler DMA_SPI2_TX_IRQHandler 
  def_irq_handler DMA_UART0_TX_IRQHandler 
  def_irq_handler DMA_UART1_TX_IRQHandler 
  def_irq_handler DMA_SC0_TX_IRQHandler 
  def_irq_handler DMA_I2C0_TX_IRQHandler
  def_irq_handler DMA_I2C1_TX_IRQHandler
  def_irq_handler RSV57_IRQHandler
  def_irq_handler DMA_I2C2_TX_IRQHandler
  def_irq_handler RSV59_IRQHandler
  def_irq_handler RSV60_IRQHandler 
  def_irq_handler RSV61_IRQHandler
  def_irq_handler DMA_UART2_TX_IRQHandler
  def_irq_handler DMA_SPI3_TX_IRQHandler 

  .global _init
  .global _fini
_init:
_fini:
  # These don't have to do anything since we use init_array/fini_array.
  ret 
