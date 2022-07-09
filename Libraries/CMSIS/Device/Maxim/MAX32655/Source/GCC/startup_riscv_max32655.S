// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#define EXCEPTION_STACK_SIZE 96

/* ========================================================= [ entry ] === */
    .section .stack
    .align 3

    /* Symbols for the memory segments */
    .equ      _FLASH_SIZE, FLASH_SIZE
    .globl    _FLASH_SIZE
    .equ      _FLASH_ORIGIN, FLASH_ORIGIN
    .globl    _FLASH_ORIGIN
    .equ      _SRAM_SIZE, SRAM_SIZE
    .globl    _SRAM_SIZE
    .equ      _SRAM_ORIGIN, SRAM_ORIGIN
    .globl    _SRAM_ORIGIN

#ifdef RISCV_SRAM_SIZE
    .equ    _RISCV_SRAM_SIZE, RISCV_SRAM_SIZE
#else
#error "RISCV_SRAM_SIZE SIZE undefined"
#endif
    .globl    _RISCV_SRAM_SIZE

#ifdef RISCV_FLASH_SIZE
    .equ    _RISCV_FLASH_SIZE, RISCV_FLASH_SIZE
#else
#error "RISCV_FLASH_SIZE SIZE undefined"
#endif
    .globl    _RISCV_FLASH_SIZE

#ifdef MAILBOX_SIZE
    .equ    _MAILBOX_SIZE, MAILBOX_SIZE
#else
    .equ    _MAILBOX_SIZE, 0x0
#endif
    .globl    _MAILBOX_SIZE

    /* Flash storage for PAL layer */
#ifdef PAL_NVM_SIZE
    .equ    _PAL_NVM_SIZE, PAL_NVM_SIZE
#else
    .equ    _PAL_NVM_SIZE, 0x0
#endif
    .globl    _PAL_NVM_SIZE

#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0x0001000
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

   .section .sram3_code_section,"a",%progbits
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
  jal x0, RSV00_IRQHandler    
  jal x0, PF_IRQHandlerWrap
  jal x0, WDT0_IRQHandlerWrap
  jal x0, GPIOWAKE_IRQHandlerWrap
  jal x0, RTC_IRQHandlerWrap
  jal x0, TMR0_IRQHandlerWrap
  jal x0, TMR1_IRQHandlerWrap
  jal x0, TMR2_IRQHandlerWrap
  jal x0, TMR3_IRQHandlerWrap
  jal x0, TMR4_IRQHandlerWrap
  jal x0, TMR5_IRQHandlerWrap
  jal x0, I2C0_IRQHandlerWrap
  jal x0, UART0_IRQHandlerWrap
  jal x0, RSV16_IRQHandlerWrap
  jal x0, I2C1_IRQHandlerWrap
  jal x0, UART1_IRQHandlerWrap
  jal x0, UART2_IRQHandlerWrap
  jal x0, I2C2_IRQHandlerWrap
  jal x0, UART3_IRQHandlerWrap
  jal x0, SPI1_IRQHandlerWrap
  jal x0, WUT_IRQHandlerWrap
  jal x0, FLC0_IRQHandlerWrap
  jal x0, GPIO0_IRQHandlerWrap
  jal x0, GPIO1_IRQHandlerWrap
  jal x0, GPIO2_IRQHandlerWrap
  jal x0, DMA0_IRQHandlerWrap
  jal x0, DMA1_IRQHandlerWrap
  jal x0, DMA2_IRQHandlerWrap
  jal x0, DMA3_IRQHandlerWrap
  jal x0, BTLE_TX_DONE_IRQHandlerWrap
  jal x0, BTLE_RX_RCVD_IRQHandlerWrap
  jal x0, BTLE_RX_ENG_DET_IRQHandlerWrap
  jal x0, BTLE_SFD_DET_IRQHandlerWrap
  jal x0, BTLE_SFD_TO_IRQHandlerWrap
  jal x0, BTLE_GP_EVENT_IRQHandlerWrap
  jal x0, BTLE_CFO_IRQHandlerWrap
  jal x0, BTLE_SIG_DET_IRQHandlerWrap
  jal x0, BTLE_AGC_EVENT_IRQHandlerWrap
  jal x0, BTLE_RFFE_SPIM_IRQHandlerWrap
  jal x0, BTLE_TX_AES_IRQHandlerWrap
  jal x0, BTLE_RX_AES_IRQHandlerWrap
  jal x0, BTLE_INV_APB_ADDR_IRQHandlerWrap
  jal x0, BTLE_IQ_DATA_VALID_IRQHandlerWrap
  jal x0, AES_IRQHandlerWrap
  jal x0, TRNG_IRQHandlerWrap
  jal x0, WDT1_IRQHandlerWrap
  jal x0, DVS_IRQHandlerWrap
  jal x0, SIMO_IRQHandlerWrap
  jal x0, RSV51_IRQHandlerWrap
  jal x0, PT_IRQHandlerWrap
  jal x0, ADC_IRQHandlerWrap
  jal x0, OWM_IRQHandlerWrap
  jal x0, I2S_IRQHandlerWrap
  jal x0, CNN_FIFO_IRQHandlerWrap
  jal x0, CNN_IRQHandlerWrap
  jal x0, RSV58_IRQHandlerWrap
  jal x0, PCIF_IRQHandlerWrap
  jal x0, MXC_IRQ_EXT_COUNT_IRQHandlerWrap

  .section .text
    .align 2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:

  /* set 0 in mtvec (base for IVT) */
  csrrw x0, mtvec, x0

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

/* TODO: Why is this symbol causing problems? Is it even necessary to have here? */
/* _start:
 *   .global _start
 */

/*     Loops to copy data from read only memory to RAM. The ranges
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
 *    will be weak symbol and just dead loops. They can be
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
  def_irq_handler RSV00_IRQHandler    
  def_irq_handler PF_IRQHandlerWrap
  def_irq_handler WDT0_IRQHandlerWrap
  def_irq_handler GPIOWAKE_IRQHandlerWrap
  def_irq_handler RTC_IRQHandlerWrap
  def_irq_handler TMR0_IRQHandlerWrap
  def_irq_handler TMR1_IRQHandlerWrap
  def_irq_handler TMR2_IRQHandlerWrap
  def_irq_handler TMR3_IRQHandlerWrap
  def_irq_handler TMR4_IRQHandlerWrap
  def_irq_handler TMR5_IRQHandlerWrap
  def_irq_handler I2C0_IRQHandlerWrap
  def_irq_handler UART0_IRQHandlerWrap
  def_irq_handler RSV16_IRQHandlerWrap
  def_irq_handler I2C1_IRQHandlerWrap
  def_irq_handler UART1_IRQHandlerWrap
  def_irq_handler UART2_IRQHandlerWrap
  def_irq_handler I2C2_IRQHandlerWrap
  def_irq_handler UART3_IRQHandlerWrap
  def_irq_handler SPI1_IRQHandlerWrap
  def_irq_handler WUT_IRQHandlerWrap
  def_irq_handler FLC0_IRQHandlerWrap
  def_irq_handler GPIO0_IRQHandlerWrap
  def_irq_handler GPIO1_IRQHandlerWrap
  def_irq_handler GPIO2_IRQHandlerWrap
  def_irq_handler DMA0_IRQHandlerWrap
  def_irq_handler DMA1_IRQHandlerWrap
  def_irq_handler DMA2_IRQHandlerWrap
  def_irq_handler DMA3_IRQHandlerWrap
  def_irq_handler BTLE_TX_DONE_IRQHandlerWrap
  def_irq_handler BTLE_RX_RCVD_IRQHandlerWrap
  def_irq_handler BTLE_RX_ENG_DET_IRQHandlerWrap
  def_irq_handler BTLE_SFD_DET_IRQHandlerWrap
  def_irq_handler BTLE_SFD_TO_IRQHandlerWrap
  def_irq_handler BTLE_GP_EVENT_IRQHandlerWrap
  def_irq_handler BTLE_CFO_IRQHandlerWrap
  def_irq_handler BTLE_SIG_DET_IRQHandlerWrap
  def_irq_handler BTLE_AGC_EVENT_IRQHandlerWrap
  def_irq_handler BTLE_RFFE_SPIM_IRQHandlerWrap
  def_irq_handler BTLE_TX_AES_IRQHandlerWrap
  def_irq_handler BTLE_RX_AES_IRQHandlerWrap
  def_irq_handler BTLE_INV_APB_ADDR_IRQHandlerWrap
  def_irq_handler BTLE_IQ_DATA_VALID_IRQHandlerWrap
  def_irq_handler AES_IRQHandlerWrap
  def_irq_handler TRNG_IRQHandlerWrap
  def_irq_handler WDT1_IRQHandlerWrap
  def_irq_handler DVS_IRQHandlerWrap
  def_irq_handler SIMO_IRQHandlerWrap
  def_irq_handler RSV51_IRQHandlerWrap
  def_irq_handler PT_IRQHandlerWrap
  def_irq_handler ADC_IRQHandlerWrap
  def_irq_handler OWM_IRQHandlerWrap
  def_irq_handler I2S_IRQHandlerWrap
  def_irq_handler CNN_FIFO_IRQHandlerWrap
  def_irq_handler CNN_IRQHandlerWrap
  def_irq_handler RSV58_IRQHandlerWrap
  def_irq_handler PCIF_IRQHandlerWrap
  def_irq_handler MXC_IRQ_EXT_COUNT_IRQHandlerWrap


  .global _init
  .global _fini
_init:
_fini:
  # These don't have to do anything since we use init_array/fini_array.
  ret 
