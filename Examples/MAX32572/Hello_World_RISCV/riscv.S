.section .rodata

/* Include RISCV image binary file, declare global symbols we can use at link time */

.ALIGN(4)
_binary_riscv_bin_start:

.incbin "riscv_bin.bin"

.ALIGN(4)
_binary_riscv_bin_end:

.globl _binary_riscv_bin_start
.globl _binary_riscv_bin_end
