.section .text

/* Include the fw_update binary file, include global symbols we can use at link time */
_binary_riscv_bin_start:
.incbin "riscv.bin"
_binary_riscv_bin_end:

.globl _binary_riscv_bin_start
.globl _binary_riscv_bin_end
