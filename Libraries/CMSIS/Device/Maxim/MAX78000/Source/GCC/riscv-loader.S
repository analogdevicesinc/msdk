.section .text

_binary_riscv_bin_start:
.incbin "build/buildrv/riscv.bin"
_binary_riscv_bin_end:

.globl _binary_riscv_bin_start
.globl _binary_riscv_bin_end
