/* Include the bl binary file, plus a reference for addition into linker script */
.section .text
_binary_bootloader_bin_start:
.incbin "build/buildbl/bootloader.bin"
_binary_bootloader_bin_end:

.globl _binary_bootloader_bin_start
.globl _binary_bootloader_bin_end
