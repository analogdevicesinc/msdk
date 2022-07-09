.section .text

/* Include the fw_update binary file, include global symbols we can use at link time */
_binary_fw_update_bin_start:
.incbin "fw_update.bin"
_binary_fw_update_bin_end:

.globl _binary_fw_update_bin_start
.globl _binary_fw_update_bin_end
