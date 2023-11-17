################################################################################
 # Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 #
 ###############################################################################

# This is the name of the build output file
ifeq "$(PROJECT)" ""
PROJECT=max32572
endif

RISCV_PREFIX=riscv-none-embed
export RISCV_PREFIX


.PHONY: all arm clean distclean riscv

all: build/combined.elf

build/combined.elf: buildrv/$(PROJECT)_riscv.elf build/$(PROJECT).elf
	@arm-none-eabi-objcopy -I elf32-littlearm build/$(PROJECT).elf -O ihex build/$(PROJECT).hex
	@cat build/$(PROJECT).hex | sed '/^:00000001FF/d' > build/$(PROJECT)a.hex
	@$(RISCV_PREFIX)-objcopy buildrv/$(PROJECT)_riscv.elf -O ihex buildrv/$(PROJECT)_riscv.hex
	@cat build/$(PROJECT)a.hex buildrv/$(PROJECT)_riscv.hex > build/combined.hex
	@arm-none-eabi-objcopy -I ihex build/combined.hex -O elf32-littlearm build/combined.elf


arm:
	@$(MAKE) -f Makefile.ARM


# Makefile will generate following RISCV files for ARM compiler to use
RISCV_BIN_FILE = riscv_bin.bin
RISCV_LOAD_ADDR_FILE = riscv_load.addr
RISCV_TEXT_ADDR_FILE = riscv_text.addr

riscv:
	@$(MAKE) -f Makefile.RISCV
	@$(RISCV_PREFIX)-objcopy buildrv/$(PROJECT)_riscv.elf -O binary $(RISCV_BIN_FILE)
	@$(RISCV_PREFIX)-objcopy buildrv/$(PROJECT)_riscv.elf -O verilog $(RISCV_LOAD_ADDR_FILE)
	@sed -i 's/@/0x/;1q' $(RISCV_LOAD_ADDR_FILE)
	@$(RISCV_PREFIX)-objcopy -j .text buildrv/$(PROJECT)_riscv.elf -O verilog $(RISCV_TEXT_ADDR_FILE)
	@sed -i 's/@/0x/;1q' $(RISCV_TEXT_ADDR_FILE)
	@sed -i 's/\.incbin.*$$/\.incbin "$(RISCV_BIN_FILE)"/' riscv.S

build/$(PROJECT).elf: arm

buildrv/$(PROJECT)_riscv.elf: riscv

clean distclean:
	@$(MAKE) -f Makefile.ARM $@
	@$(MAKE) -f Makefile.RISCV $@
	rm -f $(RISCV_BIN_FILE)
	rm -f $(RISCV_LOAD_ADDR_FILE)
	rm -f $(RISCV_TEXT_ADDR_FILE)


