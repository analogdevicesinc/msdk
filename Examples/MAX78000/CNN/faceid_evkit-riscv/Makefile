################################################################################
 # Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ###############################################################################

ifeq "$(PROJECT)" ""
PROJECT=max78000
endif

.PHONY: all arm release clean distclean riscv
all: build/$(PROJECT)-combined.elf

build/$(PROJECT)-combined.elf: build/$(PROJECT).elf buildrv/$(PROJECT).elf arm riscv
	@arm-none-eabi-objcopy -I elf32-littlearm build/$(PROJECT).elf -O ihex build/$(PROJECT).hex
	@cat build/$(PROJECT).hex | sed '/^:00000001FF/d' > build/$(PROJECT)a.hex
	@riscv-none-embed-objcopy buildrv/$(PROJECT).elf -O ihex buildrv/$(PROJECT).hex
	@cat build/$(PROJECT)a.hex buildrv/$(PROJECT).hex > build/$(PROJECT)-combined.hex
	@arm-none-eabi-objcopy -I ihex build/$(PROJECT)-combined.hex -O elf32-littlearm build/$(PROJECT)-combined.elf

arm:
	@$(MAKE) -f Makefile.ARM

build/$(PROJECT).elf: arm

buildrv/common_riscv.ld: build/$(PROJECT).elf arm
	@mkdir -p buildrv
	@grep "__FlashStart_" build/$(PROJECT).map | sed 's/__FlashStart_ = .*$$//' | sed 's/^ */__FlashStart = /' | sed 's/ *$$/;/' > $@
	@echo "__FlashLength = (0x10080000 - __FlashStart);" >> $@

riscv: buildrv/common_riscv.ld arm build/$(PROJECT).elf
	@$(MAKE) -f Makefile.RISCV

buildrv/$(PROJECT).elf: riscv arm build/$(PROJECT).elf buildrv/common_riscv.ld

clean libclean distclean:
	@$(MAKE) -f Makefile.ARM $@
	@$(MAKE) -f Makefile.RISCV $@

release: all
	@$(MAKE) -f Makefile.ARM $@
	@$(MAKE) -f Makefile.RISCV $@
	@arm-none-eabi-objcopy -I elf32-littlearm build/$(PROJECT)-combined.elf -O binary build/$(PROJECT)-combined.bin
