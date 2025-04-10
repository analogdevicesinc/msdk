###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################

ifeq "$(PROJECT)" ""
PROJECT=max32655
RV_PROJECT=max32655-riscv
else
RV_PROJECT=$(PROJECT)
endif

.PHONY: all arm release clean distclean riscv
all: build/$(PROJECT)-combined.elf

build/$(PROJECT)-combined.elf: build/$(PROJECT).elf buildrv/$(PROJECT).elf arm riscv
	@arm-none-eabi-objcopy -I elf32-littlearm build/$(PROJECT).elf -O ihex build/$(PROJECT).hex
	@cat build/$(PROJECT).hex | sed '/^:00000001FF/d' > build/$(PROJECT)a.hex
	@riscv-none-elf-objcopy buildrv/$(RV_PROJECT).elf -O ihex buildrv/$(PROJECT).hex
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
