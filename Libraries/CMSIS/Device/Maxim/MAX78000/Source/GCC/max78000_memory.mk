###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
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

# Allocate the ARM core more space (4kB) for mailbox.
# 	32kB (SRAM0) + 32kB (SRAM1) + 4kB (~SRAM2)
ARM_SRAM_SIZE   ?= 0x11000

# Allocate the RISCV core space from SRAM2 but account for
#	mailbox space that was allocated for the ARM core.
#	48kB (SRAM2) - 4kB (ARM mailbox)
RISCV_SRAM_SIZE ?= 0x0B000

# NOTE: SRAM3 (16kB) is reserved for RISCV cache, ICC1 if RISC-V core in use.
#	32kB (SRAM0) + 32kB (SRAM1) + 48kB (SRAM2)
DUAL_CORE_SRAM_SIZE ?= 0x1C000

# Add memory size definitions for startup and linker files.
PROJ_AFLAGS += -DFLASH_ORIGIN=0x10000000
PROJ_AFLAGS += -DFLASH_SIZE=0x80000 # 512kB FLASH
PROJ_AFLAGS += -DSRAM_ORIGIN=0x20000000

# Each core has a mailbox in the RISCV core SRAM memory space.
# Total memory allocated for boxes will be 2*MAILBOX_SIZE
# Will work with small MAILBOX_SIZE, cores will experience more interrupts
# and higher communication latency.
# Minimum value is 16
MAILBOX_SIZE = 64

ifneq ($(RISCV_CORE),)
# RISCV core
# Set the memory sizes for the RISCV core
# ARM uses lower address space, RISCV uses upper
# ARM application must use compatible memory sizes as to not overrun
PROJ_AFLAGS+=-DRISCV_SRAM_SIZE=$(RISCV_SRAM_SIZE)
PROJ_AFLAGS+=-DRISCV_FLASH_SIZE=0x30000
PROJ_AFLAGS+=-DSRAM_SIZE=$(DUAL_CORE_SRAM_SIZE)  # Save last SRAM instance for RISCV cache, ICC1
$(info ---INFO RISCV core, SRAM_SIZE=$(DUAL_CORE_SRAM_SIZE), )
else

ifneq ($(RISCV_LOAD),)
# ARM Core loading RISCV
PROJ_AFLAGS+=-DSRAM_SIZE=$(DUAL_CORE_SRAM_SIZE)  # Save last SRAM instance for RISCV cache, ICC1
$(info ---INFO ARM loading RISCV, SRAM_SIZE=$(DUAL_CORE_SRAM_SIZE), ARM_SRAM_SIZE=$(ARM_SRAM_SIZE))

# Set ARM memory sizes, RISCV will use remainder and will be defined in linker file.
PROJ_AFLAGS+=-DARM_SRAM_SIZE=$(ARM_SRAM_SIZE)
PROJ_AFLAGS+=-DARM_FLASH_SIZE=0x50000
else
# Just ARM core
PROJ_AFLAGS+=-DSRAM_SIZE=0x20000  # Full size SRAM
MAILBOX_SIZE = 0 # No need for mailboxes if we're only using ARM core
$(info ---INFO ARM ONLY, SRAM_SIZE=0x20000)
endif
endif

PROJ_AFLAGS += -DMAILBOX_SIZE=$(MAILBOX_SIZE)
PROJ_CFLAGS += -DMAILBOX_SIZE=$(MAILBOX_SIZE)

$(info ---INFO exit  Libraries/CMSIS/Device/Maxim/MAX78000/Source/GCC/max78000_memory.mk)
