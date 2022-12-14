$(info ---INFO enter Libraries/CMSIS/Device/Maxim/MAX32655/Source/GCC/max32655_memory.mk, RISCV_CORE=$(RISCV_CORE), RISCV_LOAD=$(RISCV_LOAD))

#ARM_SRAM_SIZE  ?= 0x10000		# 64
#RISCV_RAM_SIZE ?= 0x0C000		# 48

ARM_SRAM_SIZE  ?= 0x11000		# 64 + 4, allocate the ARM core more space
RISCV_RAM_SIZE ?= 0x0B000		# 48 - 4

# NOTE: sysram3 16 KB is reserved for RISCV cache, ICC1 if RISC-V core in use
DUAL_CORE_RAM_SIZE ?= 0x1C000   # 64 + 48

# Add memory size definitions
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
PROJ_AFLAGS+=-DRISCV_SRAM_SIZE=$(RISCV_RAM_SIZE)
PROJ_AFLAGS+=-DRISCV_FLASH_SIZE=0x30000
PROJ_AFLAGS+=-DSRAM_SIZE=$(DUAL_CORE_RAM_SIZE)  # Save last SRAM instance for RISCV cache, ICC1
$(info ---INFO RISCV core, SRAM_SIZE=$(DUAL_CORE_RAM_SIZE), )
else

ifneq ($(RISCV_LOAD),)
# ARM Core loading RISCV
PROJ_AFLAGS+=-DSRAM_SIZE=$(DUAL_CORE_RAM_SIZE)  # Save last SRAM instance for RISCV cache, ICC1
$(info ---INFO ARM loading RISCV, SRAM_SIZE=$(DUAL_CORE_RAM_SIZE), ARM_SRAM_SIZE=$(ARM_SRAM_SIZE))

# Set ARM memory sizes, RISCV will use remainder
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

$(info ---INFO exit  Libraries/CMSIS/Device/Maxim/MAX32655/Source/GCC/max32655_memory.mk)
