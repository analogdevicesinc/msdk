# Add memory size definitions
PROJ_AFLAGS += -DFLASH_ORIGIN=0x10000000
PROJ_AFLAGS += -DFLASH_SIZE=0x340000
PROJ_AFLAGS += -DSRAM_ORIGIN=0x20000000
PROJ_AFLAGS += -DSRAM_SIZE=0x120000

# Each core has a mailbox in the RISCV core SRAM memory space.
# Total memory allocated for boxes will be 2*MAILBOX_SIZE
# Will work with small MAILBOX_SIZE, cores will experience more interrupts
# and higher communication latency. Not needed for single core applications
# Minimum value is 4
MAILBOX_SIZE = 64


# RISCV core
# Set the memory sizes for the RISCV core
# ARM uses lower address space, RISCV uses upper
# ARM application must use compatible memory sizes as to not overrun
RISCV_FLASH_ORIGIN=0x10300000
RISCV_FLASH_SIZE=0x40000
RISCV_SRAM_ORIGIN=0x20100000
RISCV_SRAM_SIZE=0x20000  # Save last SRAM instance for RISCV

DUAL_CORE = 0

ifneq ($(RISCV_CORE),)
DUAL_CORE = 1
endif

ifneq ($(RISCV_LOAD),)
DUAL_CORE = 1
endif

ifeq ($(DUAL_CORE),1)
PROJ_AFLAGS+=-DRISCV_FLASH_ORIGIN=$(RISCV_FLASH_ORIGIN)
PROJ_AFLAGS+=-DRISCV_FLASH_SIZE=$(RISCV_FLASH_SIZE)
PROJ_AFLAGS+=-DRISCV_SRAM_ORIGIN=$(RISCV_SRAM_ORIGIN)
PROJ_AFLAGS+=-DRISCV_SRAM_SIZE=$(RISCV_SRAM_SIZE)
PROJ_AFLAGS += -DMAILBOX_SIZE=$(MAILBOX_SIZE)
PROJ_CFLAGS += -DMAILBOX_SIZE=$(MAILBOX_SIZE)
endif
