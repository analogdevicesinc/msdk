# Add memory size definitions
PROJ_AFLAGS += -DFLASH_ORIGIN=0x10000000
PROJ_AFLAGS += -DFLASH_SIZE=0x80000 # 512kB FLASH
PROJ_AFLAGS += -DSRAM_ORIGIN=0x20000000

# Each core has a mailbox in the RISCV core SRAM memory space.
# Total memory allocated for boxes will be 2*MAILBOX_SIZE
# Will work with small MAILBOX_SIZE, cores will experience more interrupts
# and higher communication latency.
# Minimum value is 4
MAILBOX_SIZE = 64
PROJ_AFLAGS += -DMAILBOX_SIZE=$(MAILBOX_SIZE)
PROJ_CFLAGS += -DMAILBOX_SIZE=$(MAILBOX_SIZE)

ifneq ($(RISCV_CORE),)
# RISCV core
# Set the memory sizes for the RISCV core
# ARM uses lower address space, RISCV uses upper
# ARM application must use compatible memory sizes as to not overrun
PROJ_AFLAGS+=-DRISCV_SRAM_SIZE=0xC000
PROJ_AFLAGS+=-DRISCV_FLASH_SIZE=0x30000
PROJ_AFLAGS+=-DSRAM_SIZE=0x1C000  # Save last SRAM instance for RISCV cache, ICC1

else

ifneq ($(RISCV_LOAD),)
# ARM Core loading RISCV
PROJ_AFLAGS+=-DSRAM_SIZE=0x1C000  # Save last SRAM instance for RISCV cache, ICC1

# Set ARM memory sizes, RISCV will use remainder
PROJ_AFLAGS+=-DARM_SRAM_SIZE=0x10000
PROJ_AFLAGS+=-DARM_FLASH_SIZE=0x50000
else
PROJ_AFLAGS+=-DSRAM_SIZE=0x20000  # Full size SRAM 
endif
endif
