#!/bin/sh
openocd -s "${TOOLCHAIN_PATH}/OpenOCD/scripts" -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/max78000.elf 0x0 verify reset exit"
