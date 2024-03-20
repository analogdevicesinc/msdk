#!/bin/sh
openocd -s "${TOOLCHAIN_PATH}/OpenOCD/scripts" -f interface/cmsis-dap.cfg -f target/max78002.cfg -c "flash init; init; halt; flash erase_sector 0 1 last; exit"
openocd -s "${TOOLCHAIN_PATH}/OpenOCD/scripts" -f interface/cmsis-dap.cfg -f target/max78002.cfg -c "program build/max78002.elf 0x0 verify reset exit"
