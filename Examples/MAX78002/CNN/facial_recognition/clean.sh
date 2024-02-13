#!/bin/sh
openocd -s "${TOOLCHAIN_PATH}/OpenOCD/scripts" -f interface/cmsis-dap.cfg -f target/max78002.cfg -c "flash init; init; halt; flash erase_sector 0 1 last; exit"
