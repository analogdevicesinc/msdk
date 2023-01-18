#!/bin/sh

# Uses openocd to hard reset the board
# Uses the following arguments
# 
# TARGET_CFG (just the file name, no path eg. max32665.cfg)
# CMSIS_DAP_ID (eg. 040900008bdf432e00000000000000000000000097969906)
# APPLICATION_FILE (abs path, eg. ~/Downloads/max32665.cfg)

CURRENT_DIR=$(realpath .)

# Install test application
cd ~/Tools/openocd

# Attempt to verify the image, prevent exit on error
set +e

./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/$1 -s tcl \
    -c "adapter serial $2" -c "init; reset; exit"

set -e

cd ${CURRENT_DIR}
