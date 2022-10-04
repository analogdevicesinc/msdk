#!/bin/sh


# Uses openocd to verify or load and reset
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
./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/$1 -s tcl  \
    -c "adapter serial $2" -c "init; reset halt; flash verify_image $3; reset; exit"

# Check the return value to see if we received an error
if [ "$?" -ne "0" ]
then
  set -e

  # Reprogram the device if the verify failed
  ./src/openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/$1 -s tcl  \
      -c "adapter serial $2" -c "init; reset halt; program $3 verify reset exit"
fi

set -e

cd ${CURRENT_DIR}
