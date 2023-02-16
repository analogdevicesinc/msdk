#!/usr/bin/env bash

echo "###################################################################################################"
echo "# build_flash.sh <msdk_path> <openocd> <target_type> <board_type> <project> <port> <build> <flash>#"
echo "###################################################################################################"
echo

# Examples
#   /home/btm-ci/Workspace/yc/msdk_open /home/btm-ci/Tools/openocd MAX32690 EvKit_V1 BLE5_ctr 0409170211cd0dd400000000000000000000000097969906 False False
echo $0 $@
echo

MSDK=$1
OPENOCD=$2
TARGET=$3
echo TARGET: ${TARGET}
TARGET_LC=`echo $TARGET | tr '[:upper:]' '[:lower:]'`
echo TARGET_LC: ${TARGET_LC}
BOARD_TYPE=$4
echo BOARD_TYPE: ${BOARD_TYPE}
PROJECT=$5
PORT=$6
BUILD=$7
FLASH=$8

#--------------------------------------------------------------------------------------------------
# build the project
function build()
{
    echo "-----------------------------------------------------------------------------------------"
    echo "Build the project."
    echo

    cd $MSDK/Examples/$TARGET/$PROJECT
    echo PWD=`pwd`
    echo 

    set -e
    set -x

    make MAXIM_PATH=$MSDK distclean
    make -j8 MAXIM_PATH=$MSDK TARGET=$TARGET BOARD=$BOARD_TYPE

    set +x
    set +e
}

#--------------------------------------------------------------------------------------------------
function flash_boards()
{
    echo "-----------------------------------------------------------------------------------------"
    echo "Flash the ELF to the board."
    echo

    cd $MSDK/Examples/$TARGET/$PROJECT
    echo PWD=`pwd`
    echo

    flash_with_openocd ${TARGET_LC}.elf $PORT
}

# -------------------------------------------------------------------------------------------------
# Function accepts parameters: filename, CMSIS_DAP_ID_x 
function flash_with_openocd()
{
    echo "Board DAP: $PORT"
    cd $MSDK/Examples/$TARGET/$PROJECT/build
    echo PWD=`pwd`
    echo
    
    set -x
    
    # mass erase and flash
    $OPENOCD/src/openocd -f $OPENOCD/tcl/interface/cmsis-dap.cfg -f $OPENOCD/tcl/target/$TARGET_LC.cfg -s $OPENOCD/tcl -c "adapter serial $PORT" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt;max32xxx mass_erase 0" -c "program ${TARGET_LC}.elf verify reset exit" > /dev/null &
    openocd_dapLink_pid=$!

    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid &> /dev/null; do
        sleep 1
    
        # we can add a timeout here if we want
    done

    set +x
    
    # Attempt to verify the image, prevent exit on error
    $OPENOCD/src/openocd -f $OPENOCD/tcl/interface/cmsis-dap.cfg -f $OPENOCD/tcl/target/$TARGET_LC.cfg -s $OPENOCD/tcl -c "adapter serial $PORT" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash verify_image $1; reset; exit"

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]; then
        # Reprogram the device if the verify failed
        $OPENOCD/src/openocd -f $OPENOCD/tcl/interface/cmsis-dap.cfg -f $OPENOCD/tcl/target/$TARGET_LC.cfg -s $OPENOCD/tcl -c "adapter serial $PORT" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt;max32xxx mass_erase 0" -c "program $1 verify reset exit" > /dev/null &
        openocd_dapLink_pid=$!
    fi
}

# -------------------------------------------------------------------------------------------------
# Main function
function main()
{
    if [ "${BUILD}" == "True" ]; then
        build
    else
        echo "Skip build."
    fi
    
    if [ "${FLASH}" == "True" ]; then
        flash_boards
    else
        echo "Skip flash."
    fi
}

# -------------------------------------------------------------------------------------------------
main

echo "---------------------------------------------------------------------------------------------"
echo "DONE!"
echo "---------------------------------------------------------------------------------------------"
