#!/bin/bash

# Target under test
if [ `hostname` == "wall-e" ]
then
    export CMSIS_DAP_ID_1=04091702d4f18ac600000000000000000000000097969906
    export TARGET_1_LC=max32655
    export TARGET_1_UC=MAX32655
    export devSerial_1=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D309ZDFB' | rev | cut -b 1-7 | rev)"
    export TARGET_1_CFG=${TARGET_1_LC}.cfg
    # Helper device for connected tests
    export CMSIS_DAP_ID_2=04091702f7f18a2900000000000000000000000097969906
    export devSerial_2=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D3073ICQ' | rev | cut -b 1-7 | rev)"
    export TARGET_2_LC=max32655
    export TARGET_2_UC=MAX32655
    export TARGET_2_CFG=${TARGET_2_LC}.cfg
else  # ying-cai
    export CMSIS_DAP_ID_1=0409000069c5c14600000000000000000000000097969906
    export TARGET_1_LC=max32655
    export TARGET_1_UC=MAX32655
    export devSerial_1=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D3073IDG' | rev | cut -b 1-7 | rev)"
    export TARGET_1_CFG=${TARGET_1_LC}.cfg
    # Helper device for connected tests
    export CMSIS_DAP_ID_2=040900006bd8439a00000000000000000000000097969906
    export devSerial_2=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D309ZDE9' | rev | cut -b 1-7 | rev)"
    export TARGET_2_LC=max32655
    export TARGET_2_UC=MAX32655
    export TARGET_2_CFG=${TARGET_2_LC}.cfg
fi

echo --- devSerial_1=$devSerial_1
echo --- devSerial_2=$devSerial_2

export TEST_BOARD=EvKit_V1
#export EXAMPLE_TEST_PATH=$(pwd)
#cd ../
export MSDK_DIR=$(pwd)
export EXAMPLE_TEST_PATH=/home/$USER/Workspace/BLE-examples-test/Examples_tests
export VERBOSE_TEST=1
export failedTestList=" "

export OPENOCD_TCL_PATH=/home/$USER/Tools/openocd/tcl
export OPENOCD=/home/$USER/Tools/openocd/src/openocd
export ROBOT=/home/$USER/.local/bin/robot

#------------------------------------------------------------------------------
function script_clean_up() {
    # check if some runoff opeocd instance is running
    set +e
    if ps -p $openocd_dapLink_pid >/dev/null; then
        kill -9 $openocd_dapLink_pid || true
    fi

    set -e
}

#------------------------------------------------------------------------------
# Function accepts parameters: filename , CMSIS_DAP_ID_x
function flash_with_openocd() {
    # mass erase and flash
    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1 verify reset exit" >/dev/null &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid; do
        sleep 1
        # we can add a timeout here if we want
    done
    set -e
    # Attempt to verify the image, prevent exit on error
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; flash verify_image $1; reset; exit"

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]; then
        # Reprogram the device if the verify failed
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1 verify reset exit" >/dev/null &
        openocd_dapLink_pid=$!
    fi
}
# This function accepts a CMSIS device ID as a parameter
function erase_with_openocd() {
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_2_CFG -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $1" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; flash erase_address 0x10004000 0x40000; reset exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    wait $openocd_dapLink_pid
}

function run_notConntectedTest() {
    project_marker
    cd $PROJECT_NAME
    set +x
    echo "--- Flashing $PROJECT_NAME"
    # make -j8 projects are build in validation build step
    cd build/
    flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1
    #place to store robotframework results
    mkdir $EXAMPLE_TEST_PATH/results/$PROJECT_NAME
    cd $EXAMPLE_TEST_PATH/tests
    echo --- pwd=`pwd`
    # do not let a single failed test stop the testing of the rest
    set +e
    #runs desired test
    echo --- $ROBOT -d $EXAMPLE_TEST_PATH/results/$PROJECT_NAME/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT_1:$devSerial_1 $PROJECT_NAME.robot
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$PROJECT_NAME/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT_1:$devSerial_1 $PROJECT_NAME.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| $PROJECT_NAME "
    fi
    set -e
    #get back to target directory
    cd $MSDK_DIR/Examples/$TARGET_1_UC
}

function project_marker() {
    echo -e "\n--- $PROJECT_NAME Flashing Procedure"
}

#------------------------------------------------------------------------------
trap script_clean_up EXIT SIGINT

# build BLE examples
echo -e "\n--- MSDK_DIR=$MSDK_DIR"

# Flash BLE_otac --------------------------------------------------------------
export PROJECT_NAME=BLE_otac
cd $MSDK_DIR/Examples/$TARGET_1_UC/$PROJECT_NAME
dir=`pwd`
#make -C ${dir} clean
#make -C ${dir} libclean
#make -C ${dir} -j8

cd build
echo --- pwd=`pwd`

project_marker
set +x
echo "--- Flashing $PROJECT_NAME"
flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1
echo -e "\n--- Finished flashing."

# Flash BLE_otas --------------------------------------------------------------
export PROJECT_NAME=BLE_otas
cd $MSDK_DIR/Examples/$TARGET_2_UC/$PROJECT_NAME
dir=`pwd`
#make -C ${dir} clean
#make -C ${dir} libclean
#make -C ${dir} -j8

cd build
echo --- pwd=`pwd`

project_marker
set +x
echo "--- Flashing $PROJECT_NAME"
flash_with_openocd $TARGET_2_LC.elf $CMSIS_DAP_ID_2
echo -e "\n--- Finished flashing."

echo -e "\n--- Done."

