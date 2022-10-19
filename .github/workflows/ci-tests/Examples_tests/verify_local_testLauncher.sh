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

function script_clean_up() {
    # check if some runoff opeocd instance is running
    set +e
    if ps -p $openocd_dapLink_pid >/dev/null; then
        kill -9 $openocd_dapLink_pid || true
    fi

    set -e
}
trap script_clean_up EXIT SIGINT

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

#--------------------------------------------------------------------------------------------

# build BLE examples
echo -e "\n--- MSDK_DIR=$MSDK_DIR"
cd $MSDK_DIR/Examples
SUBDIRS=$(find . -type d -name "BLE*")
echo -e "\n--- SUBDIRS=$SUBDIRS\n"
for dir in ${SUBDIRS}; do
    echo "---"
    echo " Validation build for ${dir}"
    echo pwd=`pwd`
    echo "---"
    #make -C ${dir} clean
    #make -C ${dir} libclean
    #make -C ${dir} -j8
done

echo -e "\n--- Finished building BLE examples"
#remove old robtoframework logs

cd $EXAMPLE_TEST_PATH/results/
echo -e "\n--- pwd=`pwd`"
rm -r *

#keep track of directory count, might need it
projIdx=1
numOfFailedTests=0
#no filter would support ALL projects not just ble
project_filter='BLE_'

# testing ME17 only for now
cd $MSDK_DIR/Examples/$TARGET_1_UC

echo -e "\n--- erase the helper device so that its not running an app that might connect to current app under test"
erase_with_openocd $CMSIS_DAP_ID_2
cd $MSDK_DIR/Examples/$TARGET_1_UC

echo -e "\n--- tests projects by robot framework"
for dir in ./*/; do
    if [[ "---$dir---" == *"$project_filter"* ]]; then
        export PROJECT_NAME=$(echo "$dir" | tr -d /.)
	    echo -e "\n--- PROJECT_NAME=$PROJECT_NAME"

        case $PROJECT_NAME in

        "BLE_datc")
            run_notConntectedTest
            ;;

        "BLE_dats")
            run_notConntectedTest
            ;;

        "BLE_mcs")
            run_notConntectedTest
            ;;

        "BLE_fit")
            run_notConntectedTest
            ;;

        "BLE_fcc")

            #Nothing to do here, no test for fcc
            #project is built up top and that will detect a failure if any

            ;;

        "BLE_FreeRTOS")
            run_notConntectedTest
            ;;

        "BLE_otac")
            run_notConntectedTest
            ;;

        "BLE_otas")
            # gets tested during conencted test below

            ;;

        "BLE_periph")
            # No buttons implemented for this example so lets just make sure it builds, so we can ship it
            # cd $PROJECT_NAME
            # make -j8
            # let "numOfFailedTests+=$?"
            # cd $MSDK_DIR/Examples/$TARGET_UC
            ;;

        *) ;;

        esac

    fi

    let projIdx++
done

echo -e "\n--- Finished robot framework tests for each BLE examples. projIdx=$projIdx"

echo -e "\n━━━━━━━━━━━━━━━━━━━━━━━| BLE_datc/dats Connection Test |━━━━━━━━━━━━━━━━━━━━━━━\n"
#------ datc ME17
cd $MSDK_DIR/Examples/$TARGET_1_UC/BLE_datc/build
echo -e "\n--- Flashing BLE_datc"
echo pwd=`pwd`
ls *.elf

flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1

#------ dats on ME18
#cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_dats
#make -j8
cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_dats/build
echo -e "\n--- Flashing BLE_dats"
echo pwd=`pwd`
ls *.elf

flash_with_openocd $TARGET_2_LC.elf $CMSIS_DAP_ID_2

#give them time to connect
sleep 5 #give them time to connect
# directory for resuilts logs
cd $EXAMPLE_TEST_PATH/results
mkdir BLE_dat_cs
cd $EXAMPLE_TEST_PATH/tests
#runs desired test
set +e
$ROBOT -d $EXAMPLE_TEST_PATH/results/BLE_dat_cs/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT_1:$devSerial_1 -v SERIAL_PORT_2:$devSerial_2 BLE_dat_cs.robot
let "testResult=$?"
if [ "$testResult" -ne "0" ]; then
    # update failed test count
    let "numOfFailedTests+=$testResult"
    failedTestList+="| BLE_dat_cs "
fi
set -e

echo -e "\n--- Flash BLE_otac onto Device 1  : ME17"
#make -j8
cd $MSDK_DIR/Examples/$TARGET_1_UC/BLE_otac/build
echo -e "\n--- Flashing BLE_otac"
flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1

echo -e "\n--- Flash BLE_otas  onto Device 2  : ME18"
cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_otas/build
flash_with_openocd $TARGET_2_LC.elf $CMSIS_DAP_ID_2
#------ Build & Flash Bootloader onto Device 2  : ME18
cd $MSDK_DIR/Examples/$TARGET_2_UC/Bootloader
#make -j8
cd $MSDK_DIR/Examples/$TARGET_2_UC/Bootloader/build
echo -e "\n--- Flashing Bootloader"
#not using the flash_with_openocd function here because that causes the application code to be erased and only
#bootloader to remain
set +e
$OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt" -c "program max32655.elf verify reset exit" >/dev/null &
openocd_dapLink_pid=$!
# wait for openocd to finish
while kill -0 $openocd_dapLink_pid; do
    sleep 1
    # we can add a timeout here if we want
done
set -e
# Attempt to verify the image, prevent exit on error
$OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; flash verify_image max32655.elf; reset; exit"

# Check the return value to see if we received an error
if [ "$?" -ne "0" ]; then
    # Reprogram the device if the verify failed
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt" -c "program max32655.elf verify reset exit" >/dev/null &
    openocd_dapLink_pid=$!
fi

# Give client/server time to establish connection
sleep 15
# directory for resuilts logs
cd $EXAMPLE_TEST_PATH/results
mkdir BLE_ota_cs
cd $EXAMPLE_TEST_PATH/tests
set +e
#runs desired test
$ROBOT -d $EXAMPLE_TEST_PATH/results/BLE_ota_cs/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT_1:$devSerial_1 BLE_ota_cs.robot
let "testResult=$?"
if [ "$testResult" -ne "0" ]; then
    # update failed test count
    let "numOfFailedTests+=$testResult"
    failedTestList+="| BLE_ota_cs "
fi
set -e
echo "=============================================================================="
echo "=============================================================================="
if [ "$numOfFailedTests" -ne "0" ]; then
    echo "Test completed with $numOfFailedTests failed tests located in: "
    echo " $failedTestList | "
else
    echo "Relax! ALL TESTS PASSED"
fi
echo "=============================================================================="
echo "=============================================================================="
exit $numOfFailedTests

echo -e "\n--- Done."

