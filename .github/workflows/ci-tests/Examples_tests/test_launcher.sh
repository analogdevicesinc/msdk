#!/bin/bash

# main ME17 device used to test the rest
MAIN_DEVICE_NAME_UPPER=MAX32655
MAIN_DEVICE_NAME_LOWER=max32655
MAIN_DEVICE_ID=04091702d4f18ac600000000000000000000000097969906
MAIN_DEVICE_SERIAL_PORT=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D309ZDFB' | rev | cut -d "/" -f1 | rev)"

# List of devices under test
#dut_list=(max32665)
dut_list=(max32655 max32665)
# List of serial IDs for DUT, must correlate with list above
#dut_list_ID=(0409000098d9439b00000000000000000000000097969906)
dut_list_ID=(04091702f7f18a2900000000000000000000000097969906 0409000098d9439b00000000000000000000000097969906)
# List of serail devices associated with each DUT msut correlate with device list above
#dut_list_serial=(D30A1X9X)
dut_list_serial=(D3073ICQ D30A1X9X)
# Will hold values of the current device undertest from the lists above
DUT_NAME_UPPER=NONE
DUT_NAME_LOWER=NONE
DUT_ID=0
DUT_SERIAL_PORT=0

EXAMPLE_TEST_PATH=$(pwd)
cd ../../../../
MSDK_DIR=$(pwd)
failedTestList=" "
numOfFailedTests=0

#****************************************** Change this when testing locally **************************

# WALL-E
export OPENOCD_TCL_PATH=/home/btm-ci/Tools/openocd/tcl
export OPENOCD=/home/btm-ci/Tools/openocd/src/openocd
export ROBOT=/home/btm-ci/.local/bin/robot

# Local
# export OPENOCD_TCL_PATH=/home/eddie/workspace/openocd/tcl
# export OPENOCD=/home/eddie/workspace/openocd/src/openocd
# export ROBOT=/home/eddie/.local/bin/robot

#***************************************** Helper functions *****************************************
#****************************************************************************************************
function script_clean_up() {
    # check if some runoff opeocd instance is running
    set +e
    if ps -p $openocd_dapLink_pid >/dev/null; then
        kill -9 $openocd_dapLink_pid || true
    fi

    set -e
}
trap script_clean_up EXIT SIGINT

# Function accepts parameters: device, CMSIS_DAP_ID_x
function flash_with_openocd() {
    # mass erase and flash
    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1.elf verify reset exit" >/dev/null &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid; do
        sleep 1
        # we can add a timeout here if we want
    done
    set -e
    # Attempt to verify the image, prevent exit on error
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; flash verify_image $1.elf; reset; exit"

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]; then
        # Reprogram the device if the verify failed
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1.elf verify reset exit" >/dev/null &
        openocd_dapLink_pid=$!
    fi
}
# Function accepts parameters:device , CMSIS_DAP_ID_x
function erase_with_openocd() {
    printf "> Erasing $1 : $2 \r\n"
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; max32xxx mass_erase 0;" -c " exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    wait $openocd_dapLink_pid

    # erase second bank of larger chips
    if [[ $1 == "max32665" ]]; then
        printf "> Erasing second bank of device: $1 with ID:$2 \r\n"
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; max32xxx mass_erase 1;" -c " exit" &
        openocd_dapLink_pid=$!
        # wait for openocd to finish
        wait $openocd_dapLink_pid
    fi

}

function run_notConntectedTest() {
    project_marker
    cd $PROJECT_NAME
    set +x
    echo "> Flashing $DUT_NAME_UPPER $PROJECT_NAME"
    # make -j8 projects are build in validation build step
    cd build/
    flash_with_openocd $DUT_NAME_LOWER $DUT_ID
    #place to store robotframework results

    cd $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER
    mkdir $PROJECT_NAME

    cd $EXAMPLE_TEST_PATH/tests
    # do not let a single failed test stop the testing of the rest
    set +e
    #runs desired test
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/$PROJECT_NAME -v SERIAL_PORT_1:$DUT_SERIAL_PORT $PROJECT_NAME.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| $PROJECT_NAME ($DUT_NAME_UPPER) "
    fi
    set -e

    # get back to target directory
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER
}

function flash_bootloader() {
    #------ -----------Build & Flash Bootloader onto Device 2  : ME17
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/Bootloader
    make -j8
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/Bootloader/build
    printf "> Flashing Bootloader on DUT\r\n\r\n"
    #not using the flash_with_openocd function here because that causes the application code to be erased and only
    #bootloader to remain
    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$DUT_NAME_LOWER.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $DUT_ID" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt" -c "program $DUT_NAME_LOWER.elf verify reset exit" >/dev/null &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid; do
        sleep 1
        # we can add a timeout here if we want
    done
    set -e
    # Attempt to verify the image, prevent exit on error
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$DUT_NAME_LOWER.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $DUT_ID" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; flash verify_image $DUT_NAME_LOWER.elf; reset; exit"

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]; then
        # Reprogram the device if the verify failed
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$DUT_NAME_LOWER.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $DUT_ID" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt" -c "program $DUT_NAME_LOWER.elf verify reset exit" >/dev/null &
        openocd_dapLink_pid=$!
    fi
}

function erase_all_devices() {
    # erase main device
    erase_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID
    # erase DUTs
    for device in ${!dut_list[@]}; do
        erase_with_openocd ${dut_list[device]} ${dut_list_ID[device]}
    done
}
function project_marker() {
    echo "=============================================================================="
    printf "> Start of testing $DUT_NAME_LOWER ($DUT_NAME_UPPER) \r\n> ID:$DUT_ID \r\n> Port:$DUT_SERIAL_PORT \r\n\r\n"
}

#***************************************** Start of test script *************************************
#****************************************************************************************************

# remove old robtoframework logs
cd $EXAMPLE_TEST_PATH/results/
rm -rf *

erase_all_devices

for device in ${!dut_list[@]}; do
    # translate device name to uppercase
    DUT_NAME_UPPER=$(echo ${dut_list[device]} | tr '[:lower:]' '[:upper:]')
    # change advertising name for projects under test to avoid
    # connections with office devices
    printf "> changing advertising names : $MSDK_DIR/Examples/$DUT_NAME_UPPER\r\n\r\n"
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats
    perl -i -pe "s/\'D\'/\'C\'/g" dats_main.c

    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_datc
    perl -i -pe "s/\'D\'/\'C\'/g" datc_main.c

    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otac
    perl -i -pe "s/\'S\'/\'P\'/g" datc_main.c

    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    perl -i -pe "s/\'S\'/\'P\'/g" dats_main.c

    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_FreeRTOS
    perl -i -pe "s/\'S\'/\'P\'/g" dats_main.c
done

# build BLE examples
cd $MSDK_DIR/Examples
SUBDIRS=$(find . -type d -name "BLE*")
for dir in ${SUBDIRS}; do
    echo "---------------------------------------"
    echo " Validation build for ${dir}"
    echo "---------------------------------------"
    make -C ${dir} clean
    make -C ${dir} libclean
    make -C ${dir} -j8
done

# itterate through devices and conduct standalone test followed by connected tests
for i in ${!dut_list[@]}; do

    # setup  all DUT varaibles
    DUT_NAME_LOWER=${dut_list[i]}
    DUT_NAME_UPPER=$(echo ${dut_list[i]} | tr '[:lower:]' '[:upper:]')
    DUT_SERIAL_PORT=/dev/$(ls -la /dev/serial/by-id | grep -n ${dut_list_serial[i]} | rev | cut -d "/" -f1 | rev)
    DUT_ID=${dut_list_ID[i]}

    # directory for test resutls
    mkdir $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER
    # enter current device under test directory
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER

    #no filter would support ALL projects not just ble
    project_filter='BLE_'

    #----------------------------- Run non connected tests
    for dir in ./*/; do
        if [[ "$dir" == *"$project_filter"* ]]; then

            export PROJECT_NAME=$(echo "$dir" | tr -d /.)
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

                ;;

            *) ;;

            esac

        fi

        let projIdx++

    done # end non connected tests

    #--------------------------start Datc/Dats conencted tests

    # Flash MAIN_DEVICE with BLE_datc
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_datc/build
    printf "> Flashing BLE_datc on main device: $MAIN_DEVICE_NAME_UPPER\r\n "
    flash_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID

    # flash DUT with BLE_dats
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats/build
    echo "> Flashing BLE_dats on DUT $DUT_NAME_UPP"
    flash_with_openocd $DUT_NAME_LOWER $DUT_ID

    # Run robot test
    # give them time to connect
    sleep 5
    # directory for resuilts logs
    cd $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/
    mkdir BLE_dat_cs
    cd $EXAMPLE_TEST_PATH/tests
    # runs desired test
    set +e
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_dat_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_dat_cs.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| BLE_dat_cs ($DUT_NAME_UPPER) "
    fi
    set -e

    # make sure to erase main device and current DUT to it does not store bonding info
    erase_with_openocd $DUT_NAME_LOWER $DUT_ID
    erase_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID

    #--------------------------start Otac/Otas conencted tests

    # directory for resuilts logs
    cd $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/
    mkdir BLE_ota_cs
    cd $EXAMPLE_TEST_PATH/tests

    # flash Firmware V1 BLE_otas  onto DUT
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas/build
    printf "> Flashing BLE_otas on DUT $DUT_NAME_UPP\r\n\r\n"
    flash_with_openocd $DUT_NAME_LOWER $DUT_ID

    flash_bootloader

    # change firmware version and rebuild
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    # change firmware version to verify otas worked
    perl -i -pe "s/FW_VERSION 1/FW_VERSION 2/g" wdxs_file.c
    make -j8

    # since  MAIN_DEVICE and DUT are not the same chip we need to make sure the
    # FW update binary is built for the DUT's mcu and not the  MAIN_DEVICE mcu
    # modify project.mk's recursive 'MAKE' call that builds the FW update bin
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac

    #appends TARGET , TARGET_UC and TARGET_LC to the make commands and sets them to $DUT_NAME_UPPER and $DUT_NAME_LOWER
    sed -i 's/BUILD_DIR=\$(FW_BUILD_DIR) PROJECT=fw_update/BUILD_DIR=\$(FW_BUILD_DIR) PROJECT=fw_update TARGET='"$DUT_NAME_UPPER"' TARGET_UC='"$DUT_NAME_UPPER"' TARGET_LC='"$DUT_NAME_LOWER"'/g' project.mk
    sed -i 's/BUILD_DIR=\$(FW_BUILD_DIR) \$(FW_UPDATE_BIN)/BUILD_DIR=\$(FW_BUILD_DIR) \$(FW_UPDATE_BIN) TARGET='"$DUT_NAME_UPPER"' TARGET_UC='"$DUT_NAME_UPPER"' TARGET_LC='"$DUT_NAME_LOWER"'/g' project.mk
    # flash MAIN_DEVICE with BLE_OTAC, it will use the OTAS bin with new firmware
    make clean
    make FW_UPDATE_DIR=../../$DUT_NAME_UPPER/BLE_otas -j8


    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac/build
    printf "> Flashing BLE_otac on main device: $MAIN_DEVICE_NAME_UPPER\r\n "
    flash_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID
    printf "Flashing done"

    #revert files back, took me days to find this bug
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
    sed -i 's/TARGET='"$DUT_NAME_UPPER"'//' project.mk
    sed -i 's/TARGET_UC='"$DUT_NAME_UPPER"'//' project.mk
    sed -i 's/TARGET_LC='"$DUT_NAME_LOWER"'//' project.mk
    # give time to connect
    sleep 15

    set +e
    # runs desired test
    cd $EXAMPLE_TEST_PATH/tests
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_ota_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_ota_cs.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| BLE_ota_cs ($DUT_NAME_UPPER) "
    fi
    set -e

    # make sure to erase main device and current DUT to it does not store bonding info
    erase_with_openocd $DUT_NAME_LOWER $DUT_ID
    erase_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID

done

erase_all_devices

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
