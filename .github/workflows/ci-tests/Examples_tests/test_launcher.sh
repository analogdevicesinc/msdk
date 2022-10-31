#!/bin/bash

# main ME17 device used to test the rest
MAIN_DEVICE_NAME_UPPER=MAX32655
MAIN_DEVICE_NAME_LOWER=max32655
MAIN_DEVICE_ID=04090000bdf10c0400000000000000000000000097969906
MAIN_DEVICE_SERIAL_PORT=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D309ZDEM' | rev | cut -b 1-7 | rev)"

# List of devices under test
dut_list=(max32655 max32665 )
# List of serial IDs for DUT, must correlate with list above
dut_list_ID=(0409000098d9439b00000000000000000000000097969906 0409170246dfc09500000000000000000000000097969906 )
# List of serail devices associated with eeach DUT msut correlate with device list above
dut_list_serial=(D309ZDFO D30A1X9X )

# Will hold values of the current device undertest from the lists above
DUT_NAME_UPPER=NONE
DUT_NAME_LOWER=NONE
DUT_ID=0
DUT_SERIAL_PORT=0


export TEST_BOARD=EvKit_V1
export EXAMPLE_TEST_PATH=$(pwd)
cd ../../../../
export MSDK_DIR=$(pwd)
export VERBOSE_TEST=1
export failedTestList=" "
export numOfFailedTests=0

#****************************************** Change this when testing locally **************************

# WALL-E
# export OPENOCD_TCL_PATH=/home/btm-ci/Tools/openocd/tcl
# export OPENOCD=/home/btm-ci/Tools/openocd/src/openocd
# export ROBOT=/home/btm-ci/.local/bin/robot

# Local
export OPENOCD_TCL_PATH=/home/eddie/workspace/openocd/tcl
export OPENOCD=/home/eddie/workspace/openocd/src/openocd
export ROBOT=/home/eddie/.local/bin/robot

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
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; max32xxx mass_erase 0; reset exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    wait $openocd_dapLink_pid

    # erase second bank of larger chips
    if [[ $1 == "max32665" ]]; then
    printf "Erasing second bank of device: $1 with ID:$2"
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; max32xxx mass_erase 1; reset exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    wait $openocd_dapLink_pid
    fi

}

function run_notConntectedTest() {
    project_marker
    cd $PROJECT_NAME
    set +x
    echo "> Flashing $PROJECT_NAME"
    # make -j8 projects are build in validation build step
    cd build/
    flash_with_openocd $DUT_NAME_LOWER $DUT_ID
    #place to store robotframework results
    mkdir $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER
    cd $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER
    mkdir $PROJECT_NAME

    cd $EXAMPLE_TEST_PATH/tests
    # do not let a single failed test stop the testing of the rest
    set +e
    #runs desired test
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/$PROJECT_NAME -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT_1:$DUT_SERIAL_PORT $PROJECT_NAME.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| $PROJECT_NAME ($DUT_NAME_UPPER) "
    fi
    set -e
    #get back to target directory
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER
}

function project_marker() {
    echo "=============================================================================="
    printf "\r\n$DUT_NAME_UPPER $PROJECT_NAME Flashing Procedure \r\n\r\n"
}

#***************************************** Start of test script *************************************
#****************************************************************************************************
#remove old robtoframework logs
cd $EXAMPLE_TEST_PATH/results/
rm -rf *
#erase the helper devices so that they are not running an app that might connect
#to current app under test
for device  in ${!dut_list[@]}; do
    erase_with_openocd ${dut_list[device]} ${dut_list_ID[device]}
    # change advertising name for projects under test to avoid 
    # connections with office devices
    # translate device name to uppercase
    DUT_NAME_UPPER=$(echo ${dut_list[i]} | tr '[:lower:]' '[:upper:]')
    echo changing advertising names in $MSDK_DIR/Examples/$DUT_NAME_UPPER
    # cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats
    # perl -i -pe "s/\'D\'/\'C\'/g" dats_main.c

    # cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_datc
    # perl -i -pe "s/\'D\'/\'C\'/g" datc_main.c

    # cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otac
    # perl -i -pe "s/\'S\'/\'P\'/g" datc_main.c

    # cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    # perl -i -pe "s/\'S\'/\'P\'/g" dats_main.c

    # cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_FreeRTOS
    # perl -i -pe "s/\'S\'/\'P\'/g" dats_main.c
done

# build BLE examples
cd $MSDK_DIR/Examples
SUBDIRS=$(find . -type d -name "BLE*")
for dir in ${SUBDIRS}; do
    echo "---------------------------------------"
    echo " Validation build for ${dir}"
    echo "---------------------------------------"
    # make -C ${dir} clean
    # make -C ${dir} libclean
    # make -C ${dir} -j8
done

for i in ${!dut_list[@]}; do
    
    # setup  all DUT varaibles
    DUT_NAME_LOWER=${dut_list[i]}
    DUT_NAME_UPPER=$(echo ${dut_list[i]} | tr '[:lower:]' '[:upper:]')
    DUT_SERIAL_PORT=/dev/$(ls -la /dev/serial/by-id | grep -n ${dut_list_serial[i]} | rev | cut -b 1-7 | rev)
    DUT_ID=${dut_list_ID[i]}
    printf "> Now testing $DUT_NAME_LOWER ($DUT_NAME_UPPER) \r\n> ID:$DUT_ID \r\n> Port:$DUT_SERIAL_PORT \r\n"


    # enter current device under test directory
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER

    #no filter would support ALL projects not just ble
    project_filter='BLE_'
    # tests projects
    for dir in ./*/; do
        #(cd "$dir")
        if [[ "$dir" == *"$project_filter"* ]]; then

            export PROJECT_NAME=$(echo "$dir" | tr -d /.)
            case $PROJECT_NAME in

            "BLE_datc")
                run_notConntectedTest
                ;;

            "BLE_dats")
              #  run_notConntectedTest
                ;;

            "BLE_mcs")
             #   run_notConntectedTest
                ;;

            "BLE_fit")
              #  run_notConntectedTest
                ;;

            "BLE_fcc")

                #Nothing to do here, no test for fcc
                #project is built up top and that will detect a failure if any

                ;;

            "BLE_FreeRTOS")
              #  run_notConntectedTest
                ;;

            "BLE_otac")
              #  run_notConntectedTest
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

done

