#!/bin/bash

echo
echo "##############################################################################################"
echo "# test_launcher.sh <target(lower case)> <DUT control port> <DUT DAP sn> <board type>         #"
echo "##############################################################################################"
echo

echo args: $@
if [[ $# -eq 4 ]]; then
    DUT_BOARD_TYPE=$4
else
    DUT_BOARD_TYPE=EvKit_V1
fi
echo "DUT_BOARD_TYPE:" $DUT_BOARD_TYPE
echo

EXAMPLE_TEST_PATH=$(pwd)
cd ../../../../
MSDK_DIR=$(pwd)
echo pwd=$(pwd)
failedTestList=" "
numOfFailedTests=0

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
#used to kill any openocd instances should the script exit prematurely
trap script_clean_up EXIT SIGINT
#****************************************************************************************************
function initial_setup() {

    # Get correct boards config file and tools paths when running on Wall-E
    if [ $(hostname) == "wall-e" ]; then
        echo "On machine wall-e"
        echo

        FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
        # WALL-E  paths
        export OPENOCD_TCL_PATH=/home/btm-ci/Tools/openocd/tcl
        export OPENOCD=/home/btm-ci/Tools/openocd/src/openocd
        export ROBOT=/home/btm-ci/.local/bin/robot

        MAIN_DEVICE_ID=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board1']['daplink'])")
        main_uart=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board1']['uart0'])")
        MAIN_DEVICE_SERIAL_PORT=/dev/"$(ls -la /dev/serial/by-id | grep -n $main_uart | rev | cut -d "/" -f1 | rev)"

        # Get the serial number of all daplink devices, this is used to erase them all.
        DEVICE1=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board1']['daplink'])")
        DEVICE2=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board2']['daplink'])")
        DEVICE3=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32665_board1']['daplink'])")
        DEVICE4=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_w1']['daplink'])")
        DEVICE5=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_A5']['DAP_sn'])")
    elif [ $(hostname) == "yingcai-OptiPlex-790" ]; then
        echo "On machine yingcai-OptiPlex-790"
        echo

        FILE=/home/$USER/Workspace/Resource_Share/boards_config.json

        export OPENOCD_TCL_PATH=/home/$USER/Tools/openocd/tcl
        export OPENOCD=/home/$USER/Tools/openocd/src/openocd
        export ROBOT=/home/$USER/.local/bin/robot

        MAIN_DEVICE_ID=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board_y1']['daplink'])")
        main_uart=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board_y1']['uart0'])")
        MAIN_DEVICE_SERIAL_PORT=/dev/"$(ls -la /dev/serial/by-id | grep -n $main_uart | rev | cut -d "/" -f1 | rev)"

        # Get the serial number of all daplink devices, this is used to erase them all.
        DEVICE1=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board_y1']['daplink'])")
        DEVICE2=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board_y2']['daplink'])")
        DEVICE3=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32665_board_2']['daplink'])")
        DEVICE4=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_3']['daplink'])")
        DEVICE5=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_A3']['DAP_sn'])")
    else
        # Local- eddie desktop
        FILE=/home/$USER/boards_config.json
        # local paths
        export OPENOCD_TCL_PATH=/home/eddie/workspace/openocd/tcl
        export OPENOCD=/home/eddie/workspace/openocd/src/openocd
        export ROBOT=/home/eddie/.local/bin/robot
    fi

    # "Main device" is the ME17 used as the cleint dudring connected tests
    MAIN_DEVICE_NAME_UPPER=MAX32655
    MAIN_DEVICE_NAME_LOWER=max32655

    # setup  all DUT (Device Under Test) varaibles passed into the scripts as arguments
    # eg:
    # $1 = max32655 (device name)
    # $2 = DT30DZ0  ( UART)
    # $3 = 440002342304433434r323452354696 (Dap-Link ID)
    DUT_NAME_LOWER=$1
    DUT_NAME_UPPER=$(echo $DUT_NAME_LOWER | tr '[:lower:]' '[:upper:]')
    DUT_SERIAL_PORT=/dev/$(ls -la /dev/serial/by-id | grep -n $2 | rev | cut -d "/" -f1 | rev)
    DUT_ID=$3

    # Directories needed for robotframework logs
    cd $EXAMPLE_TEST_PATH
    if [[ -e results ]]; then
        echo "results folder exists"
    else
        mkdir $EXAMPLE_TEST_PATH/results
    fi

    cd $EXAMPLE_TEST_PATH/results
    mkdir failed_elfs
    mkdir $DUT_NAME_UPPER
    ls
    cd $DUT_NAME_UPPER
    pwd
    mkdir BLE_dat_cs
    mkdir BLE_datc
    mkdir BLE_dats
    mkdir BLE_fit
    mkdir BLE_FreeRTOS
    mkdir BLE_mcs
    mkdir BLE_otac
    mkdir BLE_ota_cs
}
#****************************************************************************************************
# Function accepts parameters: device, CMSIS-DAP serial #
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

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]; then
        printf "> Verify failed , flashibng again \r\n"
        # Reprogram the device if the verify failed
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1.elf verify reset exit" >/dev/null &
        openocd_dapLink_pid=$!
    fi
}

#****************************************************************************************************
# params: target in lower case, DAP sn
function reset_board_by_openocd() {
    echo "function: ${FUNCNAME[0]} $@"

    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg \
        -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH \
        -c "adapter serial $2" \
        -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" \
        -c "init; reset run" >/dev/null &

    openocd_dapLink_pid=$!
    sleep 0.5
    if ps -p $openocd_dapLink_pid >/dev/null; then
        kill -9 $openocd_dapLink_pid || true
    fi
    set -e
}

#****************************************************************************************************
# Function accepts parameters: device, CMSIS-DAP serial #
function flash_with_openocd_fast() {
    # mass erase and flash
    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 333$3" -c "telnet_port 444$3" -c "tcl_port 666$3" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1.elf verify reset exit" >/dev/null &
    openocd_dapLink_pid=$!
    set -e
}
#****************************************************************************************************
# Function accepts parameters: device, CMSIS-DAP serial #
function softreset_with_openocd() {

    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init;reset exit" >/dev/null &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid; do
        sleep 1
        # we can add a timeout here if we want
    done
    set -e
}
#****************************************************************************************************
# Function accepts parameters:device , CMSIS-DAP serial #
function erase_with_openocd() {
    echo "-----------------------------------------------------------------------------------------"
    printf "> Erasing $1 : $2 \r\n"
    echo "-----------------------------------------------------------------------------------------"
    echo

    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; max32xxx mass_erase 0;" -c " exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    wait $openocd_dapLink_pid

    # erase second bank of larger chips
    if [[ $1 != "max32655" ]]; then
        printf "> Erasing second bank of device: $1 with ID:$2 \r\n"
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt; max32xxx mass_erase 1;" -c " exit" &
        openocd_dapLink_pid=$!
        # wait for openocd to finish
        wait $openocd_dapLink_pid
    fi
}
#****************************************************************************************************
function run_notConntectedTest() {

    print_project_banner
    echo "run_notConntectedTest"
    echo
    cd $PROJECT_NAME
    set +x
    echo "> Flashing $DUT_NAME_UPPER $PROJECT_NAME"
    echo
    # make -j8 projects are build in validation build step
    cd build/
    flash_with_openocd $DUT_NAME_LOWER $DUT_ID
    #place to store robotframework results

    cd $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER

    cd $EXAMPLE_TEST_PATH/tests
    # do not let a single failed test stop the testing of the rest
    set +e
    #runs desired test
    echo
    echo "$ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/$PROJECT_NAME -v SERIAL_PORT_1:$DUT_SERIAL_PORT $PROJECT_NAME.robot"
    echo
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/$PROJECT_NAME -v SERIAL_PORT_1:$DUT_SERIAL_PORT $PROJECT_NAME.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| $PROJECT_NAME ($DUT_NAME_UPPER) "

        # save elf of failed test
        cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/$PROJECT_NAME/build
        cp $DUT_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$DUT_NAME_LOWER"_"$PROJECT_NAME.elf
    fi
    set -e

    # get back to target directory
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER
}
#****************************************************************************************************
function flash_bootloader() {
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/Bootloader
    make clean

    if [ $4 == "WLP_V1" ]; then
        make -j8 BOARD=WLP_V1
    else
        make USE_INTERNAL_FLASH=$1 -j8
    fi

    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/Bootloader/build
    printf ">>>>>>>   Flashing Bootloader on DUT\r\n\r\n"

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
#****************************************************************************************************
function erase_all_devices() {
    erase_with_openocd max32655 $DEVICE1
    erase_with_openocd max32655 $DEVICE2
    erase_with_openocd max32665 $DEVICE3
    erase_with_openocd max32690 $DEVICE4
    erase_with_openocd max32690 $DEVICE5
}
#****************************************************************************************************
function print_project_banner() {
    echo
    echo "*****************************************************************************************"
    printf "> Start of testing $DUT_NAME_LOWER ($DUT_NAME_UPPER) \r\n> ID:$DUT_ID \r\n> Port:$DUT_SERIAL_PORT \r\n\r\n"
}
#****************************************************************************************************
function change_advertising_names_walle() {
    # change advertising name for projects under test to avoid
    # connections with office devices
    printf "> changing advertising names : $MSDK_DIR/Examples/$DUT_NAME_UPPER\r\n\r\n"
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats
    perl -i -pe "s/\'D\'/\'X\'/g" dats_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_datc
    perl -i -pe "s/\'D\'/\'X\'/g" datc_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otac
    perl -i -pe "s/\'S\'/\'X\'/g" datc_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    perl -i -pe "s/\'S\'/\'X\'/g" dats_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_FreeRTOS
    perl -i -pe "s/\'S\'/\'X\'/g" dats_main.c
    # change advertising name to scan for on the main device client apps
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_datc
    perl -i -pe "s/\'D\'/\'X\'/g" datc_main.c
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
    perl -i -pe "s/\'S\'/\'X\'/g" datc_main.c
    # build BLE examples
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER
}
#****************************************************************************************************
function change_advertising_names_local() {
    # change advertising name for projects under test to avoid
    # connections with office devices
    printf "> changing advertising names : $MSDK_DIR/Examples/$DUT_NAME_UPPER\r\n\r\n"
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats
    perl -i -pe "s/\'D\'/\'Z\'/g" dats_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_datc
    perl -i -pe "s/\'D\'/\'Z\'/g" datc_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otac
    perl -i -pe "s/\'S\'/\'Z\'/g" datc_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    perl -i -pe "s/\'S\'/\'Z\'/g" dats_main.c
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_FreeRTOS
    perl -i -pe "s/\'S\'/\'Z\'/g" dats_main.c
    # change advertising name to scan for on the main device client apps
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_datc
    perl -i -pe "s/\'D\'/\'Z\'/g" datc_main.c
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
    perl -i -pe "s/\'S\'/\'Z\'/g" datc_main.c
}

#****************************************************************************************************
#***************************************** Start of test script *************************************
#****************************************************************************************************

# parameterizes all relavent device variables and makes directories to store robot framework log files
# takes the three args given to the script: eg: max32655 , D3073ICQ , 0409000052fb0cd70000000000000000097969906
initial_setup $1 $2 $3

# does what it says
erase_all_devices

# change advertising names to avoid connection with office devices
# and builds all examples named "BLE" in the directory of the device under test
if [ $(hostname) == "wall-e" ]; then

    change_advertising_names_walle
    #build all examples
    SUBDIRS=$(find . -type d -name "BLE*")
    for dir in ${SUBDIRS}; do
        echo "---------------------------------------"
        echo " Validation build for ${dir}"
        echo "---------------------------------------"
        make -C ${dir} clean
        make -C ${dir} libclean
        make -C ${dir} -j8 BOARD=$DUT_BOARD_TYPE
    done
else
    # Allows me to run this script on my local machine with no modifications
    change_advertising_names_local
    # build BLE examples
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER
    SUBDIRS=$(find . -type d -name "BLE_*")
    for dir in ${SUBDIRS}; do
        echo "---------------------------------------"
        echo " Validation build for ${dir}"
        echo "---------------------------------------"
        make -C ${dir} clean
        #make -C ${dir} libclean
        make -C ${dir} -j8 BOARD=$DUT_BOARD_TYPE
    done
fi

# enter  device directory
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER

# Iterate through the BLE projects, flash and run robotframework tests
project_filter='BLE_'
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
            ;;

        *) ;;

        esac

    fi

    let projIdx++

done # end non connected tests

echo
echo "****************************************************************************************************"
echo "*********************************** Start of Datc/s connected tests ********************************"
echo "****************************************************************************************************"
echo

erase_all_devices

# Flash MAIN_DEVICE with BLE_datc
cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_datc
echo pwd=$(pwd)
echo

make -j8

# flash client first because it takes longer
cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_datc/build
printf "\r\n> Flashing BLE_datc on main device: $MAIN_DEVICE_NAME_UPPER\r\n"
#flash_with_openocd_fast $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID 1
flash_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID 1

# flash DUT with BLE_dats
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats/build
printf "\r\n> Flashing BLE_dats on DUT $DUT_NAME_UPPER\r\n"
#flash_with_openocd_fast $DUT_NAME_LOWER $DUT_ID 2
flash_with_openocd $DUT_NAME_LOWER $DUT_ID 2

# Reset the two boards
reset_board_by_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID
reset_board_by_openocd $DUT_NAME_LOWER $DUT_ID

cd $EXAMPLE_TEST_PATH/tests
# runs desired test but do not exit on failure, save result to list for printing later
set +e
# Robot arguments are:
# directory to save log files
# serial port 1
# optional serial port 2
# robot test file
echo
echo "$ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_dat_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_dat_cs.robot"
echo
$ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_dat_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_dat_cs.robot
let "testResult=$?"
if [ "$testResult" -ne "0" ]; then
    # update failed test count
    let "numOfFailedTests+=$testResult"
    failedTestList+="| BLE_dat_cs ($DUT_NAME_UPPER) "
    # test failed, save elfs datc/s
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_datc/build
    cp $MAIN_DEVICE_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$MAIN_DEVICE_NAME_LOWER"_BLE_datcs_client.elf"
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_dats/build
    cp $DUT_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$DUT_NAME_LOWER"_BLE_datcs_server.elf"
fi
set -e

echo
echo "Make sure to erase main device and current DUT to it does not store bonding info."
echo
erase_with_openocd $DUT_NAME_LOWER $DUT_ID
erase_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID

echo
echo "****************************************************************************************************"
echo "*********************************** Start of OTAC/s connected tests ********************************"
echo "****************************************************************************************************"

if [[ $DUT_NAME_UPPER != "MAX32690" ]]; then

    #make sure all files have correct settings
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    perl -i -pe "s/FW_VERSION_MAJOR 2/FW_VERSION_MAJOR 1/g" wdxs_file_ext.c
    sed -i "s/USE_INTERNAL_FLASH ?=1/USE_INTERNAL_FLASH ?=0/g" project.mk
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/Bootloader
    sed -i "s/USE_INTERNAL_FLASH ?=1/USE_INTERNAL_FLASH ?=0/g" project.mk
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
    #appends TARGET , TARGET_UC and TARGET_LC to the make commands and sets them to $DUT_NAME_UPPER and $DUT_NAME_LOWER
    sed -i 's/BUILD_DIR=\$(FW_BUILD_DIR) PROJECT=fw_update/BUILD_DIR=\$(FW_BUILD_DIR) PROJECT=fw_update TARGET='"$DUT_NAME_UPPER"' TARGET_UC='"$DUT_NAME_UPPER"' TARGET_LC='"$DUT_NAME_LOWER"'/g' project.mk
    sed -i 's/BUILD_DIR=\$(FW_BUILD_DIR) \$(FW_UPDATE_BIN)/BUILD_DIR=\$(FW_BUILD_DIR) \$(FW_UPDATE_BIN) TARGET='"$DUT_NAME_UPPER"' TARGET_UC='"$DUT_NAME_UPPER"' TARGET_LC='"$DUT_NAME_LOWER"'/g' project.mk

    sleep 1
    # Make OTAS V1 and flash
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    make clean
    make USE_INTERNAL_FLASH=0 -j8
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas/build
    printf ">>>>>>>> Flashing BLE_otas V1 on DUT $DUT_NAME_UPP\r\n\r\n"
    flash_with_openocd $DUT_NAME_LOWER $DUT_ID

    # Flash bootloader also make sure it uses external flash version
    flash_bootloader 0

    # change OTAS firmware version and rebuild
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    # change firmware version to verify otas worked
    perl -i -pe "s/FW_VERSION_MAJOR 1/FW_VERSION_MAJOR 2/g" wdxs_file_ext.c
    make clean
    make USE_INTERNAL_FLASH=0 -j8

    # make OTAC
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
    # flash MAIN_DEVICE with BLE_OTAC, it will use the OTAS bin with new firmware
    make clean
    make FW_UPDATE_DIR=../../$DUT_NAME_UPPER/BLE_otas -j8

    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac/build
    printf ">>>>>>> Flashing BLE_otac on main device: $MAIN_DEVICE_NAME_UPPER\r\n "
    flash_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID
    printf ">>>>>>> Flashing done"

    #revert files back
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
    sed -i 's/TARGET='"$DUT_NAME_UPPER"'//' project.mk
    sed -i 's/TARGET_UC='"$DUT_NAME_UPPER"'//' project.mk
    sed -i 's/TARGET_LC='"$DUT_NAME_LOWER"'//' project.mk
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
    perl -i -pe "s/FW_VERSION_MAJOR 2/FW_VERSION_MAJOR 1/g" wdxs_file_ext.c
    # give time to connect
    sleep 1

    set +e
    # runs desired test
    cd $EXAMPLE_TEST_PATH/tests
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_ota_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_ota_cs.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ]; then
        # update failed test count
        let "numOfFailedTests+=$testResult"
        failedTestList+="| BLE_ota_cs_ext ($DUT_NAME_UPPER) "
        # test failed, save elfs datc/s
        cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac/build
        cp $MAIN_DEVICE_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$MAIN_DEVICE_NAME_LOWER"_"$DUT_NAME_LOWER"_BLE_otacs_client_ext.elf"
        cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas/build
        cp $DUT_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$DUT_NAME_LOWER"_BLE_otacs_server_ext.elf"
    fi
    set -e

    # make sure to erase main device and current DUT to it does not store bonding info
    erase_with_openocd $DUT_NAME_LOWER $DUT_ID
    #   erase_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID
fi

#****************************************************************************************************
#*********************************** Start of OTAC/s connected tests (internal flash) ***************
#****************************************************************************************************

erase_all_devices

#make sure all files have correct settings
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
perl -i -pe "s/FW_VERSION_MAJOR 2/FW_VERSION_MAJOR 1/g" wdxs_file_int.c
sed -i "s/USE_INTERNAL_FLASH ?=0/USE_INTERNAL_FLASH ?=1/g" project.mk
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/Bootloader
sed -i "s/USE_INTERNAL_FLASH ?=0/USE_INTERNAL_FLASH ?=1/g" project.mk
cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
#appends TARGET , TARGET_UC and TARGET_LC to the make commands and sets them to $DUT_NAME_UPPER and $DUT_NAME_LOWER
sed -i 's/BUILD_DIR=\$(FW_BUILD_DIR) PROJECT=fw_update/BUILD_DIR=\$(FW_BUILD_DIR) PROJECT=fw_update TARGET='"$DUT_NAME_UPPER"' TARGET_UC='"$DUT_NAME_UPPER"' TARGET_LC='"$DUT_NAME_LOWER"'/g' project.mk
sed -i 's/BUILD_DIR=\$(FW_BUILD_DIR) \$(FW_UPDATE_BIN)/BUILD_DIR=\$(FW_BUILD_DIR) \$(FW_UPDATE_BIN) TARGET='"$DUT_NAME_UPPER"' TARGET_UC='"$DUT_NAME_UPPER"' TARGET_LC='"$DUT_NAME_LOWER"'/g' project.mk

sleep 1
# Make OTAS V1 and flash
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
make clean
make USE_INTERNAL_FLASH=1 -j8
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas/build

printf "\r\n>>>>>>>> Flashing BLE_otas on DUT $DUT_NAME_UPP\r\n\r\n"

flash_with_openocd $DUT_NAME_LOWER $DUT_ID

# Flash bootloader also make sure it uses external flash version
flash_bootloader 1

printf "\r\nChange OTAS firmware version and rebuild.\r\n\r\n"
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
# change firmware version to verify otas worked
perl -i -pe "s/FW_VERSION 1/FW_VERSION 2/g" wdxs_file.c
perl -i -pe "s/FW_VERSION_MAJOR 1/FW_VERSION_MAJOR 2/g" wdxs_file_int.c
make clean
if [ ${DUT_BOARD_TYPE} == "WLP_V1" ]; then
    make -j8 BOARD=$DUT_BOARD_TYPE
else
    make USE_INTERNAL_FLASH=1 -j8
fi

# make OTAC
cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
make clean
make FW_UPDATE_DIR=../../$DUT_NAME_UPPER/BLE_otas -j8 BOARD=$DUT_BOARD_TYPE

cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac/build

printf ">>>>>>> Flashing BLE_otac on main device: $MAIN_DEVICE_NAME_UPPER\r\n "
flash_with_openocd $MAIN_DEVICE_NAME_LOWER $MAIN_DEVICE_ID
printf ">>>>>>> Flashing done"

#revert files back
cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac
sed -i 's/TARGET='"$DUT_NAME_UPPER"'//' project.mk
sed -i 's/TARGET_UC='"$DUT_NAME_UPPER"'//' project.mk
sed -i 's/TARGET_LC='"$DUT_NAME_LOWER"'//' project.mk
cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas
perl -i -pe "s/FW_VERSION_MAJOR 2/FW_VERSION_MAJOR 1/g" wdxs_file_int.c
# give time to connect
sleep 1

set +e
# runs desired test
cd $EXAMPLE_TEST_PATH/tests
echo
echo "$ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_ota_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_ota_cs.robot"
echo
$ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/BLE_ota_cs/ -v SERIAL_PORT_1:$MAIN_DEVICE_SERIAL_PORT -v SERIAL_PORT_2:$DUT_SERIAL_PORT BLE_ota_cs.robot
let "testResult=$?"
if [ "$testResult" -ne "0" ]; then
    # update failed test count
    let "numOfFailedTests+=$testResult"
    failedTestList+="| BLE_ota_cs_int ($DUT_NAME_UPPER) "
    # test failed, save elfs datc/s
    cd $MSDK_DIR/Examples/$MAIN_DEVICE_NAME_UPPER/BLE_otac/build
    cp $MAIN_DEVICE_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$MAIN_DEVICE_NAME_LOWER"_"$DUT_NAME_LOWER"_BLE_otacs_client_int.elf"
    cd $MSDK_DIR/Examples/$DUT_NAME_UPPER/BLE_otas/build
    cp $DUT_NAME_LOWER.elf $EXAMPLE_TEST_PATH/results/failed_elfs/$DUT_NAME_LOWER"_BLE_otacs_server_int.elf"
fi
set -e

erase_all_devices

echo "=============================================================================="
echo "=============================================================================="
if [ "$numOfFailedTests" -ne "0" ]; then
    printf "Test completed with $numOfFailedTests failed tests located in: \r\n $failedTestList"
else
    echo "Relax! ALL TESTS PASSED"
fi
echo "=============================================================================="
echo "=============================================================================="
exit $numOfFailedTests
