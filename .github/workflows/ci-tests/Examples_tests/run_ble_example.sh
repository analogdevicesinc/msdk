#!/bin/bash

_DEBUG="ON"
function DEBUG()
{
    [ "$_DEBUG" == "ON" ] && $@
}

DEBUG echo
DEBUG echo "###############################################################################"
DEBUG echo "# usage: ./test_ble_examples.sh  # in path_of_msdk_repo/Example_tests         #"
DEBUG echo "###############################################################################"
DEBUG echo

start_secs=$(date +%s)

# Note: index of the two DevKit boards are 1-based.
if [ `hostname` == "wall-e" ]
then
    sniffer_sn="000680435664"
    CMSIS_DAP_ID_1=04091702f7f18a2900000000000000000000000097969906
    CMSIS_DAP_ID_2=0409170246dfc09500000000000000000000000097969906
    DevKitUart0Sn_1="D309ZDFB"
    DevKitUart0Sn_2="D3073ICQ"
    DevKitUart3Sn_1="DT03O9WB"
    DevKitUart3Sn_2="DT03OFQ0"
else  # Ying's two EvKit boards
    sniffer_sn="15261529E1F6E30B"
    CMSIS_DAP_ID_1=0409000069c5c14600000000000000000000000097969906
    CMSIS_DAP_ID_2=040900006bd8439a00000000000000000000000097969906
    DevKitUart0Sn_1="D3073IDG"
    DevKitUart0Sn_2="D309ZDE9"
    DevKitUart3Sn_1="DT03OFRJ"
    DevKitUart3Sn_2="DT03NSU1"
fi

# Target under test
export TARGET_1_LC=max32655
export TARGET_1_UC=MAX32655
export devSerial_1=/dev/"$(ls -la /dev/serial/by-id | grep -n $DevKitUart0Sn_1 | rev | cut -b 1-7 | rev)"
export TARGET_1_CFG=${TARGET_1_LC}.cfg

# Helper device for connected tests (ME18 in this case)
export TARGET_2_LC=max32655
export TARGET_2_UC=MAX32655
export devSerial_2=/dev/"$(ls -la /dev/serial/by-id | grep -n $DevKitUart0Sn_2 | rev | cut -b 1-7 | rev)"
export TARGET_2_CFG=${TARGET_2_LC}.cfg

export TEST_BOARD=EvKit_V1

#tried making this a parameter but it was not working?
echo "> --- Found Serial Device 1 @ : $devSerial_1"
echo "> --- Found Serial Device 2 @ : $devSerial_2"

export EXAMPLE_TEST_PATH=$(pwd)
DEBUG echo EXAMPLE_TEST_PATH=$EXAMPLE_TEST_PATH

cd ../
export MSDK_DIR=$(pwd)
DEBUG echo MSDK_DIR=$MSDK_DIR

export VERBOSE_TEST=1
export failedTestList=" "

export OPENOCD_TCL_PATH=/home/$USER/Tools/openocd/tcl
#export OPENOCD_TCL_PATH=/home/eddie/workspace/openocd/tcl
export OPENOCD=/home/$USER/Tools/openocd/src/openocd

export ROBOT=/home/$USER/.local/bin/robot

function script_clean_up()
{
    # check if some runoff opeocd instance is running
    set +e
    if ps -p $openocd_dapLink_pid > /dev/null
    then
    kill -9 $openocd_dapLink_pid || true
    fi

    set -e
}
trap script_clean_up  EXIT SIGINT

# Function accepts parameters: filename , CMSIS_DAP_ID_x 
function flash_with_openocd()
{
    # mass erase and flash
    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH  -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt;max32xxx mass_erase 0" -c "program $1 verify reset exit" > /dev/null &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid ; do
    sleep 1
    # we can add a timeout here if we want
    done
    set -e
    # Attempt to verify the image, prevent exit on error
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash verify_image $1; reset; exit"

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]
    then
    # Reprogram the device if the verify failed
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH  -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt;max32xxx mass_erase 0" -c "program $1 verify reset exit" > /dev/null &
    openocd_dapLink_pid=$!
    fi
}
# This function accepts a CMSIS device ID as a parameter
function erase_with_openocd()
{
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_2_CFG -s $OPENOCD_TCL_PATH/  -c "cmsis_dap_serial  $1" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash erase_address 0x10004000 0x40000; reset exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    wait $openocd_dapLink_pid
}

function run_notConntectedTest()
{
    project_marker
    cd $PROJECT_NAME
    set +x
    echo "> Flashing $PROJECT_NAME"
    make -j8 > /dev/null
    cd build/
    flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1
    #place to store robotframework results
    mkdir $EXAMPLE_TEST_PATH/results/$PROJECT_NAME
    cd $EXAMPLE_TEST_PATH/tests
    # do not let a single failed test stop the testing of the rest
    set +e
    #runs desired test 
    DEBUG echo
    DEBUG echo ----------------------------------------------------------------
    DEBUG echo $ROBOT -d $EXAMPLE_TEST_PATH/results/$PROJECT_NAME/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 $PROJECT_NAME.robot
    DEBUG echo
    $ROBOT -d $EXAMPLE_TEST_PATH/results/$PROJECT_NAME/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 $PROJECT_NAME.robot
    let "testResult=$?"
    if [ "$testResult" -ne "0" ] 
    then
        # update failed test count
        let "numOfFailedTests+=$testResult"  
        failedTestList+="| $PROJECT_NAME "
    fi
    set -e     
    #get back to target directory
    cd $MSDK_DIR/Examples/$TARGET_1_UC
}

function project_marker()
{
    echo "=============================================================================="
    printf "\r\n$PROJECT_NAME Flashing Procedure \r\n\r\n"
}
#--------------------------------------------------------------------------------------------
# build BLE examples
cd $MSDK_DIR/Examples/$TARGET_1_UC
SUBDIRS=$(find . -type d -name "BLE*")
DEBUG echo SUBDIRS=$SUBDIRS
DEBUG echo
for dir in ${SUBDIRS}
    do
    echo "---------------------------------------"
    echo " Validation build for ${dir}"
    echo "---------------------------------------"
    #make -C ${dir} clean
    #make -C ${dir} libclean
    make -C ${dir} -j8
done

#remove old robtoframework logs
cd $EXAMPLE_TEST_PATH/results/
rm -r *
#keep track of directory count, might need it
projIdx=1
numOfFailedTests=0
#no filter would support ALL projects not just ble
project_filter='BLE_'

# testing ME17 only for now 
cd $MSDK_DIR/Examples/$TARGET_1_UC

#erase the helper device so that its not running an app that might connect 
#to current app under test
erase_with_openocd $CMSIS_DAP_ID_2
cd $MSDK_DIR/Examples/$TARGET_1_UC

DEBUG echo ==================================================
DEBUG echo tests projects
DEBUG echo ==================================================
DEBUG echo 
for dir in ./*/; do
    #(cd "$dir")
    DEBUG echo dir=$dir
    DEBUG echo
    if [[ "$dir" == *"$project_filter"* ]]; then
        export PROJECT_NAME=$(echo "$dir" | tr -d /.) 
        DEBUG echo PROJECT_NAME=$PROJECT_NAME
        DEBUG echo
        case $PROJECT_NAME in
        "BLE_datc")
            run_notConntectedTest 
            ;;

        "BLE_dats") 
            run_notConntectedTest
            ;;

        "BLE_mcs" )
            run_notConntectedTest 
            ;;

        "BLE_fit" )
            run_notConntectedTest 
            ;;

        # "BLE_fcc" )
        #     # todo:
        #     # execute related test
        #     echo Found BLE_fcc #place holder
        #     ;;

        "BLE_FreeRTOS" )
            run_notConntectedTest 
            ;;

        "BLE_otac" )
            run_notConntectedTest 
            ;;

        "BLE_otas" )
            # gets tested during conencted test below
            
            ;;

        "BLE_periph" )
            # No buttons implemented for this example so lets just make sure it builds, so we can ship it
            # cd $PROJECT_NAME
            # make -j8
            # let "numOfFailedTests+=$?"  
            # cd $MSDK_DIR/Examples/$TARGET_UC          
             ;;

        *)
            
            ;;

        esac

    fi
    let projIdx++

done
 
DEBUG echo "━━━━━━━━━━━━━━━━━━━━━━━| BLE_datc/dats Connection Test |━━━━━━━━━━━━━━━━━━━━━━━ "
DEBUG echo
#------ datc ME17
cd $MSDK_DIR/Examples/$TARGET_1_UC/BLE_datc/build
DEBUG echo `pwd`
ls *.elf
echo "> Flashing BLE_datc"
DEBUG echo TARGET_1_LCAR.elf=$TARGET_1_LC.elf
DEBUG echo
flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1

#------ dats on ME18
cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_dats/build
DEBUG echo `pwd`
ls *.elf
echo "> Flashing BLE_dats"
DEBUG echo TARGET_2_LCAR.elf=$TARGET_2_LC.elf
flash_with_openocd  $TARGET_2_LC.elf $CMSIS_DAP_ID_2

DEBUG echo ---
DEBUG echo "give them 10 secs to connect"
sleep 10

# directory for resuilts logs
cd $EXAMPLE_TEST_PATH/results
mkdir BLE_dat_cs
cd $EXAMPLE_TEST_PATH/tests
#runs desired test 
set +e
DEBUG echo ---
DEBUG echo $ROBOT -d $EXAMPLE_TEST_PATH/results/BLE_dat_cs/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 BLE_dat_cs.robot
DEBUG echo
$ROBOT -d $EXAMPLE_TEST_PATH/results/BLE_dat_cs/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 BLE_dat_cs.robot
let "testResult=$?"
if [ "$testResult" -ne "0" ] 
then
    # update failed test count
    let "numOfFailedTests+=$testResult"  
    failedTestList+="| BLE_dat_cs "
fi
set -e

DEBUG echo "------ Flash BLE_otac onto Device 1  : ME17"
DEBUG echo
cd $MSDK_DIR/Examples/$TARGET_1_UC/BLE_otac/build
DEBUG echo pwd=`pwd`
ls *.elf
echo "> Flashing BLE_otac"
flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1

DEBUG echo "------ Flash BLE_otas  onto Device 2  : ME18"
cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_otas/build
DEBUG echo pwd=`pwd`
ls *.elf
echo "> Flashing BLE_otas"
flash_with_openocd $TARGET_2_LC.elf $CMSIS_DAP_ID_2

DEBUG echo "------ Build & Flash Bootloader onto Device 2  : ME18"
cd $MSDK_DIR/Examples/$TARGET_2_UC/Bootloader
DEBUG echo pwd=`pwd`
make -j8 > /dev/null
cd $MSDK_DIR/Examples/$TARGET_2_UC/Bootloader/build
DEBUG echo --- pwd=`pwd`
ls *.elf
DEBUG echo
echo "> Flashing Bootloader"
#not using the flash_with_openocd function here because that causes the application code to be erased and only
#bootloader to remain
set +e
cmd='"program file verify reset exit"'
elf_file=$MSDK_DIR/Examples/$TARGET_2_UC/Bootloader/build/max32655.elf
cmd=${cmd/file/$elf_file}
DEBUG echo elf_file=$cmd
DEBUG echo $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH  -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt" -c $cmd "> /dev/null &"
$OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH  -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt" -c $cmd > /dev/null &
openocd_dapLink_pid=$!
# wait for openocd to finish
while kill -0 $openocd_dapLink_pid ; do
    sleep 1
    # we can add a timeout here if we want
done
set -e
# Attempt to verify the image, prevent exit on error
DEBUG echo $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash verify_image max32655.elf; reset; exit"
$OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH/ -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash verify_image max32655.elf; reset; exit"

# Check the return value to see if we received an error
if [ "$?" -ne "0" ]
then
    # Reprogram the device if the verify failed
    DEBUG echo $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH  -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt" -c "program max32655.elf verify reset exit"  "> /dev/null &"
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH  -c "cmsis_dap_serial  $CMSIS_DAP_ID_2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt" -c "program max32655.elf verify reset exit" > /dev/null &
    openocd_dapLink_pid=$!
fi

DEBUG echo "Give client/server 15 secs to establish connection"
sleep 15 
# directory for resuilts logs
cd $EXAMPLE_TEST_PATH/results
mkdir BLE_ota_cs
cd $EXAMPLE_TEST_PATH/tests
set +e
#runs desired test
$ROBOT  -d $EXAMPLE_TEST_PATH/results/BLE_ota_cs/ -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 BLE_ota_cs.robot
let "testResult=$?"
if [ "$testResult" -ne "0" ] 
then
    # update failed test count
    let "numOfFailedTests+=$testResult"  
    failedTestList+="| BLE_ota_cs "
fi
set -e
echo "=============================================================================="
echo "=============================================================================="
if [ "$numOfFailedTests" -ne "0" ] 
then
    echo "Test completed with $numOfFailedTests failed tests located in: "
    echo " $failedTestList | "
else
    echo  "Relax! ALL TESTS PASSED"
fi
echo "=============================================================================="
echo "=============================================================================="
exit $numOfFailedTests

end_secs=$(date +%s)
elapsed_secs=end_secs-start_secs
DEBUG echo Used secs: $elapsed_secs
DEBUG echo {$elapsed_secs}/60.0
