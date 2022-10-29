#!/bin/bash

_DEBUG="ON"
function DEBUG()
{
    [ "$_DEBUG" == "ON" ] && $@
}

echo "###############################################################################"
echo "# usage: ./verify_tifs.sh path_of_MaximSDK                                    #"
echo "# example: ./verify_tifs.sh /home/\$USER/Workspace/msdk-main                   #"
echo "###############################################################################"
echo 

#if (( $# < 1 ))
#then
#    print "%b" "Error: not enough arguments.\n" >&2
#    exit 1
#fi

export MSDK_DIR=$1
DEBUG echo MSDK_DIR: $MSDK_DIR

if [ ! -d $MSDK_DIR ]; then
    echo The input MSDK_DIR does not exist. Try to use local repository.
    MSDK_DIR=/home/$USER/Workspace/msdk-main
    echo New msdk folder is $MSDK_DIR
fi

# Note: index of the two DevKit boards are 1-based.
if [ `hostname` == "yingcai-OptiPlex-790" ]
then
    sniffer_sn="000680435664"
    jtag_sn_1=0409000069c5c14600000000000000000000000097969906
    jtag_sn_2=040900006bd8439a00000000000000000000000097969906
    DevKitUart0Sn_1="D3073IDG"
    DevKitUart0Sn_2="D309ZDE9"
    DevKitUart3Sn_1="DT03OFRJ"
    DevKitUart3Sn_2="DT03NSU1"    
else  # wall-e
    sniffer_sn="000680435664"
    jtag_sn_1=04091702d4f18ac600000000000000000000000097969906
    jtag_sn_2=04091702f7f18a2900000000000000000000000097969906
    DevKitUart0Sn_1="D309ZDFB"
    DevKitUart0Sn_2="D3073ICQ"
    DevKitUart3Sn_1="DT03O9WB"
    DevKitUart3Sn_2="DT03OFQ0"
fi

export snifferSerial=/dev/"$(ls -la /dev/serial/by-id | grep -n $sniffer_sn | rev | cut -b 1-7 | rev)"

export CMSIS_DAP_ID_1=$jtag_sn_1
export TARGET_1_LC=max32655
export TARGET_1_UC=MAX32655
export devSerial_1=/dev/"$(ls -la /dev/serial/by-id | grep -n $DevKitUart0Sn_1 | rev | cut -b 1-7 | rev)"
export devUart3Serial_1=/dev/"$(ls -la /dev/serial/by-id | grep -n $DevKitUart3Sn_1 | rev | cut -b 1-7 | rev)"
export TARGET_1_CFG=${TARGET_1_LC}.cfg

export CMSIS_DAP_ID_2=$jtag_sn_2
export TARGET_2_LC=max32665
export TARGET_2_UC=MAX32665
export devSerial_2=/dev/"$(ls -la /dev/serial/by-id | grep -n $DevKitUart0Sn_2 | rev| cut -b 1-7| rev)"
export devUart3Serial_2=/dev/"$(ls -la /dev/serial/by-id | grep -n $DevKitUart3Sn_2 | rev| cut -b 1-7| rev)"
export TARGET_2_CFG=${TARGET_2_LC}.cfg

export APP_EXAMPLES_PATH=$MSDK_DIR/Examples
export EXAMPLE_TEST_PATH=$MSDK_DIR/Examples_tests
export OPENOCD_TOOL_PATH=/home/lparm/Tools/openocd/tcl

export VERBOSE_TEST=1

echo "nRF51 dongle: $snifferSerial"
echo "Found Serial Device 1: $devSerial_1"
echo "Found Serial Device 2: $devSerial_2"
echo "DevKit UART3 1: $devUart3Serial_1"
echo "DevKit UART3 2: $devUart3Serial_2"

export failedTestList=" "

if [ `hostname` == "wall-e" ] 
then
    export OPENOCD_TCL_PATH=/home/btm-ci/Tools/openocd/tcl
    export OPENOCD=/home/btm-ci/Tools/openocd/src/openocd
    export ROBOT=/home/btm-ci/.local/bin/robot
else
    export OPENOCD_TCL_PATH=/home/$USER/softwares/openocd/tcl
    export OPENOCD=/home/$USER/softwares/openocd/src/openocd
    export ROBOT=/home/btm-ci/.local/bin/robot
fi

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
trap script_clean_up EXIT SIGINT

# Function accepts parameters: filename, CMSIS_DAP_ID_x 
function flash_with_openocd()
{
    # mass erase and flash
    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt;max32xxx mass_erase 0" -c "program $1 verify reset exit" > /dev/null &
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

#####################################################################
# In order to verify the TIFS, two DevKit boards are required to be
# programmed with BLE5_ctr project.
function prepare_tifs_test()
{
    project_marker

    cd $PROJECT_NAME
    set +x
    echo "> Flashing $PROJECT_NAME"
    make -j8 > /dev/null
    
    cd build/
    flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1
    flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_2

    cd $MSDK_DIR/Examples/$TARGET_1_UC
}

function project_marker()
{
    echo "=============================================================================="
    printf "\r\n$PROJECT_NAME Flashing Procedure \r\n\r\n"
}

###############################################################################
# checkout submodules
# Update the submodules, this will use ssh
cd $MSDK_DIR/
git submodule init
git submodule sync
git submodule update --init --recursive

# build BLE examples
cd $MSDK_DIR/Examples/$TARGET_1_UC
DEBUG echo `pwd`
SUBDIRS=$(find . -type d -name "BLE5*")
DEBUG echo $SUBDIRS

for dir in ${SUBDIRS}
    do
    echo "---------------------------------------"
    echo " Validation build for ${dir}"
    echo "---------------------------------------"
    make -C ${dir} clean
    make -C ${dir} libclean
    make -C ${dir} -j8
done

project_filter='BLE5_ctr'


cd $MSDK_DIR/Examples/$TARGET_1_UC

# tests projects
for dir in ./*/; do
    #(cd "$dir")
    if [[ "$dir" == *"$project_filter"* ]]; then
    
        export PROJECT_NAME=$(echo "$dir" | tr -d /.) 
        echo PROJECT_NAME: $PROJECT_NAME

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

        "BLE5_ctr" )
            prepare_tifs_test            
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
done
 
echo "=== DONE ==="
