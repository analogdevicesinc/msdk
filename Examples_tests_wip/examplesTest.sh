#!/bin/bash
# Target under test
export CMSIS_DAP_ID_1=04091702adca825d00000000000000000000000097969906
export TARGET_1_LC=max32655
export TARGET_1_UC=MAX32655
export devSerial_1=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D3073IDU'| rev| cut -b 1-7| rev)"
export TARGET_1_CFG=${TARGET_1_LC}.cfg

# Helper device for connected tests (ME18 in this case)
export CMSIS_DAP_ID_2=0409000096dd433100000000000000000000000097969906
export devSerial_2=/dev/"$(ls -la /dev/serial/by-id | grep -n 'D307571R'| rev| cut -b 1-7| rev)"
export TARGET_2_LC=max32665
export TARGET_2_UC=MAX32665
export TARGET_2_CFG=${TARGET_2_LC}.cfg


export TEST_BOARD=EvKit_V1
export MSDK_DIR=$(pwd)
export APP_EXAMPLES_PATH=$MSDK_DIR/Examples
export EXAMPLE_TEST_PATH=$MSDK_DIR/Examples_tests
export OPENOCD_TOOL_PATH=/home/lparm/Tools/openocd/tcl
#tried making this a parameter but it was not working?
export VERBOSE_TEST=1
echo "> --- Found Serial Device 1 @ : $devSerial_1"
echo "> --- Found Serial Device 2 @ : $devSerial_2"

function script_clean_up()
{
    kill -9 $openocd_dapLink1_pid
    kill -9 $openocd_dapLink2_pid
}
trap script_clean_up  EXIT SIGINT

# Function accepts parameters: filename , CMSIS_DAP_ID_x
function flash_with_openocd()
{
    openocd -f $OPENOCD_TOOL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TOOL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TOOL_PATH  -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; program $1 verify reset exit" > /dev/null &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid ; do
    sleep 1
    # we can add a timeout here if we want
    done
    # Attempt to verify the image, prevent exit on error
    set +e
    openocd -f $OPENOCD_TOOL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TOOL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TOOL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash verify_image $1; reset; exit"

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]
    then
    set -e
    # Reprogram the device if the verify failed
    openocd -f $OPENOCD_TOOL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TOOL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TOOL_PATH  -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; program $1 verify reset exit" &
    openocd_dapLink_pid=$!
    fi
}

function erase_with_openocd()
{
    openocd -f $OPENOCD_TOOL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TOOL_PATH/target/$TARGET_1_CFG -s $OPENOCD_TOOL_PATH  -c "cmsis_dap_serial  $1" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666"  -c "init; reset halt; flash erase_address 0x10004000 0x40000; reset exit" &
    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid ; do
    sleep 1
    # we can add a timeout here if we want
    done   
}

function run_notConntectedTest()
{
    project_marker
    cd $PROJECT_NAME
    set +x
    echo "> Flashing $PROJECT_NAME"
    # project is prebuilt
    #make -j8 > /dev/null
    cd build/
    flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1
    cd $EXAMPLE_TEST_PATH/tests
    #runs desired test 
    python3 -m robot  -d $EXAMPLE_TEST_PATH/results -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 $EXAMPLE_TEST_PATH/tests/$PROJECT_NAME.robot
    # update failed test count
    let "numOfFailedTests+=$?"       
    #get back to target directory
    cd $MSDK_DIR/Examples/$TARGET_1_UC
}

function project_marker()
{
    printf "\r\n ━━━━━━━━━━━━━━━━━━━━━━━|Starting $PROJECT_NAME|━━━━━━━━━━━━━━━━━━━━━━━ \r\n\r\n"
}
#--------------------------------------------------------------------------------------------
#keep track of directory count, might need it
projIdx=1
numOfFailedTests=0
#no filter would support ALL projects not just ble
project_filter='BLE_'

# testing ME17 only for now 
cd Examples/$TARGET_1_UC
#used for all the not connected test
start_openocd_DAPLINK_1

#erase the helper device so that its not running an app that might connect 
#to current app under test
erase_with_openocd $CMSIS_DAP_ID_2

cd $MSDK_DIR/Examples/$TARGET_1_UC

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
            run_notConntectedTest 
            ;;

        "BLE_mcs" )
            #run_notConntectedTest 
            ;;

        "BLE_fit" )
            #run_notConntectedTest 
            ;;

        # "BLE_fcc" )
        #     # todo:
        #     # execute related test
        #     echo Found BLE_fcc #place holder
        #     ;;

        "BLE_FreeRTOS" )
            #run_notConntectedTest 
            ;;

        "BLE_otac" )
            #run_notConntectedTest 
            ;;

        "BLE_otas" )
            # gets tested during conencted test below
           
            ;;
            
        "BLE_periph" )
            # No buttons implemented for this example so lets just test if it builds
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

# Start connected-state tests
printf "\r\n ━━━━━━━━━━━━━━━━━━━━━━━|BLE_datc/dats Connection Test|━━━━━━━━━━━━━━━━━━━━━━━ \r\n\r\n"
#------ datc ME17
cd $MSDK_DIR/Examples/$TARGET_1_UC/BLE_datc/build
echo ">>     Flashing BLE_datc"
flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1

#------ dats on ME18
cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_dats
make -j8
cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_dats/build
echo "> Flashing BLE_dats"   
flash_with_openocd  $TARGET_2_LC.elf $CMSIS_DAP_ID_2

#give them time to connect
sleep 10 #give them time to connect
cd $MSDK_DIR/Examples_tests
#runs desired test 
python3 -m robot  -d $EXAMPLE_TEST_PATH/results -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 $EXAMPLE_TEST_PATH/tests/BLE_dat_cs.robot
let "numOfFailedTests+=$?" 

# printf "\r\n ━━━━━━━━━━━━━━━━━━━━━━━|BLE_Freertos Connection Test|━━━━━━━━━━━━━━━━━━━━━━━ \r\n\r\n"
# #------ datc ME17
# #device 1 should still have client app from previous test

# #------ otas on ME18
# cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_otas
# make -j8
# cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_otas/build
# echo "> Flashing BLE_FreeRTOS"   
# flash_with_openocd  $TARGET_2_LC.elf $CMSIS_DAP_ID_2

# #give them time to connect
# sleep 5 #give them time to connect
# #runs desired test 
# python3 -m robot  -d $EXAMPLE_TEST_PATH/results -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 $EXAMPLE_TEST_PATH/tests/BLE_FreeRTOS_cs.robot
# let "numOfFailedTests+=$?" 

#Not runing this till new OTAS gets merged
# printf "\r\n ━━━━━━━━━━━━━━━━━━━━━━━|BLE_otac/otas Connection Test|━━━━━━━━━━━━━━━━━━━━━━━ \r\n\r\n"
# #------ Flash BLE_otac onto Device 1  : ME17
# cd $MSDK_DIR/Examples/$TARGET_1_UC/BLE_otac/build
# echo ">>     Flashing BLE_otac"
# flash_with_openocd $TARGET_1_LC.elf $CMSIS_DAP_ID_1

# #------ Flash BLE_otas  onto Device 2  : ME18
# cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_otas
# make -j8
# cd $MSDK_DIR/Examples/$TARGET_2_UC/BLE_otas/build
# echo "> Flashing BLE_otas"   
# flash_with_openocd  $TARGET_2_LC.elf $CMSIS_DAP_ID_2

# #------ Flash Bootloader onto Device 2  : ME18
# cd $MSDK_DIR/Examples/$TARGET_2_UC/Bootloader
# make -j8
# cd $MSDK_DIR/Examples/$TARGET_2_UC/Bootloader/build
# echo "> Flashing BLE_otas"   
# flash_with_openocd  $TARGET_2_LC.elf $CMSIS_DAP_ID_2

# #runs desired test 
# python3 -m robot  -d $EXAMPLE_TEST_PATH/results -v VERBOSE:$VERBOSE_TEST -v SERIAL_PORT:$devSerial_1 $EXAMPLE_TEST_PATH/tests/BLE_ota_cs.robot
# let "numOfFailedTests+=$?" 

echo "------------------------------------------------------------------------"
echo "          >>>>>> Test completed with $numOfFailedTests failed tests<<<<<<<"
echo "-----------------------------------------------------------------------"
exit $numOfFailedTests