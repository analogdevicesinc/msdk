#!/usr/bin/env bash

echo
echo "#############################################################################################"
echo "# ./run_2_brd_robot.sh msdk_path board1 project1 board2 project2 robot_test                 #"
echo "#############################################################################################"
echo

# Example
# ./run_2_brd_robot.sh ~/Workspace/msdk_open max32655_board_y1 BLE_datc max32690_board_A3 BLE_dats BLE_dat_cs

echo $0 $@
echo

###################################################################################################
function initial_setup()
{
    echo "function: ${FUNCNAME[0]} $@"
    echo

    if [ $(hostname) == "wall-e" ]; then
        echo "On machine wall-e"
        echo

        FILE=/home/$USER/Workspace/Resource_Share/boards_config.json

        export OPENOCD=/home/btm-ci/Tools/openocd/src/openocd
        export OPENOCD_TCL_PATH=/home/btm-ci/Tools/openocd/tcl
        export ROBOT=/home/btm-ci/.local/bin/robot

    elif [ $(hostname) == "yingcai-OptiPlex-790" ]; then
        echo "On machine yingcai-OptiPlex-790"
        echo

        FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
        
        export OPENOCD=/home/$USER/Tools/openocd/src/openocd
        export OPENOCD_TCL_PATH=/home/$USER/Tools/openocd/tcl
        export ROBOT=/home/$USER/.local/bin/robot

    else
        # Local- eddie desktop
        FILE=/home/$USER/boards_config.json

        export OPENOCD=/home/eddie/workspace/openocd/src/openocd
        export OPENOCD_TCL_PATH=/home/eddie/workspace/openocd/tcl
        export ROBOT=/home/eddie/.local/bin/robot
    fi
    
    MSDK=$1
    if [ ! -d $MSDK ]; then
        echo "Error: invalid MSDK path."
        exit 1
    fi
    echo MSDK=$MSDK
    EXAMPLE_TEST_PATH=$MSDK/.github/workflows/ci-tests/Examples_tests
    echo EXAMPLE_TEST_PATH=$EXAMPLE_TEST_PATH
    echo

    # C (client)
    # S (server): the DUT board
    C_BRD=$2
    echo C_BRD: $C_BRD

    C_TGT_LOWER=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${C_BRD}']['target_lower'])"`
    C_TGT_UPPER=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${C_BRD}']['target_upper'])"`
    echo C_TGT_LOWER: $C_TGT_LOWER
    echo C_TGT_UPPER: $C_TGT_UPPER

    C_BRD_TYPE=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${C_BRD}']['type'])"`
    if [[ "$C_BRD_TYPE" != "EvKit_V1" && "$C_BRD_TYPE" != "WLP_V1" ]]; then
        echo "Error: invalid client board type (should be EvKit_V1 or WLP_V1)"
        exit 2
    fi
    echo C_BRD_TYPE: $C_BRD_TYPE

    C_PRJ=$3
    echo C_PRJ: $C_PRJ

    #--------------------------------------------
    S_BRD=$4
    echo S_BRD: $S_BRD

    S_TGT_LOWER=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${S_BRD}']['target_lower'])"`
    S_TGT_UPPER=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${S_BRD}']['target_upper'])"`
    echo S_TGT_LOWER: $S_TGT_LOWER
    echo S_TGT_UPPER: $S_TGT_UPPER

    S_BRD_TYPE=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${S_BRD}']['type'])"`
    if [[ "$S_BRD_TYPE" != "EvKit_V1" && "$S_BRD_TYPE" != "WLP_V1" ]]; then
        echo "Error: invalid server board type (should be EvKit_V1 or WLP_V1)"
        exit 3
    fi
    echo S_BRD_TYPE: $S_BRD_TYPE

    S_PRJ=$5
    echo S_PRJ: $S_PRJ

    #--------------------------------------------
    C_DAP_SN=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${C_BRD}']['DAP_sn'])"`
    echo C_DAP_SN: $C_DAP_SN

    sn=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${C_BRD}']['con_sn'])"`
    C_CON=/dev/$(ls -la /dev/serial/by-id | grep -n $sn | rev | cut -d "/" -f1 | rev)
    echo C_CON: $C_CON

    S_DAP_SN=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${S_BRD}']['DAP_sn'])"`
    echo S_DAP_SN: $S_DAP_SN

    sn=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${S_BRD}']['con_sn'])"`
    S_CON=/dev/$(ls -la /dev/serial/by-id | grep -n $sn | rev | cut -d "/" -f1 | rev)
    echo S_CON: $S_CON

    ROBOT_TEST=$6
    echo ROBOT_TEST=$ROBOT_TEST
    echo
}


###################################################################################################
# params: target in lower case, DAP sn
function reset_board_by_openocd() {
    echo
    echo "function: ${FUNCNAME[0]} $@"

    set +e
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg \
             -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH \
             -c "adapter serial $2" \
             -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" \
             -c "init; reset run" >/dev/null &

    openocd_dapLink_pid=$!

    sleep 1
    if ps -p $openocd_dapLink_pid >/dev/null; then
        kill -9 $openocd_dapLink_pid || true
    fi
    set -e

    echo
    echo "Reset done!"
    echo
    sleep 1
}


###################################################################################################
# params: device, CMSIS-DAP serial
function flash_with_openocd() {
    echo "function: ${FUNCNAME[0]} $@"

    set +e

    # mass erase and flash    
    $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH \
             -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" \
             -c "init; reset halt; max32xxx mass_erase 0" \
             -c "program $1.elf verify reset exit" >/dev/null &

    openocd_dapLink_pid=$!
    # wait for openocd to finish
    while kill -0 $openocd_dapLink_pid; do
        sleep 1
        # we can add a timeout here if we want
    done

    set -e

    # Check the return value to see if we received an error
    if [ "$?" -ne "0" ]; then
        printf "\r\n> Verify failed , flashibng again \r\n"
        # Reprogram the device if the verify failed
        $OPENOCD -f $OPENOCD_TCL_PATH/interface/cmsis-dap.cfg -f $OPENOCD_TCL_PATH/target/$1.cfg -s $OPENOCD_TCL_PATH -c "cmsis_dap_serial  $2" -c "gdb_port 3333" -c "telnet_port 4444" -c "tcl_port 6666" -c "init; reset halt;max32xxx mass_erase 0" -c "program $1.elf verify reset exit" >/dev/null &
        openocd_dapLink_pid=$!
    fi
}


#--------------------------------------------------------------------------------------------------
# MAIN CODE
initial_setup $@

#------------------------------------------------
echo "Build and flash the client board."
echo

cd $MSDK/Examples/$C_TGT_UPPER/$C_PRJ
echo pwd=`pwd`
echo

make distclean
echo

echo "make -j8 BOARD=$C_BRD_TYPE"
echo

make -j8 BOARD=$C_BRD_TYPE

echo "Flash client first because it takes longer."
echo

cd build
flash_with_openocd $C_TGT_LOWER $C_DAP_SN

#------------------------------------------------
echo
echo "Build and flash the server board (DUT board)."
echo

cd $MSDK/Examples/$S_TGT_UPPER/$S_PRJ
echo pwd=`pwd`
echo

make distclean
echo

echo "make -j8 BOARD=$S_BRD_TYPE"
echo

make -j8 BOARD=$S_BRD_TYPE

echo
echo "Flash server."
echo

cd build
flash_with_openocd $S_TGT_LOWER $S_DAP_SN

#------------------------------------------------
reset_board_by_openocd $C_TGT_LOWER $C_DAP_SN
reset_board_by_openocd $S_TGT_LOWER $S_DAP_SN

#------------------------------------------------
RESULT_PATH=$EXAMPLE_TEST_PATH/results/$S_TGT_UPPER/$ROBOT_TEST
mkdir -p $RESULT_PATH

cd $MSDK/.github/workflows/ci-tests/Examples_tests/tests
echo
echo PWD:`pwd`
echo
echo "$ROBOT -d $RESULT_PATH -v SERIAL_PORT_1:$C_CON -v SERIAL_PORT_2:$S_CON $ROBOT_TEST.robot"
echo
$ROBOT -d $RESULT_PATH -v SERIAL_PORT_1:$C_CON -v SERIAL_PORT_2:$S_CON $ROBOT_TEST.robot

let "testResult=$?"
if [ "$testResult" -ne "0" ]; then
    # update failed test count
    let "numOfFailedTests+=$testResult"

    failedTestList+="| BLE_dat_cs ($S_TGT_UPPER) "
fi
set -e

