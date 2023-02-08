#!/usr/bin/env bash

echo
echo "#############################################################################################"
echo "# ./run_1_brd_robot.sh msdk_path board robot_test                                          #"
echo "#############################################################################################"
echo

# Example
# ./run_1_brd_robot.sh ~/Workspace/msdk_open max32690_board_A3 BLE_datc

echo $0 $@
echo

ERR_1=1

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

    DUT_BRD=$2
    echo DUT_BRD=$DUT_BRD

    DUT_TGT_LOWER=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${DUT_BRD}']['target_lower'])"`
    DUT_TGT_UPPER=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${DUT_BRD}']['target_upper'])"`
    echo DUT_TGT_LOWER=$DUT_TGT_LOWER
    echo DUT_TGT_UPPER=$DUT_TGT_UPPER

    DUT_BRD_TYPE=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${DUT_BRD}']['type'])"`
    if [[ "$DUT_BRD_TYPE" != "EvKit_V1" && "$DUT_BRD_TYPE" != "WLP_V1" ]]; then
        echo "Error: invalid board type (should be EvKit_V1 or WLP_V1)"
        exit 2
    fi
    echo DUT_BRD_TYPE=$DUT_BRD_TYPE

    DUT_DAP_SN=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${DUT_BRD}']['DAP_sn'])"`
    echo DUT_DAP_SN=$DUT_DAP_SN

    sn=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${DUT_BRD}']['con_sn'])"`
    DUT_SERIAL_PORT=/dev/$(ls -la /dev/serial/by-id | grep -n $sn | rev | cut -d "/" -f1 | rev)
    echo DUT_SERIAL_PORT=$DUT_SERIAL_PORT

    ROBOT_TEST=$3    
    echo ROBOT_TEST=$ROBOT_TEST
    echo
}


###################################################################################################
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


#--------------------------------------------------------------------------------------------------
# MAIN CODE
initial_setup $@

reset_board_by_openocd $DUT_TGT_LOWER $DUT_DAP_SN

RESULT_PATH=$EXAMPLE_TEST_PATH/results/$DUT_TGT_UPPER/$ROBOT_TEST
mkdir -p $RESULT_PATH

cd $MSDK/.github/workflows/ci-tests/Examples_tests/tests
echo PWD:`pwd`
echo "$ROBOT -d $RESULT_PATH -v SERIAL_PORT_1:$DUT_SERIAL_PORT $ROBOT_TEST.robot"
echo
$ROBOT -d $EXAMPLE_TEST_PATH/results/$DUT_NAME_UPPER/$PROJECT_NAME -v SERIAL_PORT_1:$DUT_SERIAL_PORT $ROBOT_TEST.robot


