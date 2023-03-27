#!/bin/bash
#script accepts 2 arguments: device , device key in boards config

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
if [ $(hostname) == "wall-e" ]; then
    FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
    # WALL-E  paths
    export OPENOCD_TCL_PATH=/home/btm-ci/Tools/openocd/tcl
    export OPENOCD=/home/btm-ci/Tools/openocd/src/openocd
elif [ $(hostname) == "yingcai-OptiPlex-790" ]; then
    FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
    export OPENOCD_TCL_PATH=/home/$USER/Tools/openocd/tcl
    export OPENOCD=/home/$USER/Tools/openocd/src/openocd
else
    FILE=/home/$USER/boards_config.json
    export OPENOCD_TCL_PATH=/home/eddie/workspace/openocd/tcl
    export OPENOCD=/home/eddie/workspace/openocd/src/openocd
fi
dut_serial=$(/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['$2']['daplink'])")
erase_with_openocd $1 $dut_serial

