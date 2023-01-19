#!/usr/bin/env bash

echo
echo "#####################################################################################################################"
echo "# This script is used to test the BLE_Examples_Test.yml locally at wall-e.                                          #"
echo "# local_BLE_Examples_Test.sh <0/1 for max32655> <0/1 for max32665> <0/1 for max32690 devkit> <0/1 for max32690 WLP> #"
echo "#####################################################################################################################"

echo
echo $0 $@

if [[ $# -ne 4 ]]; then
    echo
    echo "Invalid arguments."
    echo "local_BLE_Examples_Test.sh <0/1 for max32655> <0/1 for max32665> <0/1 for max32690 devkit> <0/1 for max32690 WLP>"
    exit 1
fi

host=`hostname`
if [ $host == "wall-e" ]; then
    MSDK=/home/$USER/Workspace/yc/msdk_open
else
    MSDK=/home/$USER/Workspace/msdk_open
fi

echo
echo "MSDK=$MSDK"

DO_MAX32655=$1
DO_MAX32665=$2
DO_MAX32690_EVKIT=$3
DO_MAX32690_WLP=$4

#--------------------------------------------------------------------------------------------------
# For MAX32655
if [[ $DO_MAX32655 -eq 1 ]]; then
    echo
    echo "-----------------------------------------------------------------------------------------"
    echo "TEST MAX32655"

    echo 
    echo "Lock the used recourse files."
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_a5.txt

    touch /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_EvKit.txt

    set -e

    cd $MSDK
    echo "PWD="`pwd`

    cd .github/workflows/ci-tests/Examples_tests
    chmod +x test_launcher.sh
    FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
    if [ $(hostname) == "wall-e" ]; then
        dut_uart=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board2']['uart0'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board2']['daplink'])"`
    else
        dut_uart=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board_y2']['uart0'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32655_board_y2']['daplink'])"`
    fi
    ./test_launcher.sh max32655 $dut_uart $dut_serial
 
    echo 
    echo "Release the used resource files."
    echo
    ls -hal ~/Workspace/Resource_Share

    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_a5.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_1.txt

    echo 
    echo "-----------------------------------------------------------------------------------------"
    echo "FINISH TEST MAX32655"
fi

#--------------------------------------------------------------------------------------------------
# For MAX32665
if [[ $DO_MAX32665 -eq 1 ]]; then
    echo
    echo "-----------------------------------------------------------------------------------------"
    echo "TEST MAX32665"

    echo "Lock the used recourse files."
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_a5.txt

    touch /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32665.txt

    set +e

    cd $MSDK
    echo "PWD="`pwd`

    cd .github/workflows/ci-tests/Examples_tests
    chmod +x test_launcher.sh
    FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
    if [ $(hostname) == "wall-e" ]; then
        dut_uart=`  /usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32665_board1']['uart1'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32665_board1']['daplink'])"`            
    else
        dut_uart=`  /usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32665_board_2']['uart1'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32665_board_2']['daplink'])"`            
    fi

    ./test_launcher.sh max32665 $dut_uart $dut_serial
 
    set -e
    
    echo 
    echo "Release the used resource files."
    echo
    ls -hal ~/Workspace/Resource_Share
    
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_a5.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_1.txt

    echo 
    echo "-----------------------------------------------------------------------------------------"
    echo "FINISH TEST MAX32655"
fi

#--------------------------------------------------------------------------------------------------
# For MAX32690_EVKIT
if [[ $DO_MAX32690_EVKIT -eq 1 ]]; then
    echo
    echo "-----------------------------------------------------------------------------------------"
    echo "TEST MAX32690 EVKIT"
    echo 

    echo "Lock the used recourse files."
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_a5.txt

    touch /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_EvKit.txt

    set +e

    cd $MSDK
    echo "PWD="`pwd`

    cd .github/workflows/ci-tests/Examples_tests
    chmod +x test_launcher.sh
    FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
    if [ $(hostname) == "wall-e" ]; then
        dut_uart=`  /usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_w1']['uart2'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_w1']['daplink'])"`            
    else
        dut_uart=`  /usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_3']['uart2'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_3']['daplink'])"`            
    fi

    ./test_launcher.sh max32690 $dut_uart $dut_serial
 
    set -e

    echo 
    echo "Release the used resource files."
    echo
    ls -hal ~/Workspace/Resource_Share
    
    if [ -f /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_EvKit.txt ]; then
        rm /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_EvKit.txt
    fi
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_a5.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_1.txt

    echo 
    echo "-----------------------------------------------------------------------------------------"
    echo "FINISH TEST MAX32690 EVKIT"
fi


#--------------------------------------------------------------------------------------------------
# For MAX32690_WLP
if [[ $DO_MAX32690_WLP -eq 1 ]]; then
    echo
    echo "-----------------------------------------------------------------------------------------"
    echo "TEST MAX32690 WLP"
    echo

    echo "Lock the used recourse files."    
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 1800 /home/$USER/Workspace/Resource_Share/max32690_a5.txt

    touch /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_WLP.txt # indicate which jos is running
    set +e

    cd $MSDK
    echo "PWD="`pwd`

    cd .github/workflows/ci-tests/Examples_tests
    chmod +x test_launcher.sh
    FILE=/home/$USER/Workspace/Resource_Share/boards_config.json
    
    if [ $host == "wall-e" ]; then
        dut_uart=`  /usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_A5']['con_sn'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_A5']['DAP_sn'])"`            
    else
        dut_uart=`  /usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_A3']['con_sn'])"`
        dut_serial=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['max32690_board_A3']['DAP_sn'])"`            
    fi

    ./test_launcher.sh max32690 $dut_uart $dut_serial WLP_V1
 
    set -e

    echo 
    echo "Release the used resource files."
    echo
    ls -hal ~/Workspace/Resource_Share
    
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_a5.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32690_w1.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32665_13.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_2.txt
    python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_1.txt

    if [ -f /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_WLP.txt ]; then
        rm /home/$USER/Workspace/Resource_Share/local-BLE_Examples_Test-MAX32690_WLP.txt
    fi

    echo 
    echo "-----------------------------------------------------------------------------------------"
    echo "FINISH TEST MAX32690 WLP"
fi
