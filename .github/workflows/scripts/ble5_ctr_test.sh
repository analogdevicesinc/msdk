#!/usr/bin/env bash

#
# Example:
#     ./ble5_ctr_test.sh ~/Workspace/yc/msdk_open MAX32655 EvKit_V1 `date +%Y-%m-%d_%H-%M-%S`
#
FORMAT="./ble5_ctr_test.sh 1_MSDK 2_CHIP_UC 3_BOARD_TYPE 4_TEST_TIME"
printf "\n\n############################################################################################"
printf "\n# ${FORMAT}"
printf "\n############################################################################################\n\n"

echo $0 $@
echo

if [ $# -ne 4 ]; then
    printf "Invalid command. Follow the format: \"${FORMAT}\"\n\n"
    exit 1
fi

MSDK=$1
CHIP_UC=$2
BRD_TYPE=$3
TEST_TIME=$4

source ~/anaconda3/etc/profile.d/conda.sh
conda activate py3_10

RES_FOLDER=/tmp/ci_test/ble5_ctr
LOCK_FILE=${RES_FOLDER}/${TEST_TIME}.lock  # must match ble5_ctr_test.yml
mkdir -p ${RES_FOLDER}

#--------------------------------------------------------------------------------------------------
function cleanup {
    set +x
    printf "\n<<<<<<< cleanup before exit <<<<<<\n"
    if [ -f $LOCK_FILE ]; then
        echo "cat $LOCK_FILE"
	cat $LOCK_FILE
	echo
        set -x
        bash $LOCK_FILE
        set +x
    fi
    printf "\n<<<<<< EXIT <<<<<<\n\n"
}
#--------------------------------------------------------------------------------------------------

trap cleanup EXIT
trap cleanup INT

#--------------------------------------------------------------------------------------------------
FILE=/home/$USER/Workspace/ci_config/boards_config.json
TEST_CONFIG_FILE=/home/$USER/Workspace/ci_config/ble5_ctr_test.json
CI_TEST=ble5_ctr_test.yml

HOST_NAME=`hostname`
# skip FCC(file change check) or not
SKIP_FCC=`python3 -c "import json; import os; obj=json.load(open('${TEST_CONFIG_FILE}')); print(obj['${CI_TEST}']['${HOST_NAME}']['SKIP_FCC'])"`
printf "\nSKIP_FCC: ${SKIP_FCC}\n"

MSDK_COMMIT=`python3 -c "import json; import os; obj=json.load(open('${TEST_CONFIG_FILE}')); print(obj['${CI_TEST}']['${HOST_NAME}']['msdk_commit'])"`
printf "\nMSDK_COMMIT: ${MSDK_COMMIT}\n\n"

if [ "x$MSDK_COMMIT" != "x" ]; then
    # need to switch to required version
    set -x
    cd $MSDK
    git branch
    echo

    git status -u
    echo

    git checkout -- .
    echo

    git fetch
    echo

    git checkout $MSDK_COMMIT
    echo

    set +x
fi

cd $MSDK
echo "PWD: "`pwd`
echo

#--------------------------------------------------------------------------------------------------
CHIP_LC=${CHIP_UC,,}
BRD_TYPE_LC=${BRD_TYPE,,}

CHIP_BRD_TYPE=${CHIP_LC}_${BRD_TYPE_LC}

printf "\n#====================================================================================\n"
printf "# Test on ${CHIP_BRD_TYPE}"
printf "\n#====================================================================================\n"

# check the configuration
DO_THIS=`python3 -c "import json; import os; obj=json.load(open('${TEST_CONFIG_FILE}')); print(obj['${CI_TEST}']['${HOST_NAME}']['${CHIP_BRD_TYPE}']['do_this'])"`
printf "           DO_THIS: ${DO_THIS}\n\n"

if [ ${DO_THIS} == "0" ]; then
    printf "\n#----------------------------------------------------------------------"
    printf "\n# Skip this ${CHIP_BRD_TYPE} board according to the configuration file."
    printf "\n#----------------------------------------------------------------------\n"
    continue
fi

BRD2=`python3 -c "import sys, json; print(json.load(open('$TEST_CONFIG_FILE'))['${CI_TEST}']['$HOST_NAME']['$CHIP_BRD_TYPE']['board2'])"`
echo "              BRD2: ${BRD2}"
echo

BRD2_CHIP_UC=`python3 -c "import json; import os; obj=json.load(open('${FILE}')); print(obj['${BRD2}']['chip_uc'])"`
BRD2_TYPE=`python3 -c "import json; import os; obj=json.load(open('${FILE}')); print(obj['${BRD2}']['type'])"`
BRD2_DAP_SN=`python3 -c "import json; import os; obj=json.load(open('${FILE}')); print(obj['${BRD2}']['DAP_sn'])"`

jtag_sn_2=`/usr/bin/python3 -c "import sys, json; print(json.load(open('$FILE'))['${BRD2}']['DAP_sn'])"`

# get the lock file for each board
BRD2_LOCK=`python3 -c "import json; import os; obj=json.load(open('${FILE}')); print(obj['${BRD2}']['lockfile'])"`
echo "    BRD2 lock file: $BRD2_LOCK"
echo 

echo "         jtag_sn_2: $jtag_sn_2"
echo

if [[ $BRD2 =~ "nRF" ]]; then
    echo "Board 2 is a nrf52840 board."
else
    echo "Board 2 is not a nRF52840 board."
fi

printf "\nTry to lock the files...\n"    
python3 ~/Workspace/Resource_Share/Resource_Share.py -l -t 3600 ${BRD2_LOCK}
if [ $? -ne 0 ]; then
    printf "\nFail to acquire the resources.\n"
    exit FAIL_TO_ACQUIRE_BOARDS
fi

touch $LOCK_FILE
echo "python3 ~/Workspace/Resource_Share/Resource_Share.py ${BRD2_LOCK}" >> $LOCK_FILE
bash -x -c "cat $LOCK_FILE"
echo

SH_RESET_BRD2=/tmp/ci_test/timing/${TEST_TIME}_brd2_reset.sh

if [[ $BRD2 =~ "nRF" ]]; then
    printf "\n<<<<<< reset board 2: nRF board\n\n"
    set -x
    nrfjprog --family nrf52 -s ${BRD2_DAP_SN} --debugreset
    set +x

    echo $SH_RESET_BRD2
    echo "#!/usr/bin/env bash" > $SH_RESET_BRD2
    echo "nrfjprog --family nrf52 -s ${BRD2_DAP_SN} --debugreset" >> $SH_RESET_BRD2
    chmod u+x $SH_RESET_BRD2
    cat $SH_RESET_BRD2
else
    printf "\n<<<<<< build and flash board 2: ${BRD2}\n\n"
    echo
    set -x
    bash -e $MSDK/.github/workflows/scripts/build_flash.sh \
        ${MSDK}                     \
        /home/$USER/Tools/openocd   \
        ${BRD2_CHIP_UC}             \
        ${BRD2_TYPE}                \
        BLE5_ctr                    \
        ${BRD2_DAP_SN}              \
        True                        \
        True
    set +x

    echo $SH_RESET_BRD2
    echo "#!/usr/bin/env bash" > $SH_RESET_BRD2
    echo "bash -e $MSDK/.github/workflows/scripts/build_flash.sh ${MSDK} /home/$USER/Tools/openocd ${BRD2_CHIP_UC} ${BRD2_TYPE} BLE5_ctr ${BRD2_DAP_SN} False True" >> $SH_RESET_BRD2
    echo "sleep 10" >> $SH_RESET_BRD2
    chmod u+x $SH_RESET_BRD2
    cat $SH_RESET_BRD2
    echo
fi

set -x
unbuffer python3 $MSDK/.github/workflows/scripts/ble5_ctr_test.py \
    --msdk $MSDK        \
    --chip $CHIP_UC     \
    --type $BRD2_TYPE   \
    --time $TEST_TIME
RES=$?
set +x

# release locked boards
python3 ~/Workspace/Resource_Share/Resource_Share.py ${BRD2_LOCK}

if [[ $RES -eq 0 ]]; then
    echo "exit 0"
    exit 0
else
    echo "exit 1"
    exit 1
fi
