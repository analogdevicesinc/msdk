#!/usr/bin/env bash

echo
echo "##########################################################################################################################"
echo "# board_per_test.sh 1_MSDK 2_BRD1 3_BRD2 4_CURR_TIME 5_CURR_JOB_FILE 6_CURR_LOG 7_all_in_one 8PKG 9PHY 10STEP  11LIMIT   #"
echo "##########################################################################################################################"
echo
echo $0 $@
echo

if [[ $# -ne 11 ]]; then
    echo "Invalid call. Follow: board_per_test.sh 1_MSDK 2_BRD1 3_BRD2 4_CURR_TIME 5_CURR_JOB_FILE 6_CURR_LOG 7_all_in_one 8PKG 9PHY 10STEP 11LIMIT"
    exit 1
fi

MSDK=$1
BRD1=$2
BRD2=$3
CURR_TIME=$4
CURR_JOB_FILE=$5
CURR_LOG=$6
all_in_one=$7
PKG_RA=$8
PHY_RA=$9
STEP=${10}
LIMIT=${11}

echo "         MSDK: $MSDK"
echo "         BRD1: $BRD1"
echo "         BRD2: $BRD2"
echo "    CURR_TIME: $CURR_TIME"
echo "CURR_JOB_FILE: $CURR_JOB_FILE"
echo "     CURR_LOG: $CURR_LOG"
echo "   all_in_one: $all_in_one"
echo "       PKG_RA: $PKG_RA"
echo "       PHY_RA: $PHY_RA"
echo "         STEP: $STEP"
echo "        LIMIT: $LIMIT"
echo ""

#------------------------------------------------
# Set up the running environment
TMP_PATH=/tmp/msdk/ci/per
mkdir -p $TMP_PATH

echo "Use python 3.10.9."
source ~/anaconda3/etc/profile.d/conda.sh
conda activate py3_10
python3 -c "import sys; print(sys.version)"
echo ""

RS_FILE=~/Workspace/Resource_Share/boards_config.json
echo "The board info are stored in ${RS_FILE}."
echo

BRD1_DAP_SN=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD1}']['DAP_sn'])")
echo BRD1_DAP_SN: ${BRD1_DAP_SN}
BRD1_HCI=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD1}']['hci_id'])")
echo BRD1_HCI: ${BRD1_HCI}
BRD1_SW_MODEL=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD1}']['sw_model'])")
echo BRD1_SW_MODEL: ${BRD1_SW_MODEL}
BRD1_SW_ST=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD1}']['sw_state'])")
echo BRD1_SW_ST: ${BRD1_SW_ST}
echo

BRD2_CHIP_LC=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['chip_lc'])")
echo BRD2_CHIP_LC: ${BRD2_CHIP_LC}
BRD2_CHIP_UC=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['chip_uc'])")
echo BRD2_CHIP_UC: ${BRD2_CHIP_UC}
BRD2_TYPE=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['type'])")
echo BRD2_TYPE: ${BRD2_TYPE}
BRD2_DAP_SN=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['DAP_sn'])")
echo BRD2_DAP_SN: ${BRD2_DAP_SN}
BRD2_DAP_ID=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['DAP_id'])")
echo BRD2_DAP_ID: ${BRD2_DAP_ID}
BRD2_HCI=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['hci_id'])")
echo BRD2_HCI: ${BRD2_HCI}
BRD2_CON=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['con_id'])")
echo BRD2_CON: ${BRD2_CON}
BRD2_SW_MODEL=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['sw_model'])")
echo BRD2_SW_MODEL: ${BRD2_SW_MODEL}
BRD2_SW_ST=$(python3 -c "import json; import os; obj=json.load(open('${RS_FILE}')); print(obj['${BRD2}']['sw_state'])")
echo BRD2_SW_ST: ${BRD2_SW_ST}
echo

echo "#--------------------------------------------------------------------------------------------"
echo "# PER test on board ${BRD2} ${BRD2_TYPE}"
echo "#--------------------------------------------------------------------------------------------"
echo

echo "Try to lock the hardware resources."
python3 ~/Workspace/Resource_Share/Resource_Share.py -l -t 3600 /home/$USER/Workspace/Resource_Share/mc_rf_sw.txt
python3 ~/Workspace/Resource_Share/Resource_Share.py -l -t 3600 /home/$USER/Workspace/Resource_Share/${BRD1}.txt
python3 ~/Workspace/Resource_Share/Resource_Share.py -l -t 3600 /home/$USER/Workspace/Resource_Share/${BRD2}.txt
echo ""

echo CURR_JOB_FILE: ${CURR_JOB_FILE}
touch ${CURR_JOB_FILE}
echo CURR_LOG: ${CURR_LOG}
touch ${CURR_LOG}
echo

echo /home/$USER/Workspace/Resource_Share/${BRD2}.txt  >> ${CURR_JOB_FILE}
echo /home/$USER/Workspace/Resource_Share/${BRD1}.txt  >> ${CURR_JOB_FILE}
echo /home/$USER/Workspace/Resource_Share/mc_rf_sw.txt >> ${CURR_JOB_FILE}

#------------------------------------------------
echo "Disable UART assertion."
cd ${MSDK}
echo PWD: `pwd`
echo

set +e
sed -i "s/ PAL_SYS_ASSERT(result3 == 0)/ \/\/PAL_SYS_ASSERT(result3 == 0)/g" Libraries/Cordio/platform/targets/maxim/max32655/sources/pal_uart.c || true
cat Libraries/Cordio/platform/targets/maxim/max32655/sources/pal_uart.c | grep PAL_SYS_ASSERT\(result[0-3]

host_name=`hostname`
if [ "${host_name}" == "wall-e" ]; then
    echo "#--------------------------------------------------------------------------------------------"
    echo "Set the Mini-circuits RF Switches."
    set -x
    echo RF switch for ${BRD1}
    unbuffer python3 $MSDK/Tools/Bluetooth/mc_rf_sw.py --model ${BRD1_SW_MODEL} --op set --state ${BRD1_SW_ST}
    echo ""
    echo RF switch for ${BRD2}
    unbuffer python3 $MSDK/Tools/Bluetooth/mc_rf_sw.py --model ${BRD2_SW_MODEL} --op set --state ${BRD2_SW_ST}
    set +x
    echo ""
fi

echo "#--------------------------------------------------------------------------------------------"
echo "Build the project BLE5_ctr for the 2nd board ${BRD2}"
echo

bash -e $MSDK/.github/workflows/scripts/build_flash.sh \
    ${MSDK} \
    /home/$USER/Tools/openocd \
    ${BRD2_CHIP_UC} \
    ${BRD2_TYPE} \
    BLE5_ctr \
    ${BRD2_DAP_SN} \
    True \
    False
set +e
set +x
echo

echo "#--------------------------------------------------------------------------------------------"
echo "Test in different packet length, PHY, attenuation, and txPower"

i=0
echo "packetLen,phy,atten,txPower,perMaster,perSlave" > "${all_in_one}"
step=${STEP}
echo ""

SH_RESET_BRD1=$TMP_PATH/${CURR_TIME}_brd1_reset.sh
echo $SH_RESET_BRD1
echo "#!/usr/bin/env bash" > $SH_RESET_BRD1
echo "nrfjprog --family nrf52 -s ${BRD1_DAP_SN} --debugreset" >> $SH_RESET_BRD1
chmod u+x $SH_RESET_BRD1
cat $SH_RESET_BRD1
echo ""

SH_RESET_BRD2=$TMP_PATH/${CURR_TIME}_brd2_reset.sh
echo $SH_RESET_BRD2
echo "#!/usr/bin/env bash" > $SH_RESET_BRD2
#echo "bash -ex $MSDK/.github/workflows/scripts/hard_reset.sh ${BRD2_CHIP_LC}.cfg ${BRD2_DAP_SN} $(realpath ${MSDK}/Examples/${BRD2_CHIP_UC}/BLE5_ctr/build/${BRD2_CHIP_LC}.elf) 2>&1 | tee test.log" >> $SH_RESET_BRD2
echo "bash -e $MSDK/.github/workflows/scripts/build_flash.sh ${MSDK} /home/$USER/Tools/openocd ${BRD2_CHIP_UC} ${BRD2_TYPE} BLE5_ctr ${BRD2_DAP_SN} False True 2>&1 | tee ${TMP_PATH}/test.log" >> $SH_RESET_BRD2
echo "sleep 10" >> $SH_RESET_BRD2
chmod u+x $SH_RESET_BRD2
cat $SH_RESET_BRD2

for pkt_len in ${PKG_RA}
do
    for phy in ${PHY_RA}
    do
        echo "---------------------------------------------------------------------------------------------"
        echo "Next turn: pkt_len ${pkt_len}, phy ${phy}"
        echo

        echo "Program or reset the board ${BRD2}."
        if [[ $i -eq 0 ]]; then
            echo "Flash the board."            
            bash -e $MSDK/.github/workflows/scripts/build_flash.sh \
                ${MSDK} \
                /home/$USER/Tools/openocd \
                ${BRD2_CHIP_UC} \
                ${BRD2_TYPE} \
                BLE5_ctr \
                ${BRD2_DAP_SN} \
                False \
                True
        else
            echo "Reset the board ${BRD1}"
            bash -ex $SH_RESET_BRD1
            echo

            echo "Hard reset the board ${BRD2}."
            bash -ex $MSDK/.github/workflows/scripts/hard_reset.sh ${BRD2_CHIP_LC}.cfg ${BRD2_DAP_SN} $(realpath ${MSDK}/Examples/${BRD2_CHIP_UC}/BLE5_ctr/build/${BRD2_CHIP_LC}.elf)
            set +e
            set +x
            echo
        fi

        echo
        echo "Sleep 5 secs."
        sleep 5
        echo "Continue the test."
        echo

        # Run the PER test
        RESULT_PATH=~/Workspace/ci_results/per
        res=${RESULT_PATH}/msdk-${CURR_TIME}
        res_files[i]=${res}_${BRD2_CHIP_LC}_${BRD2_TYPE}_${i}.csv
        echo "The test results will be saved in file ${res_files[i]}."

        slv_ser=${BRD2_HCI}
        mst_ser=${BRD1_HCI}
        
        set -x
        unbuffer python3 $MSDK/Tools/Bluetooth/conn_sweep.py ${slv_ser} ${mst_ser} ${res_files[i]} \
            --stp ${BRD2_CON} --pktlen ${pkt_len} --phys ${phy} --step ${step} --loss -15.7 \
            --brd1_reset $SH_RESET_BRD1 --brd2_reset $SH_RESET_BRD2
        set +x

        echo "cat ${res_files[i]}"
        cat "${res_files[i]}"

        cat "${res_files[i]}" >> "${all_in_one}"  # put all results into one file

        i=$((i+1))
    done
done

# Reset the boards to end the TX
python3 $MSDK/Tools/Bluetooth/BLE_hci.py ${BRD2_HCI} -c "reset; exit"
python3 $MSDK/Tools/Bluetooth/BLE_hci.py ${BRD1_HCI} -c "reset; exit"

echo
echo $(date)
echo "Started at ${start_time}"
end_secs=$(date +%s)
exe_time=$((end_secs - start_secs))
echo

echo "${BRD2_CHIP_UC} ${BRD2_TYPE} test is completed."

rm $SH_RESET_BRD1
rm $SH_RESET_BRD2

echo "#--------------------------------------------------------------------------------------------"
echo "cat ${all_in_one}"
cat "${all_in_one}"
echo ""
echo "Check the PER values."
echo "unbuffer python3 ${MSDK}/.github/workflows/scripts/check_results.py --csv $(realpath ${all_in_one}) --debug --limit ${LIMIT}"
echo ""
unbuffer python3 ${MSDK}/.github/workflows/scripts/check_results.py --csv $(realpath ${all_in_one}) --debug --limit ${LIMIT}

if [[ $? -ne 0 ]]; then
    echo
    echo "#-----------------------------"
    echo "# Check failed!"
    echo "#-----------------------------"
    echo
    exit 1
fi
