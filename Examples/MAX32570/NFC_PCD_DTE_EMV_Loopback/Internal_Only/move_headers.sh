#!/bin/sh

SRC_PATH=".."

RF_LIB_PATH="../../../../Libraries/NFC/lib_nfc_pcd_rf_driver_MAX32570"
PBM_LIB_PATH="../../../../Libraries/NFC/lib_nfc_pcd_pbm"
DEMO_PATH="../MAX32570-Demo"

RF_SUB_PATH="include/nfc"
PBM_SUB_PATH="include"
EMV_SUB_PATH="emv_l1_stack"
POLL_SUB_PATH="contactless_l1_app"

RF_HEADER_NAME="mml_nfc_pcd_rf_driver.h"
PORT_HEADER_NAME="mml_nfc_pcd_port.h"
EMV_HEADER_NAME="iso14443_3_common.h"
PBM_HEADER_NAME="pbm_commands.h"
POLL_HEADER_NAME="EMV_polling_and_loopback.h"
TIMER_HEADER_NAME="mml_nfc_pcd_timer_utils.h"

SRC_PATH_RF=$SRC_PATH/$RF_SUB_PATH
DST_PATH_RF=$RF_LIB_PATH/$RF_SUB_PATH
DST_PATH_RF_PBM=$PBM_LIB_PATH/$RF_SUB_PATH
DST_PATH_RF_DEMO=$DEMO_PATH/$RF_SUB_PATH

SRC_PATH_EMV=$SRC_PATH/$RF_SUB_PATH/$EMV_SUB_PATH
DST_PATH_EMV_PBM=$PBM_LIB_PATH/$PBM_SUB_PATH/$EMV_SUB_PATH
DST_PATH_EMV_DEMO=$DEMO_PATH/$RF_SUB_PATH/$EMV_SUB_PATH

SRC_PATH_PBM=$SRC_PATH/$RF_SUB_PATH/pbm
DST_PATH_PBM=$PBM_LIB_PATH/$PBM_SUB_PATH

SRC_PATH_POLL=$SRC_PATH/$RF_SUB_PATH/$POLL_SUB_PATH
DST_PATH_POLL_DEMO=$DEMO_PATH/$RF_SUB_PATH/$POLL_SUB_PATH

copy_if_new () {
    if [[ "$2/$1" -nt "$3/$1" ]]; then 
        echo
        echo Moving $1 to $3 
        cp "$2/$1" "$3/$1"
        echo
    else
        echo File time older for: "$3/$1"
        stat -c %y "$2/$1"
        stat -c %y "$3/$1"
    fi
}

copy_if_new $RF_HEADER_NAME $SRC_PATH_RF $DST_PATH_RF
copy_if_new $RF_HEADER_NAME $SRC_PATH_RF $DST_PATH_RF_PBM
#copy_if_new $RF_HEADER_NAME $SRC_PATH_RF $DST_PATH_RF_DEMO

copy_if_new $EMV_HEADER_NAME $SRC_PATH_EMV $DST_PATH_EMV_PBM
#copy_if_new $EMV_HEADER_NAME $SRC_PATH_EMV $DST_PATH_EMV_DEMO

copy_if_new $PORT_HEADER_NAME $SRC_PATH_RF $DST_PATH_RF
copy_if_new $PORT_HEADER_NAME $SRC_PATH_RF $DST_PATH_RF_PBM
#copy_if_new $PORT_HEADER_NAME $SRC_PATH_RF $DST_PATH_RF_DEMO

copy_if_new $PBM_HEADER_NAME $SRC_PATH_PBM $DST_PATH_PBM

# This one is differnt from the other in this list, uses DST as 2nd arg
copy_if_new $TIMER_HEADER_NAME $DST_PATH_RF $PBM_LIB_PATH/$RF_SUB_PATH

#copy_if_new $POLL_HEADER_NAME $SRC_PATH_POLL $DST_PATH_POLL_DEMO

echo "** All Done **"
