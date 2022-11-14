#!/bin/sh
# More to handle:
# timer_utils
# all stack headers

SRC_PATH_RF_HEADER="../../../../Libraries/NFC/lib_nfc_pcd_rf_driver_MAX32570/include/nfc"
DST_PATH_RF_HEADER="../include/nfc"

SRC_PATH_PBM_HEADER="../../../../Libraries/NFC/lib_nfc_pcd_pbm/include"
DST_PATH_PBM_HEADER="../include/nfc/pbm"

RF_HEADER_NAME="mml_nfc_pcd_rf_driver.h"
RF_HEADER_SRC="$SRC_PATH_RF_HEADER/$RF_HEADER_NAME"
RF_HEADER_DST="$DST_PATH_RF_HEADER/$RF_HEADER_NAME"

PORT_HEADER_NAME="mml_nfc_pcd_port.h"
PORT_HEADER_SRC="$SRC_PATH_RF_HEADER/$PORT_HEADER_NAME"
PORT_HEADER_DST="$DST_PATH_RF_HEADER/$PORT_HEADER_NAME"

PBM_HEADER_NAME="pbm_commands.h"
PBM_HEADER_SRC="$SRC_PATH_PBM_HEADER/$PBM_HEADER_NAME"
PBM_HEADER_DST="$DST_PATH_PBM_HEADER/$PBM_HEADER_NAME"

update_check () {
    if [[ $2 -nt $3 ]]; then
        echo
        echo $1
        echo
        echo $2 is newer than $3

        echo
        echo DIFF
        echo
        diff $2 $3

        echo
        read -p "Update? " -n 1 -r
        echo    # (optional) move to a new line
        if [[ $REPLY =~ ^[Yy]$ ]]
        then
            cp $2 $3
        fi
    fi
}

update_check "RF Driver Header" $RF_HEADER_SRC      $RF_HEADER_DST
update_check "RF Port Header"   $PORT_HEADER_SRC    $PORT_HEADER_DST
update_check "PBM Header"       $PBM_HEADER_SRC     $PBM_HEADER_DST

echo "** All Done **"
