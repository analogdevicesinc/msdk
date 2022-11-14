#!/bin/bash

VERSION=4.2.0

TOOL_PATH=/c/MaximSDK_2_5_2020/Tools/GNUTools/bin
PATH_TO_RELEASE=/c/MaximSDK_2_5_2020/Examples/MAX32570/NFC_PCD_DTE_EMV_Loopback/Internal_Only
PATH_TO_DTE=/c/MaximSDK_2_5_2020/Examples/MAX32570/NFC_PCD_DTE_EMV_Loopback
PATH_TO_RF_DRIVER=../../../../Libraries/NFC/lib_nfc_pcd_rf_driver_MAX32570
PATH_TO_PBM=../../../../Libraries/NFC/lib_nfc_pcd_pbm
DEST_DIR=$PATH_TO_RELEASE/lib
DEV_L1_APP=$PATH_TO_DTE/src/nfc/contactless_l1_app/emvl1_app.c

mkdir -p $DEST_DIR

# Grab all the libraries, and move to the Release directory
cp $PATH_TO_RF_DRIVER/libnfc_pcd_rf_driver_MAX32570_softfp.a "$DEST_DIR"
cp $PATH_TO_PBM/libnfc_pcd_pbm_softfp.a "$DEST_DIR"
cp $PATH_TO_DTE/build/iso14443_*.o "$DEST_DIR"

# Show the compiler version
LIB_INFO=$(echo arm-none-eabi-gcc --version:)
LIB_INFO+=$'\n'
LIB_INFO+=$'\n'
LIB_INFO+=$("$TOOL_PATH"/arm-none-eabi-gcc --version)
LIB_INFO+=$'\n'
LIB_INFO+=$'\n'
RETURN_TO_DIR=$(pwd)

cd "$DEST_DIR"

# List the size/mem usage of the libraries
LIB_INFO+=$(echo arm-none-eabi-size -t *.a *.o:)
LIB_INFO+=$'\n'
LIB_INFO+=$'\n'
LIB_INFO+=$("$TOOL_PATH"/arm-none-eabi-size -t *.a *.o)
LIB_INFO+=$'\n'
LIB_INFO+=$'\n'

## Calculate the HASH of the libraries
LIB_INFO+=$(echo "cat" *.a " > nfc_lib_blob")
LIB_INFO+=$(cat *.a > nfc_lib_blob)
LIB_INFO+=$'\n'
LIB_INFO+=$(echo "cat" *.o nfc_lib_blob" > nfc_lib_blob_combo")
LIB_INFO+=$(cat *.o nfc_lib_blob > nfc_lib_blob_combo)
LIB_INFO+=$'\n'

LIB_INFO+=$(echo "sha1sum nfc_lib_blob_combo")
LIB_INFO+=$'\n'
LIB_INFO+=$'\n'
HASH=$(sha1sum nfc_lib_blob_combo)
LIB_INFO+=$HASH

rm nfc_lib_blob
rm nfc_lib_blob_combo
rm *.o
rm *.a
cd "$RETURN_TO_DIR"
rmdir lib

JUST_HASH=$(echo $HASH | awk '{printf $1}')

# Insert the VERSION and HASH into the DTE
#echo sed -i 's/\(^#define.*PCD_SW_VER.*"\)[0-9]\.[0-9]\.[0-9]\(".*\)/\1'$VERSION'\2/g' "$DEV_L1_APP"
sed -i 's/\(^#define.*PCD_SW_VER.*"\)[0-9]\.[0-9]\.[0-9]\(".*\)/\1'$VERSION'\2/g' "$DEV_L1_APP"

sed -i 's/\(^#define.*PCD_FW_SUM.*"\)[0-9a-z]*\(".*\)/\1'$JUST_HASH'\2/g' "$DEV_L1_APP"

echo "$LIB_INFO"

# Use per for multiline substitution of size output generated above.
# Almost working, needs tab.
# Does it have to be tab? can it be space?
#TABBED_LIB_INFO=$(sed 's|^|\t|' $LIB_INFO)
#TABBED_LIB_INFO=$(perl -pe "s~^~\t~i" -- "$LIB_INFO")

#echo "$TABBED_LIB_INFO"
#perl -0777 -i.original -pe "s~\sarm-none-eabi-gcc\ --version:.*[0-9,a-f]*\ nfc_lib_blob_combo~$LIB_INFO~igs" MAXIM_TEST.md
