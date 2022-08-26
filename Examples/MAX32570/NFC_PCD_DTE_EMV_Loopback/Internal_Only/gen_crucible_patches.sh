#!/bin/sh

TAG_TO_DIFF_FROM="v4.1.0"
PATH_TO_DTE=".."
PATH_TO_PBM="../../../../Libraries/NFC/lib_nfc_pcd_pbm"
PATH_TO_RF_DRIVER="../../../../Libraries/NFC/lib_nfc_pcd_rf_driver_MAX32570"
LAUNCH_DIR=$(pwd)

echo "Launched here: $LAUNCH_DIR"

echo
echo "Generating DTE Diff from $TAG_TO_DIFF_FROM"
pushd $PATH_TO_DTE
git diff -U5000 $TAG_TO_DIFF_FROM > $LAUNCH_DIR/stack_and_dte_code_review.patch
popd

echo
echo "Generating RF Driver Diff from $TAG_TO_DIFF_FROM"
pushd $PATH_TO_RF_DRIVER
git diff -U5000 $TAG_TO_DIFF_FROM > $LAUNCH_DIR/rf_driver_code_review.patch
popd

echo
echo "Generating PBM Diff from $TAG_TO_DIFF_FROM"
pushd $PATH_TO_PBM
git diff -U5000 $TAG_TO_DIFF_FROM > $LAUNCH_DIR/pbm_lib_code_review.patch
popd
