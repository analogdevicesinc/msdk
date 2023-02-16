#!/usr/bin/env bash

echo
echo "#################################################################################################################"
echo "# msdk_per_skip_check.sh 1_NO_SKIP 2_MSDK 3_CHIP_UC 4_BRD_TYPE                                                  #"
echo "#      return 0: run the test                                                                                   #"
echo "#      return 1: skip the test                                                                                  #"
echo "#################################################################################################################"
echo
echo $0 $@
echo

NO_SKIP=$1
MSDK=$2
CHIP_UC=$3
BRD_TYPE=$4

echo " NO_SKIP: $NO_SKIP"
echo "    MSDK: $MSDK"
echo " CHIP_UC: $CHIP_UC"
echo "BRD_TYPE: $BRD_TYPE"
echo

CHIP_LC=${CHIP_UC,,}

#----------------------------------------------------------------------------------------------------------------------
# Check if need to do this job or not.
if [ "${NO_SKIP}" == "1" ]; then
    echo "Disable the file change check skip."
    echo

    exit 0
fi

#----------------------------------------------------------------------------------------------------------------------
# Need to check the repo changes.
# Remove local modifications
cd ${MSDK}
echo PWD: `pwd`
echo

git scorch

BLE_FILES_CHANGED=0

# Check for changes made to these files
WATCH_FILES="\
    .github                      \
    Examples/${CHIP_UC}/BLE5_ctr \
    Libraries/libs.mk            \
    Libraries/Cordio             \
    Libraries/CMSIS/Device/Maxim/${CHIP_UC}     \
    Libraries/PeriphDrivers/libPeriphDriver.mk  \   
    Libraries/PeriphDrivers/periphdriver.mk     \
    Libraries/PeriphDrivers/${CHIP_LC}_files.mk \
    Libraries/PeriphDrivers/Source              \
    Libraries/PeriphDrivers/Include/${CHIP_UC}  \
    Libraries/BlePhy/${CHIP_UC}                 \
    Libraries/Boards/${CHIP_UC}"
    
# Get the diff from main
CHANGE_FILES=$(git diff --ignore-submodules --name-only remotes/origin/main)

echo "Watching these locations and files"
echo $WATCH_FILES
echo

echo "Checking the following changes"
echo $CHANGE_FILES
echo

# Assume we want to actually run the workflow if no files changed
if [[ "$CHANGE_FILES" != "" ]]; then
    for watch_file in $WATCH_FILES; do 
        if [[ "$CHANGE_FILES" == *"$watch_file"* ]]; then
            BLE_FILES_CHANGED=1
            echo "Found BLE file changes. Run the test."
            
            exit 0
        fi
    done
    if [[ $BLE_FILES_CHANGED -eq 0 ]]
    then
        echo "Skipping ${CHIP_UC} Test"
        # Files were changed but not with this chip
        exit 1
    fi
fi

echo "No changes. Skip the test."
exit 1
