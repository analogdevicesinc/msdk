echo
echo "#################################################################################################################"
echo "# ble5_ctr_best_file_change_check.sh 1_SKIP_FCC 2_MSDK 3_CHIP_UC 4_BRD_TYPE                                     #"
echo "#     return 0: need to run the test because of file changes                                                    #"
echo "#     return 1: no need to run the test                                                                         #"
echo "#################################################################################################################"
echo
echo $0 $@
echo

SKIP_FCC=$1
MSDK=$2
CHIP_UC=$3
BRD_TYPE=$4

echo " SKIP_FCC: $SKIP_FCC"
echo "     MSDK: $MSDK"
echo "  CHIP_UC: $CHIP_UC"
echo " BRD_TYPE: $BRD_TYPE"
echo

CHIP_LC=${CHIP_UC,,}

#----------------------------------------------------------------------------------------------------------------------
# Check if need to do this job or not.
if [ "${SKIP_FCC}" == "1" ]; then
    echo "Skip this file change check."
    echo

    exit 0
fi

#----------------------------------------------------------------------------------------------------------------------
# Need to check the repo file changes.
# Remove local modifications
cd ${MSDK}
echo PWD: `pwd`
echo

set -x
git scorch
set +x

BLE_FILES_CHANGED=0

# Check for changes made to the files
WATCH_FILES="\
    Examples/$CHIP_UC                                             \
    .github/workflows/ble5_ctr_test.yml                           \
    .github/workflows/scirpts/ble5_ctr_test_file_change_check.sh  \
    "

echo "Watching these locations and files"
echo $WATCH_FILES
echo

# Get the diff from main
CHANGE_FILES=$(git diff --ignore-submodules --name-only remotes/origin/main)

echo "Checking the following changes"
echo $CHANGE_FILES
echo

# Assume we want to actually run the workflow if no files changed
if [[ "$CHANGE_FILES" != "" ]]; then
    for watch_file in $WATCH_FILES; do 
        if [[ "$CHANGE_FILES" == *"$watch_file"* ]]; then
            BLE_FILES_CHANGED=1
            echo "Found BLE file changes. Need to run the test."
            
            exit 0
        fi
    done
    
    if [[ $BLE_FILES_CHANGED -eq 0 ]]
    then
        echo "No need to run test on ${CHIP_UC}."
        exit 1
    fi
fi

echo "No changes. Skip the test."
echo

exit 1
