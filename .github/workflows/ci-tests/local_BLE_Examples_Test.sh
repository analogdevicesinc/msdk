#!/usr/bin/bash

echo "#############################################################################################"
echo "# Match the test in BLE_Examples_Test.yml                                                   #"
echo "# Usage: local_BLE_Examples_Test.sh /path/of/msdk                                           #"
echo "#############################################################################################"
echo

#set -x
set -e

if [ $# != 1 ]; then
    echo "Usage: local_BLE_Examples_Test.sh /path/of/msdk"
    exit 1
fi

MSDK=$1

if [ ! -d "$1" ]; then
    echo "Invalid MSDK path."
    exit 2
fi

cd $MSDK
echo "PWD=$(pwd)"

#--------------------------------------------------------------------------------------------------
BLE_FILES_CHANGED=0

# Check for changes made to these files
WATCH_FILES="\
Examples/MAX32655/BLE \
Libraries/libs.mk \
Libraries/Cordio \
Libraries/CMSIS/Device/Maxim/MAX32655 \
Libraries/PeriphDrivers/libPeriphDriver.mk \
Libraries/PeriphDrivers/periphdriver.mk \
Libraries/PeriphDrivers/max32655_files.mk \
Libraries/PeriphDrivers/Source \
Libraries/PeriphDrivers/Include/MAX32655 \
Libraries/BlePhy/MAX32655 \
Libraries/Boards/MAX32655"

echo "$(git status -u)"

# Get the diff from main
CHANGE_FILES=$(git diff --ignore-submodules --name-only remotes/origin/main)

echo "Watching these locations and files"
echo $WATCH_FILES

echo "Checking the following changes"
echo $CHANGE_FILES

# Assume we want to actually run the workflow if no files changed
if [[ "$CHANGE_FILES" != "" ]]; then
    for watch_file in $WATCH_FILES; do 
        if [[ "$CHANGE_FILES" == *"$watch_file"* ]]; then
            BLE_FILES_CHANGED=1
        fi
    done
    if [[ $BLE_FILES_CHANGED -eq 0 ]]
    then
        echo "Skipping Test"
        #exit 0  # remove me !!!
    fi
fi

echo "#############################################################################################"
echo "Running Test"
echo "#############################################################################################"
echo 

# Lock hardware resources
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 600 /home/$USER/Workspace/Resource_Share/max32655_0.txt
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 600 /home/$USER/Workspace/Resource_Share/max32655_1.txt

cp -r .github/workflows/ci-tests/Examples_tests .
chmod u+x Examples_tests/local_testLauncher.sh
cd Examples_tests/
./local_testLauncher.sh

python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_1.txt
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_0.txt