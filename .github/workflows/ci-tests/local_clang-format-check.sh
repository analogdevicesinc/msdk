#!/usr/bin/bash

echo "#############################################################################################"
echo "# Match the test in clang-format-change.yml                                                 #"
echo "# Usage: local_clang-format-change.sh /path/of/msdk                                         #"
echo "#############################################################################################"
echo

#set -x
set -e

if [ $# != 1 ]; then
    echo "Usage: local_clang-format-change.sh /path/of/msdk"
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
if [ $(hostname) == "yingcai-OptiPlex-790" ]; then 
    CLANG_VERSION=12
else
    CLANG_VERSION=14
fi

cat .clang-format
CHANGE_FILES=$(git diff --ignore-submodules --name-only remotes/origin/main '*.c' '*.h' ':!*_regs.h' ':!*weights.h' ':!*cnn.h' ':!*cnn.c' ':!*sampledata.h' ':!*sampleoutput.h' ':!*softmax.c' ':!Libraries/FCL' ':!Libraries/FreeRTOS' ':!Libraries/lwIP' ':!Libraries/littlefs' ':!Libraries/FreeRTOS-Plus' ':!Libraries/LC3' ':!Libraries/SDHC' ':!Libraries/MAXUSB' ':!Libraries/Cordio')
CHECK_FAIL=0
set +e
for change_file in ${CHANGE_FILES}
do
if [ -f ${change_file} ];
then
clang-format-${CLANG_VERSION} --verbose --style=file -n -Werror ${change_file}
if [ $? != 0 ];
then
    echo ""
    echo "===================================================="
    echo "FAIL: ${change_file}"
    echo "----------------------------------------------------"
    echo "Expected:"
    clang-format-${CLANG_VERSION} --style=file ${change_file} > ${change_file}.clang
    diff -u --color=always ${change_file} ${change_file}.clang
    rm ${change_file}.clang
    CHECK_FAIL=1
    echo "===================================================="
    echo ""
fi
fi
done
set -e
exit $CHECK_FAIL