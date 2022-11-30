#!/bin/sh

echo "****************************************************************"
echo "* Run this script from the REPO root folder.                   *"
echo "* Example:                                                     *"
echo "*   $ .github/workflows/local_clang-format-check.sh            *"
echo "****************************************************************"
echo 

echo pwd=$(pwd)
echo

cat .clang-format

CHANGE_FILES=$(git diff --ignore-submodules --name-only remotes/origin/main '*.c' '*.h' ':!*_regs.h')

echo CHANGE_FILES=$CHANGE_FILES

CHECK_FAIL=0

set +e

for change_file in ${CHANGE_FILES}
do
    if [ -f ${change_file} ];
    then
        clang-format --verbose --style=file -n -Werror ${change_file}
        if [ $? != 0 ];
        then
            clang-format --style=file ${change_file} > ${change_file}.clang
            diff -u --color=always ${change_file} ${change_file}.clang
            rm ${change_file}.clang
            CHECK_FAIL=1
        fi
    fi
done
set -e


echo "\nDone with $CHECK_FAIL."

exit $CHECK_FAIL
