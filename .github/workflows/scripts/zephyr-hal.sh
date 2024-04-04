#!/bin/bash

echo
echo "Test workflow and script for copying files from MSDK repo to hal_adi."
echo

if [ $# -eq 2 ]; then
    msdk="$1"
    hal_adi="$2"
else
    msdk="./msdk"
    hal_adi="./hal_adi"
fi

src="${msdk}/Libraries"
dst="${hal_adi}/MAX/Libraries"

root_dir=$(pwd)

# Get SHA
cd ${msdk}
msdk_head=$(git rev-parse HEAD)
cd ${root_dir}


# Cleanup hal_adi
rm -rf ${dst}/CMSIS/*
rm -rf ${dst}/PeriphDrivers/*

# Copy CMSIS folder
cp -rf ${src}/CMSIS/Device  ${dst}/CMSIS/
cp -rf ${src}/CMSIS/Include  ${dst}/CMSIS/

# Copy PeriphDrivers folder
cp -rf ${src}/PeriphDrivers/Include  ${dst}/PeriphDrivers/
cp -rf ${src}/PeriphDrivers/Source  ${dst}/PeriphDrivers/

# Remove unneeded files
rm -rf ${dst}/CMSIS/Device/Maxim/GCC
rm -rf ${dst}/CMSIS/Device/Maxim/MAX*/Source/IAR
rm -rf ${dst}/CMSIS/Device/Maxim/MAX*/Source/GCC
rm -rf ${dst}/CMSIS/Device/Maxim/MAX*/Source/ARM

# Check either dirty or clean
cd ${hal_adi}
if [[ -n $(git status -s) ]]; then
    echo "New change exist need to be pushed"
    # Set new SHA
    echo "${msdk_head}" > MAX/Libraries/msdk_sha
else
    # No need to push to hal_adi
    echo "No any change in hal_adi"
fi

cd ${root_dir}
