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

root_dir=$(pwd)

# Get SHA
cd ${msdk}
msdk_head=$(git rev-parse HEAD)
cd ${root_dir}


# Cleanup hal_adi
rm -rf ${hal_adi}/MAX/

# Create parent folder
mkdir -p ${hal_adi}/MAX/Libraries/CMSIS
mkdir -p ${hal_adi}/MAX/Libraries/PeriphDrivers

# Copy zephyr wrappers, system files and cmakefiles
cp -rf ${msdk}/Libraries/zephyr/MAX/*  ${hal_adi}/MAX/

# Copy CMSIS folder
cp -rf ${msdk}/Libraries/CMSIS/Device  ${hal_adi}/MAX/Libraries/CMSIS/
cp -rf ${msdk}/Libraries/CMSIS/Include ${hal_adi}/MAX/Libraries/CMSIS/

# Copy PeriphDrivers folder
cp -rf ${msdk}/Libraries/PeriphDrivers/Include ${hal_adi}/MAX/Libraries/PeriphDrivers/
cp -rf ${msdk}/Libraries/PeriphDrivers/Source  ${hal_adi}/MAX/Libraries/PeriphDrivers/

# Remove unneeded files
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/GCC
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/IAR
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/GCC
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/ARM

# Check either dirty or clean
cd ${hal_adi}
if [[ -n $(git status -s) ]]; then
    echo "New change exist need to be pushed"
    # Set new SHA
    echo "${msdk_head}" > MAX/msdk_sha
else
    # No need to push to hal_adi
    echo "No any change in hal_adi"
fi

cd ${root_dir}
