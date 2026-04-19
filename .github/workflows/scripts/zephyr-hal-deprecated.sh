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

# Store msdk_sha file if exist
if [ -e "${hal_adi}/MAX/msdk_sha" ]; then
    mv ${hal_adi}/MAX/msdk_sha ${root_dir}
fi

# Cleanup hal_adi
rm -rf ${hal_adi}/MAX/

# Create parent folder
mkdir -p ${hal_adi}/MAX/Libraries/CMSIS
mkdir -p ${hal_adi}/MAX/Libraries/MAXUSB/include
mkdir -p ${hal_adi}/MAX/Libraries/MAXUSB/src
mkdir -p ${hal_adi}/MAX/Libraries/PeriphDrivers

# Copy zephyr wrappers, system files and cmakefiles
cp -rf ${msdk}/Libraries/zephyr/MAX/*  ${hal_adi}/MAX/

# Move "msdk_sha" file again to not create a difference.
if [ -e "${root_dir}/msdk_sha" ]; then
    mv ${root_dir}/msdk_sha ${hal_adi}/MAX
fi

# Copy CMSIS folder
cp -rf ${msdk}/Libraries/CMSIS/Device  ${hal_adi}/MAX/Libraries/CMSIS/
cp -rf ${msdk}/Libraries/CMSIS/Include ${hal_adi}/MAX/Libraries/CMSIS/

# Copy PeriphDrivers folder
cp -rf ${msdk}/Libraries/PeriphDrivers/Include ${hal_adi}/MAX/Libraries/PeriphDrivers/
cp -rf ${msdk}/Libraries/PeriphDrivers/Source  ${hal_adi}/MAX/Libraries/PeriphDrivers/

# Copy MAXUSB folder
cp -rf ${msdk}/Libraries/MAXUSB/include/core ${hal_adi}/MAX/Libraries/MAXUSB/include/
cp -rf ${msdk}/Libraries/MAXUSB/src/core     ${hal_adi}/MAX/Libraries/MAXUSB/src/

# Remove unneeded files
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/GCC
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/IAR
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/GCC
rm -rf ${hal_adi}/MAX/Libraries/CMSIS/Device/Maxim/MAX*/Source/ARM
rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/include/core/arm
rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/include/core/maxq
rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/src/core/arm
rm -rf ${hal_adi}/MAX/Libraries/MAXUSB/src/core/maxq

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
