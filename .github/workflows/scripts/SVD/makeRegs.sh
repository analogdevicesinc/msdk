#!/bin/bash
################################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ###############################################################################

# Generate {part_name}.svd and register files.

PART_NAME=${1^^}

MAXIM_PATH=$2

declare CHIP_NAME
declare DIE_NAME

declare -A CHIP_TO_DIE_NAMES 
declare -A DIE_TO_CHIP_NAMES

# Add future parts to this dictionary
CHIP_TO_DIE_NAMES[MAX78000]=AI85
CHIP_TO_DIE_NAMES[MAX78002]=AI87
CHIP_TO_DIE_NAMES[MAX32520]=ES17
CHIP_TO_DIE_NAMES[MAX32650]=ME10
CHIP_TO_DIE_NAMES[MAX32660]=ME11
CHIP_TO_DIE_NAMES[MAX32662]=ME12
CHIP_TO_DIE_NAMES[MAX32570]=ME13
CHIP_TO_DIE_NAMES[MAX32571]=ME13B
CHIP_TO_DIE_NAMES[MAX32665]=ME14
CHIP_TO_DIE_NAMES[MAX32670]=ME15
CHIP_TO_DIE_NAMES[MAX32675]=ME16
CHIP_TO_DIE_NAMES[MAX32655]=ME17
CHIP_TO_DIE_NAMES[MAX32690]=ME18
CHIP_TO_DIE_NAMES[MAX32680]=ME20
CHIP_TO_DIE_NAMES[MAX32672]=ME21
CHIP_TO_DIE_NAMES[MAX32572]=ME55

DIE_TO_CHIP_NAMES[AI85]=MAX78000
DIE_TO_CHIP_NAMES[AI87]=MAX78002
DIE_TO_CHIP_NAMES[ES17]=MAX32520
DIE_TO_CHIP_NAMES[ME10]=MAX32650
DIE_TO_CHIP_NAMES[ME11]=MAX32660
DIE_TO_CHIP_NAMES[ME12]=MAX32662
DIE_TO_CHIP_NAMES[ME13]=MAX32570
DIE_TO_CHIP_NAMES[ME13B]=MAX32571
DIE_TO_CHIP_NAMES[ME14]=MAX32665
DIE_TO_CHIP_NAMES[ME15]=MAX32670
DIE_TO_CHIP_NAMES[ME16]=MAX32675
DIE_TO_CHIP_NAMES[ME17]=MAX32655
DIE_TO_CHIP_NAMES[ME18]=MAX32690
DIE_TO_CHIP_NAMES[ME20]=MAX32680
DIE_TO_CHIP_NAMES[ME21]=MAX32672
DIE_TO_CHIP_NAMES[ME55]=MAX32572

# Confirm second argument is the part designation
if [[ ${DIE_TO_CHIP_NAMES[${PART_NAME}]+_} ]]; then
	CHIP_NAME=${DIE_TO_CHIP_NAMES[${PART_NAME}]}
	DIE_NAME=$PART_NAME

elif [[ ${CHIP_TO_DIE_NAMES[${PART_NAME}]+_} ]]; then
	CHIP_NAME=$PART_NAME
	DIE_NAME=${CHIP_TO_DIE_NAMES[${PART_NAME}]}

else
	echo -e "[\e[0;31mERROR\e[0m] Unknown part number."
	exit 1
fi

# Echo 
echo -e "\n****************************************************************************"
echo -e "*"
echo -e "* Generating register files and ${CHIP_NAME,,}.svd"
echo -e "*"
echo -e "****************************************************************************"

# Increase readability due to long paths
SVD_SCRIPTS_PATH=${MAXIM_PATH}/.github/workflows/scripts/SVD
SVD_DEVICE_PATH=${SVD_SCRIPTS_PATH}/Devices/${DIE_NAME}

if [[ -d "${SVD_DEVICE_PATH}/chip_test" ]]; then
	rm -rf ${SVD_DEVICE_PATH}/chip_test
fi

if [[ -f "${SVD_DEVICE_PATH}/${DIE_NAME,,}_register_name.txt" ]]; then
	rm ${SVD_DEVICE_PATH}/${DIE_NAME,,}_register_name.txt
fi

if [[ -f "${SVD_DEVICE_PATH}/svdconv_error_log.txt" ]]; then
	rm ${SVD_DEVICE_PATH}/svdconv_error_log.txt
fi

mkdir ${SVD_DEVICE_PATH}/chip_test
cp ${SVD_DEVICE_PATH}/empty_template.svd ${SVD_DEVICE_PATH}/chip_test/${CHIP_NAME,,}.svd

# Gather peripherals (including their peripheral SVD files) from chip_periph.txt and 
#   create {part_name}.svd from the peripheral SVD files.
#   Note: the final parameter is the MAXIM_PATH of the MSDK.
python3 ${SVD_SCRIPTS_PATH}/svd_add_peripheral_modified.py ${SVD_DEVICE_PATH}/chip_periph.txt ${SVD_DEVICE_PATH}/chip_test/${CHIP_NAME,,}.svd $MAXIM_PATH
error=$?
if [[ $error -ne 0 ]]; then
	echo "[ERROR] svd_add_peripheral_modified.py: Issue reading peripheral SVD files."
	exit 1
fi

echo Formatting xml file
xmlformat -i ${SVD_DEVICE_PATH}/chip_test/${CHIP_NAME}.svd

# Generate register files using the peripheral information gathered from {part_name}.svd
#   Note: -r parameter means generate the register offset definitions: MXC_R_{PERIPH}_{REGISTER}
python3 ${SVD_SCRIPTS_PATH}/svd_reg3.py ${SVD_DEVICE_PATH}/chip_test/${CHIP_NAME,,}.svd ${SVD_DEVICE_PATH}/chip_test ${SVD_DEVICE_PATH}/${DIE_NAME,,}_register_name.txt -r
error=$?
if [[ $error -ne 0 ]]; then
	echo "[ERROR] svd_reg3.py: Could not finish generating register files."
	exit 1
fi

cp -rf ${SVD_DEVICE_PATH}/chip_test/. ${MAXIM_PATH}/Libraries/CMSIS/Device/Maxim/${CHIP_NAME^^}/Include/

# Check for errors and warnings in {part_name}.svd for IAR/Keil.
#   Note: Not all warnings removable, but all errors should be resolved.
# This conditional is for the Check_Register_SVD Workflow. .EXE can't run on Ubuntu.
if [[ ${@: -1} == "windows" || -z ${3} ]]; then
	${SVD_SCRIPTS_PATH}/SVDConv.exe ${SVD_DEVICE_PATH}/chip_test/${CHIP_NAME,,}.svd -b ${SVD_DEVICE_PATH}/svdconv_error_log.txt
fi
