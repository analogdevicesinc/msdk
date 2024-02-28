#!/bin/bash
# Generate {part_name}.svd and register files.

if [[ -d "chip_test" ]]; then
	rm -rf chip_test
fi

if [[ -f "me15_register_name.txt" ]]; then
	rm me15_register_name.txt
fi

if [[ -f "svdconv_error_log.txt" ]]; then
	rm svdconv_error_log.txt
fi

mkdir chip_test
cp empty_template.svd chip_test/max32670.svd

# Gather peripherals (including their peripheral SVD files) from chip_periph.txt and 
#   create {part_name}.svd from the peripheral SVD files.
#   Note: the final parameter (../../../../../../) represents the path to where the MSDK repo
#   is located relative to where this file is located. In this case, it's the relative path
#	to the root of the repo.
python3 ../../svd_add_peripheral_modified.py chip_periph.txt chip_test/max32670.svd ../../../../../../
error=$?
if [[ $error -ne 0 ]]; then
	echo "[ERROR] svd_add_peripheral_modified.py: Issue reading peripheral SVD files."
	exit 1
fi

echo Formatting xml file
xmlformat -i chip_test/max32670.svd

# Generate register files using the peripheral information gathered from {part_name}.svd
#   Note: -r parameter means generate the register offset definitions: MXC_R_{PERIPH}_{REGISTER}
python3 ../../svd_reg3.py chip_test/max32670.svd chip_test me15_register_name.txt -r
error=$?
if [[ $error -ne 0 ]]; then
	echo "[ERROR] svd_reg3.py: Could not finish generating register files."
	exit 1
fi

# Check for errors and warnings in {part_name}.svd for IAR/Keil.
#   Note: Not all warnings removable, but all errors should be resolved.
# This conditional is for the Check_Register_SVD Workflow. .EXE can't run on Ubuntu unless you
#   install some packages.
if [[ $1 == "windows" || -z $1 ]]; then
	../../SVDConv.exe chip_test/max32670.svd -b svdconv_error_log.txt
fi
