#!/bin/bash
# Generate {part_name}.svd and register files.
# This won't be used by the workflow. For manual use only.

if [[ -d "private_rev_reg_files" ]]; then
	rm -rf private_rev_reg_files
fi

mkdir private_rev_reg_files
cp empty_template.svd private_rev_reg_files/private.svd

# Gather peripherals (including their peripheral SVD files) from chip_periph.txt and 
#   create {part_name}.svd from the peripheral SVD files.
#   Note: the final parameter (../../../../../../) represents the path to where the MSDK repo
#   is located relative to where this file is located. In this case, it's the relative path
#	to the root of the repo.
python3 ../../svd_add_peripheral_modified.py private_rev_paths.txt private_rev_reg_files/private.svd ../../../../../../ -r
error=$?
if [[ $error -ne 0 ]]; then
	echo "[ERROR] svd_add_peripheral_modified.py: Issue reading peripheral SVD files."
	exit 1
fi

echo Formatting xml file
xmlformat -i private_rev_reg_files/private.svd

# Generate register files using the peripheral information gathered from {part_name}.svd
#   Note: -r parameter means generate the register offset definitions: MXC_R_{PERIPH}_{REGISTER}
python3 ../../svd_reg3.py private_rev_reg_files/private.svd private_rev_reg_files private_register_name.txt
error=$?
if [[ $error -ne 0 ]]; then
	echo "[ERROR] svd_reg3.py: Could not finish generating register files."
	exit 1
fi

rm private_register_name.txt
