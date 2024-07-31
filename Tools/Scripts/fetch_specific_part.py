###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
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
 ##############################################################################

import sys
import os
import subprocess
import shutil
import argparse
import logging
from pathlib import Path

chip_name_dict = {
    "max32520": "es17",
    "max32650": "me10",
    "max32655": "me17",
    "max32657": "me30",
    "max32660": "me11",
    "max32662": "me12",
    "max32665": "me14",
    "max32670": "me15",
    "max32672": "me21",
    "max32675": "me16",
    "max32680": "me20",
    "max32690": "me18",
    "max78000": "ai85",
    "max78002": "ai87"
}

die_name_dict = { die_name: chip_name for chip_name, die_name in chip_name_dict.items() }

# Instantiate the argument parser.
parser = argparse.ArgumentParser(description='Fetch all files related to a specific part from MSDK.')

# Add required arguments.
parser.add_argument('part_name', type=str, help='The name of the MCU to fetch (capital-insensitive). Both chip names (MAX32690) and die names (AI85) are valid part names.')
parser.add_argument('msdk_path', type=str, help='The path to msdk location (MAXIM_PATH equivalent).')
parser.add_argument('destination', type=str, help='The destination directory to store all related files.')

# Parse incoming arguments.
args = parser.parse_args()

# Get chip and die name
part_name = args.part_name.lower()
if part_name in chip_name_dict.keys():
    chip_name = part_name
    die_name = chip_name_dict[part_name]
elif part_name in die_name_dict.keys():
    chip_name = die_name_dict[part_name]
    die_name = part_name
else:
    print("Invalid part name. Please enter proper chip or die name.")
    sys.exit(1)


msdk_path = args.msdk_path
destination = args.destination

print("MSDK Path: " + msdk_path)
print("Destination: " + destination)

def delete_lines(filename, start_line, end_line):
    with open(filename, "r") as f:
        lines = f.readlines()

    with open(filename, "w") as f:
        for i, line in enumerate(lines, start=1):
            if i < start_line or i > end_line:
                f.write(line)



#########################################################################################

# Create directory if it doesn't exist:
if not os.path.isdir(destination):
    os.makedirs(destination, exist_ok=True)

# Copy examples over
examples_src = os.path.join(msdk_path, "Examples/" + chip_name.upper())
examples_dst = os.path.join(destination, "Examples/" + chip_name.upper())

shutil.copytree(examples_src, examples_dst, dirs_exist_ok=True)

# Copy Boards over
boards_src = os.path.join(msdk_path, "Libraries/Boards/" + chip_name.upper() + "/")
boards_dst = os.path.join(destination, "Libraries/Boards/" + chip_name.upper() + "/")

shutil.copytree(boards_src, boards_dst, dirs_exist_ok=True)

# Copy MiscDrivers
miscdrivers_src = os.path.join(msdk_path, "Libraries/MiscDrivers/")
miscdrivers_dst = os.path.join(destination, "Libraries/MiscDrivers/")

shutil.copytree(miscdrivers_src, miscdrivers_dst, dirs_exist_ok=True)

# Copy CMSIS
cmsis_device_gcc_src = os.path.join(msdk_path, "Libraries/CMSIS/Device/Maxim/GCC/gcc.mk")
cmsis_device_gcc_dst = os.path.join(destination, "Libraries/CMSIS/Device/Maxim/GCC/")

os.makedirs(cmsis_device_gcc_dst, exist_ok=True)
shutil.copy(cmsis_device_gcc_src, cmsis_device_gcc_dst + "gcc.mk")

# Remove mxc_version, not compatible outside of GitHub repo or MaximSDK Install.
delete_lines(cmsis_device_gcc_dst + "gcc.mk", 21, 60)

cmsis_core_src = os.path.join(msdk_path, "Libraries/CMSIS/5.9.0")
cmsis_core_dst = os.path.join(destination, "Libraries/CMSIS/5.9.0")

shutil.copytree(cmsis_core_src, cmsis_core_dst, dirs_exist_ok=True)

cmsis_device_src = os.path.join(msdk_path, "Libraries/CMSIS/Device/Maxim/" + chip_name.upper() + "/")
cmsis_device_dst = os.path.join(destination, "Libraries/CMSIS/Device/Maxim/" + chip_name.upper() + "/")

shutil.copytree(cmsis_device_src, cmsis_device_dst, dirs_exist_ok=True)

# Copy PeriphDrivers
periphdrivers_dir = os.path.join(destination, "Libraries/PeriphDrivers/")
os.makedirs(periphdrivers_dir, exist_ok=True)

periphdrivers_mk_src = os.path.join(msdk_path, "Libraries/PeriphDrivers/libPeriphDriver.mk")
periphdrivers_mk_dst = os.path.join(destination, "Libraries/PeriphDrivers/libPeriphDriver.mk")

shutil.copy(periphdrivers_mk_src, periphdrivers_mk_dst)

periphdrivers_mk_src = os.path.join(msdk_path, "Libraries/PeriphDrivers/periphdriver.mk")
periphdrivers_mk_dst = os.path.join(destination, "Libraries/PeriphDrivers/periphdriver.mk")

shutil.copy(periphdrivers_mk_src, periphdrivers_mk_dst)

periphdrivers_mk_src = os.path.join(msdk_path, "Libraries/PeriphDrivers/" + chip_name.lower() + "_files.mk")
periphdrivers_mk_dst = os.path.join(destination, "Libraries/PeriphDrivers/" + chip_name.lower() + "_files.mk")

shutil.copy(periphdrivers_mk_src, periphdrivers_mk_dst)

periphdrivers_include_src = os.path.join(msdk_path, "Libraries/PeriphDrivers/Include/" + chip_name.upper() + "/")
periphdrivers_include_dst = os.path.join(destination, "Libraries/PeriphDrivers/Include/" + chip_name.upper() + "/")

shutil.copytree(periphdrivers_include_src, periphdrivers_include_dst, dirs_exist_ok=True)

periphdrivers_source_src = os.path.join(msdk_path, "Libraries/PeriphDrivers/Source/")
periphdrivers_source_dst = os.path.join(destination, "Libraries/PeriphDrivers/Source/")

os.makedirs(periphdrivers_source_dst, exist_ok=True)

# PeriphDrivers

# Save separate lists of source directories and files required for the part.
#   SYS can not be easily parsed using the {part}_files.mk file.
periphdrivers_src_dirs = ["SYS"]

periphdrivers_src_files = []

with open(periphdrivers_mk_src, 'r') as part_files_mk:
    for line in part_files_mk:
        # Get Source Directories.
        if line.startswith("PERIPH_DRIVER_INCLUDE_DIR"):
            # Specify with another unique token for valid directory names.
            if "SOURCE_DIR" in line:
                # Isolate last directory name only.
                src_dir = line.split("/")[-1].rstrip('\n')
                periphdrivers_src_dirs.append(src_dir)

        # Get Source Files.
        if line.startswith("PERIPH_DRIVER_C_FILES"):
            # Isolate file and their parent directory path.
            src_file = line.split("$(SOURCE_DIR)/")[-1].rstrip('\n')
            periphdrivers_src_files.append(src_file)
        
        # Quick hack to add i3c_ccc.h.
        if "I3C" in line:
            periphdrivers_src_files.append("I3C/i3c_ccc.h")

# Create all source directories.
for src_dir in periphdrivers_src_dirs:
    os.makedirs(periphdrivers_source_dst + src_dir, exist_ok=True)

# Copy all source files to their appropriate locations.
for src_file in periphdrivers_src_files:
    shutil.copy(periphdrivers_source_src + src_file, periphdrivers_source_dst + src_file)

    # Grab accompanying src/inc files that weren't listed in {part}_files.mk.
    #   For example, {periph}_common.c/.h, {periph}_revX_regs.h, {periph}_revX.h 
    if "_rev" in src_file:
        # Isolate parent directory.
        parent_periph_dir = src_file.split("/")[0]

        # Isolate revision version. 
        revision = src_file.split("_rev")[-1]
        index_to_first_underscore = revision.find("_")
        revision_id = "_rev" + revision[:index_to_first_underscore-1]

        rev_files = [file for file in os.listdir(periphdrivers_source_src + "/" + parent_periph_dir) if revision_id in file and ".svd" not in file]

        # Account fo rev revisions (_reva1 and _reva2 files share the same _reva register file)
        # _revX is 5 characters long
        if len(revision_id) > 5:
            # Grab register file
            rev_files.append(parent_periph_dir.lower() + revision_id[:-1] + "_regs.h")

        for file in rev_files:
            src_file = periphdrivers_source_src + "/" + parent_periph_dir + "/" + file
            dst_file = periphdrivers_source_dst + "/" + parent_periph_dir + "/" + file
            if os.path.isfile(src_file):
                shutil.copy(src_file, dst_file)


    if "_common" in src_file:
        # Replace .c file extension to header file
        common_file = src_file[:-2] + ".h"
        # Check if file at source exists, if not, don't copy
        if os.path.exists(periphdrivers_source_src + common_file):
            shutil.copy(periphdrivers_source_src + common_file, periphdrivers_source_dst + common_file)

# Copy libs.mk
libs_mk_src = os.path.join(msdk_path, "Libraries/libs.mk")
libs_mk_dst = os.path.join(destination, "Libraries/libs.mk")
shutil.copy(libs_mk_src, libs_mk_dst)

# Copy Tools
tools_src = os.path.join(msdk_path, "Tools/Flash")
tools_dst = os.path.join(destination, "Tools/Flash")
shutil.copytree(tools_src, tools_dst, dirs_exist_ok=True)
