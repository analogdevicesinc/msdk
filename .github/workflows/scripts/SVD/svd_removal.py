#!/usr/bin/env python3
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

import sys
import os
import pathlib
import re
import xml.etree.ElementTree as ET
import textwrap

def get_peripheral_name(register_file):
  dir_path, file_name = os.path.split(register_file)
  return file_name.split("_regs.h")[0]

#==================================================================================

def remove_register_struct(register_file):
  peripheral_name = get_peripheral_name(register_file).lower()

  print("    -Removing register struct from peripheral: " + peripheral_name.upper())

  # Get struct type name of peripheral
  reg_struct_name = "mxc_" + peripheral_name + "_regs_t"

  # Get File Contents
  try:
    with open(register_file, 'r+') as file:
      file_contents = file.readlines()

      # Variable to indicate if structure exists.
      struct_detect = 0

      # Keep track of the lines with the struct to remove
      struct_begin_line = 0
      struct_end_line = 0

      # Find line indexs for struct section
      for index, line in enumerate(file_contents):
        # Unique token to find beginning of register structure section
        if ("Structure type to access the" in line):
          # Token can not begin at the beginning of the file due to nature of generated comments
          # 2 is so there can't be a negative index
          if index < 2:
            struct_begin_line = 0
          else:
            struct_detect += 1
            struct_begin_line = index - 2

        # Find the end of the struct
        if (struct_detect == 1):
          if (reg_struct_name in line):
            struct_detect += 1
            struct_end_line = index + 2 # remove next lines characters too

            # No more structs
            break

      # Beginning and End Sections are detected
      if (struct_detect == 2):
        del file_contents[struct_begin_line:struct_end_line]

        # Write back updated text
        with open(register_file, 'w') as file:
          file.writelines(file_contents)

      file.close()

  # Add your own exceptions
  except:
    print("Error with file: " + register_file + "\n")

#==================================================================================

# Peripheral name should be upper cased when passed in
def remove_peripheral_from_part_svd(svd_file, peripheral_name):
  peripheral_name.upper()
  print("    -Removing peripheral from SVD file: " + peripheral_name)

  beginning_token = "<name>" + peripheral_name + "</name>"

  # Get File Contents
  try:
    with open(svd_file, 'r') as file:
      file_contents = file.readlines()

      # Variable to indicate if structure exists.
      peripheral_detect = 0

      # Keep track of the lines of peripheral.
      peripheral_begin_line = 0
      peripheral_end_line = 0

      # Find line indexs for peripheral section
      for index, line in enumerate(file_contents):
        # Unique token to find beginning of peripheral section
        if (beginning_token in line):
          peripheral_detect += 1

          # Could've not added the "+ 1 - 2" but using it for future reference:
          # + 1 is for index since doesn't increment until next interation
          # - 2 is for the index at beginning of section
          peripheral_begin_line = index + 1 - 2

        # Find the end of the peripheral section
        if (peripheral_detect == 1):
          # This is the ending token of a section, find the first instance of this since there can't be
          # nested peripherals.
          if ("</peripheral>" in line) :
            peripheral_detect += 1

            # + 1 is for index since doesn't increment until next interation
            peripheral_end_line = index + 1

        # Check if next line is a comment that needs to be removed too
        if (peripheral_detect == 2):
          svd_comments = "<!--" + peripheral_name.upper()
          if (svd_comments in line):
            peripheral_detect += 1
            peripheral_end_line = index + 1

            # Should be end of section at this point
            break

      # Beginning and End Sections are detected
      if (peripheral_detect > 0):
        del file_contents[peripheral_begin_line:peripheral_end_line]

        # Write back updated text
        with open(svd_file, 'w') as file:
          file.writelines(file_contents)

      file.close()

  # Add your own exceptions
  except Exception:
    print("Error with file: " + svd_file + " Exception: " + str(Exception) +"\n")    

#==================================================================================

def remove_deprecated_fields_from_part_svd(svd_file):
  print("    -Removing deprecated fields from SVD file.")

  beginning_token = "<field deprecated="

  removal_done = 0
  # Get File Contents
  while (removal_done == 0):
    try:
      with open(svd_file, 'r') as file:
        file_contents = file.readlines()

        # Variable to indicate if structure exists.
        deprecated_field_detect = 0

        # Keep track of the lines of deprecated fields.
        deprecated_field_begin_line = 0
        deprecated_field_end_line = 0

        # Find line indexs for field section
        for index, line in enumerate(file_contents):
          # Unique token to find beginning of peripheral section
          if (beginning_token in line):
            deprecated_field_detect += 1
            print("-----")
            print(line + " Col#: " + str(index))

            # Could've not added the "+ 1 - 2" but using it for future reference:
            # + 1 is for index since doesn't increment until next interation
            # - 2 is for the index at beginning of section
            deprecated_field_begin_line = index + 1 - 2

          # Find the end of the peripheral section
          if (deprecated_field_detect == 1):
            # This is the ending token of a section, find the first instance of this since there can't be
            # nested fields.
            if ("</field>" in line) :
              deprecated_field_detect += 1
              print(line + " Col#: " + str(index))

              deprecated_field_end_line = index

              break

        # Beginning and End Sections are detected
        if (deprecated_field_detect > 0):
          del file_contents[deprecated_field_begin_line:deprecated_field_end_line]

          # Write back updated text
          with open(svd_file, 'w') as file:
            file.writelines(file_contents)
          print("Removed\n")

        if (deprecated_field_detect == 0):
          removal_done = 1
          print("Finished Removal.")

        file.close()

    # Add your own exceptions
    except Exception:
      print("Error with file: " + svd_file + " Exception: " + str(Exception) +"\n")    

#==================================================================================

usage_error =  "You can add support to remove more sections besides the struct.\n"
usage_error += "Usage svd_removal.py <part_name.svd> <parameter> <register_file_path>\n"
usage_error += "  -s to remove register struct\n"
usage_error += "  -p to remove peripheral from <part>.svd file\n"
usage_error += "  -r to remove register offsets [NOT YET SUPPORTED]\n"
usage_error += "  -f to remove deprecated fields from <part>.svd file\n"
usage_error += "  -rs for multiple removal parameters [FUTURE SUPPORT]\n"

global register_file

# Check inputs
if (os.path.isfile(sys.argv[1]) == 0):
  print("SVD file not found.")
  print(usage_error)
  sys.exit(0)

# You can add more options to remove
if (sys.argv[2] == 0):
  print("Removal parameter not found.")
  print(usage_error)
  sys.exit(0)

# Check inputs
if ('s' in sys.argv[2] and os.path.isfile(sys.argv[3]) == 0):
  print("Register file not found.")
  print(usage_error)
  sys.exit(0)  


svd_file = sys.argv[1]

if ('s' in sys.argv[2]):
  register_file = sys.argv[3]

# Default, set to structs
parameter = str(sys.argv[2])[1:]

# You can add more options to support
parameter_options = ["s", "p", "f"]

# Set removing structs as default parameter (only one when this was created)
removal_parameter = "s"

for i, single_parameter in enumerate(parameter):
  if (single_parameter not in parameter_options):
    print("    -Parameter not supported: " + single_parameter)
    print(usage_error)
    sys.exit(0)

removal_parameter = parameter

# Add function calling for new commands here.
if ("s" in removal_parameter):
  remove_register_struct(register_file)

if ("p" in removal_parameter):
  peripheral_name = get_peripheral_name(register_file).upper()
  remove_peripheral_from_part_svd(svd_file, peripheral_name)

if ("f" in removal_parameter):
  remove_deprecated_fields_from_part_svd(svd_file)
