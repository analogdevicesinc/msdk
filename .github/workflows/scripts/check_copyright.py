#! /usr/bin/env python3

###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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
 ##############################################################################

import subprocess
import re
import os.path

from datetime import datetime


command_git_diff = "git diff --ignore-submodules --name-only @ remotes/origin/main"
args = list(command_git_diff.split(" "))

# Run git command to find all changed files
git_command_ret_files = subprocess.run(args, capture_output=True, text=True)

# Place in list
changed_files = list(filter(None, git_command_ret_files.stdout.split("\n")))

current_year = datetime.now().year

files_with_incorrect_copyright = []

# Iterate through all files
for file in changed_files:
    try: 
        with open(file, 'r') as f:
            # Flag whether copyright exists
            has_copyright = False

            # Using enumerate because the copyright is always at the beginning of a file
            # so there's no point to search in rest of the file.
            for i, line in enumerate(f):
                # 30 is a generous limit to find the Copyright line
                #   Format: "Copyright (C) YEAR Analog Devices, Inc.".
                #   This should be on the same line.
                # Typically, the copyright line should exist within the first 15 lines of a file.
                if i == 30:
                    break

                # Print file if there's no copyright. File is printed so workflow bash can easily handle it.
                # Stop searching.
                if ("Copyright (C)" in line and "Analog Devices, Inc." in line and current_year in line):
                    has_copyright = true
                    break

            if not has_copyright:
                files_with_incorrect_copyright.append(file)
    except:
        print("error")

# Print list with only space delimiters for workflow to easily grab
print(*files_with_incorrect_copyright,sep=' ')

