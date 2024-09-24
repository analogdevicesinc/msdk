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
# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

#BOARD=MAX32520FTHR
# ^ For example, you can uncomment this line to make the 
# project build for the "MAX32520FTHR" board.

# **********************************************************

# Enable UCL library
LIB_UCL = 1
export UCL_VERSION=2.7.0

# Add project's include and source paths
VPATH += ./src
VPATH += ./src/cipher
VPATH += ./src/public_key
VPATH += ./src/mac

IPATH += ./src/include
