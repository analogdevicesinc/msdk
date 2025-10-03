###############################################################################
 #
 # Copyright (C) 2025 Analog Devices, Inc.
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

# **********************************************************

# Enable Cordio library
LIB_CORDIO = 1

CORDIO_DIR = $(LIBS_DIR)/Packetcraft-ADI
LIB_PHY_DIR ?= $(LIBS_DIR)/RF-PHY

# Cordio library options
BLE_HOST = 0
BLE_CONTROLLER = 1

# TRACE option
# Set to 0 to disable
# Set to 2 to enable serial port trace messages
TRACE = 0
DEBUG = 1
