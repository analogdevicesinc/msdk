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

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# Uncomment the line below to build for the MAX78000FTHR
#BOARD=FTHR_RevA

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

# Add some additional directories to the project based on
# what board we're building for...
ifeq "$(BOARD)" "EvKit_V1"
VPATH += TFT/evkit
endif
ifeq "$(BOARD)" "FTHR_RevA"
FONTS = LiberationSans16x16
endif

IPATH += TFT/evkit

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif

ifeq ($(BOARD),CAM01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM01 board)
endif

ifeq ($(BOARD),CAM02_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM02 board)
endif


