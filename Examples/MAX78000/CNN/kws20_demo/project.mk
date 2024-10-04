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

# Uncomment the line below to build for the MAX78000FTHR
#BOARD=FTHR_RevA

# Set a higher optimization level.  The increased performance
# is required for the CameraIF DMA code to work within the
# timing requirements of the Parallel Camera Interface.
MXC_OPTIMIZE_CFLAGS = -O2

# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
PROJ_CFLAGS+=-DTFT_ENABLE
IPATH += TFT/evkit/
VPATH += TFT/evkit/
endif

# If enabled, it sends out the Mic samples used for inference to the serial port
#PROJ_CFLAGS+=-DSEND_MIC_OUT_SERIAL

# Place build files specific to FTHR_RevA here.
ifeq "$(BOARD)" "FTHR_RevA"
# Only Enable if 2.4" TFT is connected to Feather
#PROJ_CFLAGS+=-DTFT_ENABLE

# If enabled, it saves out the Mic samples used for inference to SDCARD
# Note that if both SDCARD and TFT are enabled, the TFT will be disabled to avoid SPI driver conflict.
#PROJ_CFLAGS+=-DSEND_MIC_OUT_SDCARD

# If enabled, it captures audio from line input of MAX9867 audio codec instead of the on-board mic.
# Note that SEND_MIC_OUT_SDCARD should be disabled in this mode
#PROJ_CFLAGS+=-DENABLE_CODEC_MIC
LIB_SDHC = 1
IPATH += TFT/fthr
VPATH += TFT/fthr
FONTS = LiberationSans16x16
endif

ifeq ($(BOARD),CAM01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM01 board)
endif

ifeq ($(BOARD),CAM02_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM02 board)
endif


