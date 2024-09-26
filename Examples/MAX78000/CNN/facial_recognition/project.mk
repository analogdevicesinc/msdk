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
# https://github.com/analogdevicesinc/VSCode-Maxim/tree/develop#build-configuration

#BOARD=FTHR_RevA
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_RevA" board.

# **********************************************************

# Add your config here!

# Project config options (see README):
# -----------------------
TFT_ENABLE = 0
# -----------------------

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# This example is only compatible with the FTHR board
BOARD := FTHR_RevA

ifneq "$(BOARD)" "FTHR_RevA"
define ERR_MSG
ERR_NOTSUPPORTED: 
This project is only supported on the MAX78000FTHR (FTHR_RevA)
See https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages
endef
$(error $(ERR_MSG))
endif

# Place build files specific to FTHR_RevA here.
SDHC_CLK_FREQ  = 25000000

ifeq "$(TFT_ENABLE)" "1"
# Only Enable if 2.4" TFT is connected to Feather
PROJ_CFLAGS+=-DTFT_ENABLE
IPATH += TFT/fthr
VPATH += TFT/fthr
FONTS = LiberationSans16x16
endif

# Enable the SDHC library
LIB_SDHC = 1
# Include a custom ffconf.h file that disables exFAT support.  exFAT support requires additional flash and RAM.
# - FF_FS_EXFAT = 0
FATFS_VERSION = ff15
FF_CONF_DIR = ./include
