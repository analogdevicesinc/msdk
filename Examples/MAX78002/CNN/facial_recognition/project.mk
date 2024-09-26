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

# **********************************************************

# Add your config here!

$(info Note: This project is designed and tested for the OV7692 only.)
override CAMERA=OV7692

# This project only supports the Newhaven NHD-2.4 TFT display
TFT = NEWHAVEN

# Enable TFT and touchscreen
PROJ_CFLAGS+=-DTFT_ENABLE
PROJ_CFLAGS+=-DTS_ENABLE
PROJ_CFLAGS +=-DTS_MAX_BUTTONS=32

# Add font
FONTS = LiberationSans16x16
