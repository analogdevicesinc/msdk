# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

ENABLE_TFT = 1
# ^ Set to 1 to enable TFT display, or 0 to disable.

ifeq ($(ENABLE_TFT),1)
# Uncomment to use legacy Adafruit 3315 TFT drivers (TFT = ADAFRUIT)
# Otherwise, default drivers (TFT = NEWHAVEN) will be used for NewHaven NHD-2.4
# TFT = ADAFRUIT

PROJ_CFLAGS += -DENABLE_TFT

# Add TFT resources folder to build
VPATH += tft
IPATH += tft
endif
