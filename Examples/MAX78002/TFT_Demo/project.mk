# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Uncomment to use legacy Adafruit 3315 TFT drivers (TFT = ADAFRUIT)
# Otherwise, default drivers (TFT = NEWHAVEN) will be used for NewHaven NHD-2.4
# TFT = ADAFRUIT

# Add TFT resources folder to build
VPATH += resources/tft
IPATH += resources/tft

