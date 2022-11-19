# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#MXC_OPTIMIZE_CFLAGS = -Og
# ^ For example, you can uncomment this line to 
# optimize the project for debugging

# **********************************************************

# Select TFT drivers to match the connected display 
TFT=ADAFRUIT
# TFT=NEWHAVEN

# Add TFT resources folder to build
VPATH += resources/tft
IPATH += resources/tft

