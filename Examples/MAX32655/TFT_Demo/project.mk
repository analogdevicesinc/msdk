# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add resources folder to build
IPATH += resources
VPATH += resources

ifeq ($(BOARD),FTHR_Apps_P1)
$(error This example requires a TFT display, therefore it's not supported on the MAX32650FTHR)
endif
