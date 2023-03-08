# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

ifeq ($(BOARD),MAX32520FTHR)
$(error This example requires a pushbutton, therefore it is not supported on the MAX32520FTHR)
endif
