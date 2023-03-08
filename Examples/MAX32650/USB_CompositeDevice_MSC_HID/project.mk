# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#BOARD=FTHR_RevA
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_RevA" board.

# **********************************************************

# Add your config here!

# If you have secure version of MCU (MAX32651), set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Enable MAXUSB library
LIB_MAXUSB=1

ifeq ($(BOARD),FTHR_APPS_A)
$(error This example requires an an external flash IC, therefore it's not supported on the MAX32650FTHR)
endif
