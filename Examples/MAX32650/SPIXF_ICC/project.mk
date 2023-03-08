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

ifeq ($(BOARD),FTHR_APPS_A)
$(error This project requires an external flash IC, therefore it's not supported on the MAX32650FTHR)
endif

# Override the default linkerfile
LINKERFILE=$(TARGET_LC)_spix.ld
