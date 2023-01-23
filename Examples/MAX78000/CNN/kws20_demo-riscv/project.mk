# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Specify the board used
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
#BOARD=FTHR_RevA
endif

# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
PROJ_CFLAGS+=-DENABLE_TFT
endif

# Place build files specific to FTHR_RevA here.
ifeq "$(BOARD)" "FTHR_RevA"
# Only Enable if 2.4" TFT is connected to Feather
#PROJ_CFLAGS+=-DENABLE_TFT
endif

# Set a higher optimization level to maximize performance
MXC_OPTIMIZE_CFLAGS = -O2
# Default optimization level for debugging purpose
#MXC_OPTIMIZE_CFLAGS = -Og
