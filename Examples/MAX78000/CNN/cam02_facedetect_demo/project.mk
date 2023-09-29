# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

MXC_OPTIMIZE_CFLAGS = -Og

BOARD = CAM02_RevA

ifneq ($(BOARD),CAM02_RevA)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX78000CAM02 board.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
endif
