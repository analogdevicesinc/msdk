# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

CAMERA = HM0360_COLOR

BOARD = CAM01_RevA

ifneq ($(BOARD),CAM01_RevA)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX78000CAM01 board.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
endif
