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

# This example is only compatible with the FTHR board
BOARD = FTHR_RevA
ifneq ($(BOARD),FTHR_RevA)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX78000FTHR.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
endif

LIB_SDHC = 1

