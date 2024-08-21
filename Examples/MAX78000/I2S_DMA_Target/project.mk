# This file can be used to set build configuration
# variables.  These variables are defined in a file called
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# This example is only compatible with the FTHR board,
# so we override the BOARD value to hard-set it.
ifneq ($(BOARD),FTHR_RevA)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX78000FTHR board.  See https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages)
endif

