# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

BOARD = FTHR_RevA

ifneq "$(BOARD)" "FTHR_RevA"
$(error ERR_NOTSUPPORTED: This example is only supported on the MAX78000FTHR board!  See https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages)
endif
