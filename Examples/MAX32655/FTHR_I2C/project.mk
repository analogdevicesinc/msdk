# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
override BOARD=FTHR_Apps_P1

# This example is only compatible with the MAX32655FTHR
ifneq ($(BOARD),FTHR_Apps_P1)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX32655FTHR.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
endif
