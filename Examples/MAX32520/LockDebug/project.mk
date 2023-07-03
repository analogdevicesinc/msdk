# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

ifeq ($(BOARD),MAX32520FTHR)
$(error ERR_NOTSUPPORTED: This example requires a pushbutton, therefore it is not supported on the MAX32520FTHR)
endif
