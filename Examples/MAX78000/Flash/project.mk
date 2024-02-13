# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

ifeq ($(BOARD),CAM01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM01 board)
endif

ifeq ($(BOARD),CAM02_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM02 board)
endif
