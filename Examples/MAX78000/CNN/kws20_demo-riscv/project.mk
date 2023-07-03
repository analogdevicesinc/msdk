# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
PROJ_CFLAGS+=-DENABLE_TFT
endif

# Place build files specific to FTHR_RevA here.
ifeq "$(BOARD)" "FTHR_RevA"
# Only Enable if 2.4" TFT is connected to Feather
#PROJ_CFLAGS+=-DENABLE_TFT
endif

ifeq ($(BOARD),CAM01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the CAM01 board)
endif

