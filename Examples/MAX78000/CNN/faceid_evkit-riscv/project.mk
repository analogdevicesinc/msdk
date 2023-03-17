# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Specify the board used
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
#BOARD=FTHR_RevA
endif

# Place build files specific to EvKit_V1 here.
ifeq "$(BOARD)" "EvKit_V1"
PROJ_CFLAGS+=-DROTATE_SCREEN=1
PROJ_CFLAGS+=-DTFT_ENABLE
PROJ_CFLAGS+=-DTS_ENABLE
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

ifeq ($(BOARD),Aud01_RevA)
$(error ERR_NOTSUPPORTED: This project is not supported for the Audio board)
endif