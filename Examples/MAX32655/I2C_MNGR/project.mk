# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add config here

# This example is only compatible with the MAX32655EVKIT
ifneq ($(BOARD),EvKit_V1)
$(error ERR_NOTSUPPORTED: This project is only supported on the MAX32655EVKIT.  (see https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages))
endif

# Build the FreeRTOS Library
LIB_FREERTOS = 1

# Tell make file where to find the I2C Manager source files
VPATH += i2c_mngr
IPATH += i2c_mngr