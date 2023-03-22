# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Build the FreeRTOS Library
LIB_FREERTOS = 1

# Tell make file where to find the I2C Manager source files
VPATH += i2c_mngr
IPATH += i2c_mngr