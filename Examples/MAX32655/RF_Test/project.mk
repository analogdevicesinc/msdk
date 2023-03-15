# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Enable Cordio library
LIB_CORDIO = 1

# Enable the FreeRTOS library
LIB_FREERTOS=1

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

# Enable low level trace
TRACE = 2