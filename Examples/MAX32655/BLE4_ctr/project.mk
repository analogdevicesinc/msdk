# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# Set CORDIO library options
TOKEN = 0
BT_VER = 8
BLE_CONTROLLER = 1

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os
