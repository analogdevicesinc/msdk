# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# Set CORDIO library options
TOKEN = 0
BLE_CONTROLLER = 1

# TRACE option
# Set to 1 to enable serial port trace messages
# Set to 0 to disable
TRACE = 1

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os
