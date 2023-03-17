# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# CORDIO library options
INIT_PERIPHERAL = 1
INIT_BROADCASTER = 0
INIT_CENTRAL = 0
INIT_OBSERVER = 0

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os
