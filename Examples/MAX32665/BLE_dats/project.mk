# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

# Disable central and observer.  They're
# not needed for this server app.
INIT_PERIPHERAL = 1
INIT_BROADCASTER = 1
INIT_CENTRAL = 0
INIT_OBSERVER = 0

# TRACE option
# Set to 1 to enable serial port trace messages
# Set to 0 to disable
TRACE = 1

# Add services directory to build
IPATH += services
VPATH += services
