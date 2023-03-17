# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
# 
# Changes to this variable could require a rebuild of the Cordio library
#    # make clean.cordio
TRACE = 0

DEBUG = 0

# Add services directory to build
IPATH += services
VPATH += services
