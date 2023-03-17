# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# If you have secure version of MCU (MAX32666), set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Enable FreeRTOS library
LIB_FREERTOS = 1

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
RTOS = freertos
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
# 
# Changes to this variable could require a rebuild of the Cordio library
#    # make clean.cordio
TRACE = 1
