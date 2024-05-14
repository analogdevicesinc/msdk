# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable debugging flags
DEBUG           ?= 1



# Sleep when the CPU is idle
# Will disconnect the debugger while sleeping. This can make debugging difficult. 
# Reset the board prior to debugging
ROLE ?= 1



PROJ_CFLAGS+= -DROLE=${ROLE}
PROJ_CFLAGS+= -DDEBUG=${DEBUG}


# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Enable Cordio library
LIB_CORDIO = 1



INIT_PERIPHERAL = 1
INIT_CENTRAL = 1



# Cordio library options

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
TRACE = 1

USE_DUAL_CORE ?= 0
ifeq "$(USE_DUAL_CORE)" "1"
BLE_HOST = 1
BLE_CONTROLLER = 0
endif
