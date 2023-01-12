# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable FreeRTOS library
LIB_FREERTOS = 1

# Enable CORDIO library
LIB_CORDIO = 1

TRACE = 1

# This application only operates as a peripheral
RTOS = freertos
INIT_PERIPHERAL = 1
INIT_BROADCASTER = 1
INIT_CENTRAL = 0
INIT_OBSERVER = 0

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

# Use local linkerfile
LINKERFILE = ota.ld
