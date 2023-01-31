# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable FreeRTOS library
LIB_FREERTOS = 1

# Un-comment this line to enable tickless mode, standby mode between events
# PROJ_CFLAGS += -DUSE_TICKLESS_IDLE=1

# Enable CORDIO library
LIB_CORDIO = 1

# Set to zero to minimize code size
DEBUG = 1

# Set to zero to minimize code size
TRACE = 1

# This application only operates as a peripheral
RTOS = freertos
BT_VER = 8
INIT_PERIPHERAL = 1
INIT_BROADCASTER = 0
INIT_CENTRAL = 0
INIT_OBSERVER = 0


# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

AUTOSEARCH=0
VPATH += .
SRCS += stack_dats.c 
SRCS += dats_main.c
SRCS += main.c
SRCS += freertos_tickless.c

USE_INTERNAL_FLASH ?= 1

ifeq ($(USE_INTERNAL_FLASH), 1)
LINKERFILE = ota_internal_mem.ld
SRCS += wdxs_file_int.c
else
LINKERFILE = ota_external_mem.ld
SRCS += wdxs_file_ext.c
endif
