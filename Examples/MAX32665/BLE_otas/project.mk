# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# Cordio library options
STANDBY_ENABLED = 0
INIT_PERIPHERAL = 1
INIT_BROADCASTER = 1
INIT_CENTRAL = 0
INIT_OBSERVER = 0


# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

AUTOSEARCH=0
VPATH += .
SRCS += $(wildcard $(addsuffix /*.c, $(VPATH)))

USE_INTERNAL_FLASH ?=0
ifeq ($(USE_INTERNAL_FLASH), 1)
LINKERFILE = ota_internal_mem.ld
SRCS:=$(subst wdxs_file_ext.c,wdxs_file_int.c,${SRCS})
else
LINKERFILE = ota_external_mem.ld
SRCS:=$(subst wdxs_file_int.c,wdxs_file_ext.c,${SRCS})

endif