# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default project configuration isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available config variables.

LIB_FREERTOS = 1

# FreeRTOS (Disabled by default)
# ************************
LIB_FREERTOS ?= 0
ifeq ($(LIB_FREERTOS), 1)
# Where to find FreeRTOSConfig.h
RTOS_CONFIG_DIR ?= .

# Include FreeRTOS-Plus-CLI
IPATH += $(LIBS_DIR)/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI
VPATH += $(LIBS_DIR)/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI
SRCS += FreeRTOS_CLI.c

# Include the FreeRTOS library
include $(LIBS_DIR)/FreeRTOS/freertos.mk
endif
# ************************
