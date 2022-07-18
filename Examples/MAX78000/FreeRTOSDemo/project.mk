# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default project configuration isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available config variables.


RTOS_CONFIG_DIR=.
include $(MAXIM_PATH)/Libraries/FreeRTOS/freertos.mk

IPATH += $(LIBS_DIR)/FreeRTOS-plus/Source/FreeRTOS-Plus-CLI
VPATH += $(LIBS_DIR)/FreeRTOS-plus/Source/FreeRTOS-Plus-CLI

