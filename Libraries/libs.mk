# This Makefile is used to manage the inclusion of the various
# libraries that are available in the MaximSDK.  'include'-ing 
# libs.mk offers 'toggle switch' variables that can be used to
# manage the inclusion of the available libraries.

# Each library below may also have its own set of configuration
# variables that can be overridden.

# If LIBS_DIR is not specified, this Makefile will locate itself.
LIBS_DIR ?= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# BSP (Enabled by default)
# ************************
LIB_BOARD ?= 1
ifeq ($(LIB_BOARD), 1)
BOARD_DIR := $(LIBS_DIR)/Boards/$(TARGET_UC)/$(BOARD)
include $(BOARD_DIR)/board.mk
endif
# ************************

# PeriphDrivers (Enabled by default)
# ************************
LIB_PERIPHDRIVERS ?= 1
ifeq ($(LIB_PERIPHDRIVERS), 1)
PERIPH_DRIVER_DIR := $(LIBS_DIR)/PeriphDrivers
include $(PERIPH_DRIVER_DIR)/periphdriver.mk
endif
# ************************

# CMSIS-DSP (Disabled by default)
# ************************
LIB_CMSIS_DSP ?= 0
ifeq ($(LIB_CMSIS_DSP), 1)
# Include the CMSIS-DSP library
include $(LIBS_DIR)/CMSIS/5.9.0/DSP/CMSIS-DSP.mk
endif
# ************************

# Cordio (Disabled by default)
# ************************
LIB_CORDIO ?= 0
ifeq ($(LIB_CORDIO), 1)
# Cordio Library Options
DEBUG           ?= 1
TRACE           ?= 1
BT_VER          ?= 9
INIT_CENTRAL    ?= 1
INIT_OBSERVER   ?= 1
INIT_ENCRYPTED  ?= 1

# Enter standby mode when idle
STANDBY_ENABLED ?= 0

# Select either option, or both for combined Host and Controller on single core
BLE_HOST        ?= 1
BLE_CONTROLLER  ?= 1

# Disable these trace messages for the speed testing
PROJ_CFLAGS += -DATT_TRACE_ENABLED=0 -DHCI_TRACE_ENABLED=0

# Include the Cordio Library
CORDIO_DIR ?= $(LIBS_DIR)/Cordio
include $(CORDIO_DIR)/platform/targets/maxim/build/cordio.mk
endif
# ************************

# FCL (Disabled by default)
# ************************
LIB_FCL ?= 0
ifeq ($(LIB_FCL), 1)
FCL_DIR  ?= $(LIBS_DIR)/FCL
include $(FCL_DIR)/fcl.mk
endif
# ************************

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

# LC3 (Disabled by default)
# ************************
LIB_LC3 ?= 0
ifeq ($(LIB_LC3), 1)
LC3_ROOT ?= $(LIBS_DIR)/LC3
include $(LC3_ROOT)/build/sources.mk
endif
# ************************

# littleFS (Disabled by default)
# ************************
LIB_LITTLEFS ?= 0
ifeq ($(LIB_LITTLEFS), 1)
LITTLEFS_DIR ?= $(LIBS_DIR)/littlefs
include $(LITTLEFS_DIR)/littlefs.mk
endif
# ************************

# lwIP (Disabled by default)
# ************************
LIB_LWIP ?= 0
ifeq ($(LIB_LWIP), 1)
LWIP_DIR ?= $(LIBS_DIR)/lwIP
include $(LWIP_DIR)/lwip.mk
endif
# ************************

# MAXUSB (Disabled by default)
# ************************
LIB_MAXUSB ?= 0
ifeq ($(LIB_MAXUSB), 1)
MAXUSB_DIR ?= $(LIBS_DIR)/MAXUSB
include $(MAXUSB_DIR)/maxusb.mk
endif
# ************************

# SDHC (Disabled by default)
# ************************
LIB_SDHC ?= 0
ifeq ($(LIB_SDHC), 1)
# Set the SDHC driver directory
SDHC_DRIVER_DIR ?= $(LIBS_DIR)/SDHC

# Set the FAT32 driver directory
FAT32_DRIVER_DIR ?= $(SDHC_DRIVER_DIR)/ff13

# Include the SDHC library
include $(FAT32_DRIVER_DIR)/fat32.mk
include $(SDHC_DRIVER_DIR)/sdhc.mk
endif
# ************************