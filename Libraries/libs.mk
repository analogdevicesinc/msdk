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
INIT_PERIPHERAL ?= 1
INIT_BROADCASTER?= 1

# For the Controller in the RISC-V core in the split host and controller
ifeq "$(RISCV_CORE)" "1"
ifeq "$(BLE_CONTROLLER)" "1"
ifeq "$(HOST_PROJECT)" "BLE_fit"
INIT_ENCRYPTED = 0
$(info ---INFO INIT_ENCRYPTED=$(INIT_ENCRYPTED))
endif  # PROJECT
endif  # BLE_CONTROLLER
endif  # RISCV_CORE

# Enter standby mode when idle
STANDBY_ENABLED ?= 0

# Select either option, or both for combined Host and Controller on single core
BLE_HOST        ?= 1
BLE_CONTROLLER  ?= 1

ifneq "$(BLE_HOST)" ""
ifneq "$(BLE_HOST)" "0"
ifneq "$(BLE_CONTROLLER)" "1"
RISCV_LOAD = 1
RISCV_APP ?= ../BLE4_ctr
endif
endif
endif

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

# NFC (Disabled by default)
# Only available via NDA
# ************************
LIB_NFC ?= 0
ifeq ($(LIB_NFC), 1)

# NFC lib has two components, pcd_pbm and rf_driver
LIB_NFC_PCD_PBM_DIR ?= $(LIBS_DIR)/NFC/lib_nfc_pcd_pbm
LIB_NFC_PCD_RF_DRIVER_DIR ?= $(LIBS_DIR)/NFC/lib_nfc_pcd_rf_driver_$(TARGET_UC)

ifeq ("$(wildcard $(LIB_NFC_PCD_PBM_DIR))","")
$(warning Warning: Failed to locate $(LIB_NFC_PCD_PBM_DIR))
$(error NFC libraries not found.  Please install the NFC package to $(LIBS_DIR)/NFC)
endif

ifeq ("$(wildcard $(LIB_NFC_PCD_RF_DRIVER_DIR))","")
$(warning Warning: Failed to locate $(LIB_NFC_PCD_RF_DRIVER_DIR))
$(error NFC libraries not found.  Please install the NFC package to $(LIBS_DIR)/NFC)
endif

ifneq ($(DEV_LIB_NFC),1)
# The libraries are released as pre-compiled library files.
# Only need to set up include paths and link library

# Add to include directory list
IPATH += $(LIB_NFC_PCD_PBM_DIR)/include
PROJ_LDFLAGS += -L$(LIB_NFC_PCD_PBM_DIR)
PROJ_LIBS += nfc_pcd_pbm_$(LIBRARY_VARIANT)

# Add to include directory list
IPATH += $(LIB_NFC_PCD_RF_DRIVER_DIR)/include
IPATH += $(LIB_NFC_PCD_RF_DRIVER_DIR)/include/nfc
PROJ_LDFLAGS += -L$(LIB_NFC_PCD_RF_DRIVER_DIR)
PROJ_LIBS += nfc_pcd_rf_driver_MAX32570_$(LIBRARY_VARIANT)

else
# Development setup (DEV_LIB_NFC=1) for building libraries
# from source
include $(LIB_NFC_PCD_PBM_DIR)/nfc_pcd_pbm.mk
include $(LIB_NFC_PCD_RF_DRIVER_DIR)/nfc_pcd_rf_driver.mk
endif

endif
# ************************

# EMV (Disabled by default)
# Only available via NDA
# ************************
LIB_EMV ?= 0
ifeq ($(LIB_EMV), 1)
EMV_DIR ?= $(LIBS_DIR)/EMV

ifeq ("$(wildcard $(EMV_DIR))","")
$(error EMV library not found. Please install the EMV package to $(EMV_DIR))
endif

include $(EMV_DIR)/emv.mk
endif
# ************************
