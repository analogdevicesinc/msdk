# This file is used by the Makefile system to build source code
# for the Secure Boot Tools (SBT).  It's imported by the core
# Makefile for each example project.

ifeq "$(TARGET_UC)" "MAX32650"
TARGET_SEC:=MAX32651
endif

CA_SIGN_BUILD = $(MAXIM_SBT_DIR)/bin/sign_app
BUILD_SESSION = $(MAXIM_SBT_DIR)/bin/build_scp_session

ifeq ($(OS), Windows_NT)
CA_SIGN_BUILD := $(CA_SIGN_BUILD).exe
BUILD_SESSION := $(BUILD_SESSION).exe
# Must use .exe extension on Windows
endif

CA_SIGN_BUILD := "$(CA_SIGN_BUILD)"
BUILD_SESSION := "$(BUILD_SESSION)"
TEST_KEY="$(MAXIM_SBT_DIR)/devices/$(TARGET_SEC)/keys/maximtestcrk.key"
# ^ Have to surround these with quotes, because the SBT may install to C:/Program Files (x86) on Windows,
# and the spaces in the filepaths break the Makefile and SBT.  Surrounding with quotes
# gets everything working.  TODO: Does this work on Linux???

SCP_PACKETS=$(BUILD_DIR)/scp_packets
# ^ Generate scp packets in the build directory

ifeq ($(MAKECMDGOALS),sla)
PROJ_CFLAGS += -D__SLA_FWK__
SRCS += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/sla_header.c
# ^ Include the SLA C file for the device.
LINKERFILE = $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_sla.ld
# ^ Override linkerfile.
endif

ifeq ($(MAKECMDGOALS),ram)
PROJ_CFLAGS += -D__RAM_FWK__
LINKERFILE = $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_ram.ld
# ^ Override linkerfile.
endif

ifeq ($(MAKECMDGOALS), scpa)
PROJ_CFLAGS += -D__SCPA_FWK__
PROJ_CFLAGS += -DSCPA_MEM_BASE_ADDR=0xC0000000 -DSCPA_MEM_SIZE=1024
LINKERFILE = $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_scpa.ld
# ^ Override linkerfile.
endif
