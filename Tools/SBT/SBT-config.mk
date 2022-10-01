# This file is used by the Makefile system to build source code
# for the Secure Boot Tools (SBT).  It's imported by the core
# Makefile for each example project.

# Set the default secure target.  Different microcontrollers
# may have different secure variants, each with their own
# part numbers.  TARGET_SEC can be used to point the SBTs
# to the correct variant.
ifeq "$(TARGET_UC)" "MAX32650"
TARGET_SEC ?= MAX32651
else ifeq "$(TARGET_UC)" "MAX32665"
TARGET_SEC ?= MAX32666
else
TARGET_SEC ?= $(TARGET_UC)
endif

# Locate the SBT binaries.
CA_SIGN_BUILD = $(MAXIM_SBT_DIR)/bin/sign_app
BUILD_SESSION = $(MAXIM_SBT_DIR)/bin/build_scp_session

ifeq ($(OS), Windows_NT)
# Must use .exe extension on Windows, since the binaries
# for Linux may live in the same place.
CA_SIGN_BUILD := $(CA_SIGN_BUILD).exe
BUILD_SESSION := $(BUILD_SESSION).exe
endif

# Surround the tools with quotes, because the SBT may install to C:/Program Files (x86) on Windows,
# and the spaces in the filepaths break the Makefile and SBT.
CA_SIGN_BUILD := "$(CA_SIGN_BUILD)"
BUILD_SESSION := "$(BUILD_SESSION)"
TEST_KEY="$(MAXIM_SBT_DIR)/devices/$(TARGET_SEC)/keys/maximtestcrk.key"

# Generate SCP packets inside of the build directory by default
SCP_PACKETS?=$(BUILD_DIR)/scp_packets

# Based on the Make goal the source code may need some specific compiler
# definitions and files.  Additionally, the linkerfile may need to be changed.
ifeq ($(MAKECMDGOALS),sla)
PROJ_CFLAGS += -D__SLA_FWK__
SRCS += $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/sla_header.c
# ^ Include the SLA C file for the device.
LINKERFILE = $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_sla.ld
$(info Overriding LINKERFILE to $(LINKERFILE))
endif

ifeq ($(MAKECMDGOALS), scpa)
SCPA_MEM_BASE_ADDR ?= 0xC0000000
SCPA_MEM_SIZE ?= 1024

PROJ_CFLAGS += -D__SCPA_FWK__
PROJ_CFLAGS += -DSCPA_MEM_BASE_ADDR=$(SCPA_MEM_BASE_ADDR) -DSCPA_MEM_SIZE=$(SCPA_MEM_SIZE)
LINKERFILE = $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/GCC/$(TARGET_LC)_scpa.ld
$(info Overriding LINKERFILE to $(LINKERFILE))
endif
