###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################

################################################################################
# This file can be included in a project makefile to build the library for the 
# project.
################################################################################

ifeq "$(CORDIO_DIR)" ""
# If CORDIO_DIR is not specified, this Makefile will locate itself.
CORDIO_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../../../..
endif

TARGET_UC ?= $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC ?= $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))

# Get all of the necessary include paths
include ${CORDIO_DIR}/platform/targets/maxim/build/cordio.mk

# Build a library for each TARGET
CORDIO_LIB_VAR:=${TARGET_LC}

# Build for each RTOS
CORDIO_LIB_VAR:=${CORDIO_LIB_VAR}_${RTOS}_T${TRACE}

ifneq "$(RISCV_CORE)" ""
CORDIO_LIB_VAR:=${CORDIO_LIB_VAR}_riscv
endif

# Specify the library variant.
ifeq "$(MFLOAT_ABI)" "hardfp"
CORDIO_LIB_VAR:=${CORDIO_LIB_VAR}_hardfp
else
ifeq "$(MFLOAT_ABI)" "hard"
CORDIO_LIB_VAR:=${CORDIO_LIB_VAR}_hardfp
else
CORDIO_LIB_VAR:=${CORDIO_LIB_VAR}_softfp
endif
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
CORDIO_BUILD_DIR=${CORDIO_DIR}/bin/$(CORDIO_LIB_VAR)
else
CORDIO_BUILD_DIR=$(BUILD_DIR)/Cordio
endif

IPATH += ${INC_DIRS} # Variable from cordio.mk
CORDIO_C_FILES=${C_FILES} # Variable from cordio.mk

# APP_BUILD_C_FILES from cordio.mk
VPATH += %.c $(sort $(dir $(APP_BUILD_C_FILES)))
SRCS += ${APP_BUILD_C_FILES}
PROJ_CFLAGS += $(addprefix -D,$(sort $(CFG_DEV))) # Remove duplicates
PROJ_AFLAGS += -DPAL_NVM_SIZE=$(PAL_NVM_SIZE)

# Determine the library variant
ifeq "$(CORDIO_LIB_VAR)" ""
CORDIO_LIB := cordio.a
else
CORDIO_LIB := cordio_$(CORDIO_LIB_VAR).a
endif

export CORDIO_LIB
export CORDIO_BUILD_DIR

# Add to library list
LIBS += ${CORDIO_BUILD_DIR}/${CORDIO_LIB}

# Allow user to set Cordio optimization level if needed
CORDIO_OPTIMIZE_CFLAGS ?= ${MXC_OPTIMIZE_CFLAGS}

# Add rule to build the Driver Library
${CORDIO_BUILD_DIR}/${CORDIO_LIB}: ${CORDIO_C_FILES} ${PROJECTMK}
	$(MAKE) -f ${CORDIO_DIR}/platform/targets/maxim/build/libCordio.mk lib MAXIM_PATH=${MAXIM_PATH} PROJECT=${CORDIO_LIB} \
	CORDIO_LIB_VAR=${CORDIO_LIB_VAR} BUILD_DIR=${CORDIO_BUILD_DIR} MFLOAT_ABI=$(MFLOAT_ABI) \
	DUAL_CORE=$(DUAL_CORE) RISCV_CORE=$(RISCV_CORE) TRACE=${TRACE} DEBUG=${DEBUG} RTOS=${RTOS} \
	CFG_DEV="${CFG_DEV}" PROJECTMK=${PROJECTMK} BOARD=${BOARD} MXC_OPTIMIZE_CFLAGS=${CORDIO_OPTIMIZE_CFLAGS} \
	BT_VER=${BT_VER}

clean.cordio:
	@$(MAKE) -f ${CORDIO_DIR}/platform/targets/maxim/build/libCordio.mk BUILD_DIR=${CORDIO_BUILD_DIR} clean
