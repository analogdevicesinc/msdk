################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 ###############################################################################

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

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER

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

# Add rule to build the Driver Library
${CORDIO_BUILD_DIR}/${CORDIO_LIB}: ${CORDIO_C_FILES} # Variable from cordio.mk
	$(MAKE) -f ${CORDIO_DIR}/platform/targets/maxim/build/libCordio.mk lib PROJECT=${CORDIO_LIB} \
	CORDIO_LIB_VAR=${CORDIO_LIB_VAR} BUILD_DIR=${CORDIO_BUILD_DIR} MFLOAT_ABI=$(MFLOAT_ABI) \
	DUAL_CORE=$(DUAL_CORE) RISCV_CORE=$(RISCV_CORE) TRACE=${TRACE} DEBUG=${DEBUG} RTOS=${RTOS} \
    

clean.cordio:
	@rm -rf ${CORDIO_BUILD_DIR}/*
