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
 
ifeq "$(FCL_DIR)" ""
$(error FCL_DIR must be specified")
endif

TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))

# Specify the library variant.
ifeq "$(MFLOAT_ABI)" "hardfp"
LIBRARY_VARIANT=hardfp
else
ifeq "$(MFLOAT_ABI)" "hard"
LIBRARY_VARIANT=hardfp
else
LIBRARY_VARIANT=softfp
endif
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
FCL_BUILD_DIR=${FCL_DIR}/bin/$(LIBRARY_VARIANT)
else
FCL_BUILD_DIR=$(BUILD_DIR)/FCL
endif

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS
export MXC_OPTIMIZE_CFLAGS

include ${FCL_DIR}/fcl_files.mk
IPATH += ${FCL_INCLUDE_DIR}
ifeq "$(LIBRARY_VARIANT)" ""
FCL_LIB := libfcl.a
else
FCL_LIB := libfcl_$(LIBRARY_VARIANT).a
endif

# export FCL_DIR
export FCL_LIB
export FCL_BUILD_DIR

# Add to library list
LIBS += ${FCL_BUILD_DIR}/${FCL_LIB}
# Add rule to build the Driver Library
${FCL_BUILD_DIR}/${FCL_LIB}: ${FCL_C_FILES} ${FCL_H_FILES}
	$(MAKE) -f ${FCL_DIR}/libfcl.mk  lib BUILD_DIR=${FCL_BUILD_DIR} 

clean.fcl:
	@rm -rf ${FCL_BUILD_DIR}/*