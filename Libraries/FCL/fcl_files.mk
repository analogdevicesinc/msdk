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

# This is the name of the build output file

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))
ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif


# This is the path to the CMSIS root directory
ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../CMSIS
endif
ifeq "$(LIBS_DIR)" ""
LIBS_DIR = $(CMSIS_ROOT)/..
endif


FCL_DIR := $(LIBS_DIR)/FCL
PERIPH_DRIVER := $(LIBS_DIR)/PeriphDrivers
SOURCE_DIR := $(FCL_DIR)/Source
INCLUDE_DIR := $(FCL_DIR)/Include

FCL_INCLUDE_DIR += $(FCL_DIR)/include
FCL_INCLUDE_DIR += $(FCL_DIR)/src/include
FCL_INCLUDE_DIR += $(PERIPH_DRIVER)/Include/$(TARGET_UC)

FCL_C_FILES += $(sort $(wildcard $(FCL_DIR)/src/*.c))
# Where to find header files for this project
FCL_H_FILES +=  $(wildcard $(addsuffix /*.h,$(FCL_INCLUDE_DIR)))
