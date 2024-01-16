###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
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


LWIP_DIR := $(LIBS_DIR)/lwIP
PERIPH_DRIVER := $(LIBS_DIR)/PeriphDrivers
SOURCE_DIR := $(LWIP_DIR)/Source
INCLUDE_DIR := $(LWIP_DIR)/Include

LWIP_INCLUDE_DIR += $(LWIP_DIR)/include
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/compat
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/compat/posix
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/compat/posix/arpa
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/compat/posix/net
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/compat/posix/sys
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/compat/stdc
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/lwip
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/lwip/apps
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/lwip/priv
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/lwip/prot
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/Maxim
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/Maxim/arch
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/netif
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/netif/ppp
LWIP_INCLUDE_DIR += $(LWIP_DIR)/include/netif/ppp/polarssl
LWIP_INCLUDE_DIR += $(PERIPH_DRIVER)/Include/$(TARGET_UC)

LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/api/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/core/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/core/ipv4/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/core/ipv6/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/netif/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/netif/ppp/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/netif/ppp/polarssl/*.c))
LWIP_C_FILES += $(sort $(wildcard $(LWIP_DIR)/Maxim/*.c))

# Where to find header files for this project
LWIP_H_FILES += $(wildcard $(addsuffix /*.h,$(LWIP_INCLUDE_DIR)))
