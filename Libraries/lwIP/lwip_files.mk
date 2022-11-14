################################################################################
 # Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 # $Date: 2018-08-31 14:08:14 -0500 (Fri, 31 Aug 2018) $
 # $Revision: 37586 $
 #
 ###############################################################################

# This is the name of the build output file

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)
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
LWIP_H_FILES +=  $(shell find $(LWIP_INCLUDE_DIR) -name '*.h')
