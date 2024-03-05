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
PROJECT=libbarcode_decoder

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC := $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC := $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))
$(info $(TARGET_UC))

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

# This is the path to the peripheral drivers
ifeq "$(PERIPH_DRIVER_DIR)" ""
PERIPH_DRIVER_DIR=../PeriphDrivers/$(TARGET_UC)
endif

# Source files for this test (add path to VPATH below)
SRCS  = src/decoder.c  
SRCS += src/error.c
SRCS += src/image.c 
SRCS += src/img_scanner.c
SRCS += src/refcnt.c 
SRCS += src/scanner.c 
SRCS += src/sqcode.c  
SRCS += src/symbol.c 

SRCS += src/decoder/codabar.c 
SRCS += src/decoder/code128.c 
SRCS += src/decoder/code39.c 
SRCS += src/decoder/code93.c 
SRCS += src/decoder/databar.c 
SRCS += src/decoder/ean.c 
SRCS += src/decoder/i25.c 
SRCS += src/decoder/pdf417.c 
SRCS += src/decoder/qr_finder.c 
SRCS += src/decoder/sq_finder.c

SRCS += src/qrcode/bch15_5.c 
SRCS += src/qrcode/binarize.c 
SRCS += src/qrcode/isaac.c  
SRCS += src/qrcode/qrdec.c 
SRCS += src/qrcode/qrdectxt.c  
SRCS += src/qrcode/rs.c 
SRCS += src/qrcode/util.c

# Where to find source files for this project
VPATH = ${BARCODE_DECODER_DIR}/src
VPATH += ${BARCODE_DECODER_DIR}/src/decoder
VPATH += ${BARCODE_DECODER_DIR}/src/qrcode

# Where to find header files for this project
IPATH = include
IPATH += src
IPATH += src/decoder
IPATH += src/qrcode
IPATH += $(PERIPH_DRIVER_DIR)/Include/$(TARGET_UC)

# libiconv
SRCS  += libiconv/iconv.c
SRCS  += libiconv/relocatable.c
VPATH += ${BARCODE_DECODER_DIR}/libiconv
IPATH += ${BARCODE_DECODER_DIR}/libiconv

# Localcharset
SRCS  += ./localcharset/localcharset.c
VPATH += ${BARCODE_DECODER_DIR}/localcharset
IPATH += ${BARCODE_DECODER_DIR}/localcharset


# Open compile guard flag
PROJ_CFLAGS+=-D__$(TARGET_UC)

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL := lib
