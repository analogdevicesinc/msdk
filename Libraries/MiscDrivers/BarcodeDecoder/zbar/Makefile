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

# This is the name of the build output file
PROJECT=libbarcode_decoder

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)
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
PROJ_CFLAGS:=$(filter-out -Wall, $(PROJ_CFLAGS))

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL := lib
