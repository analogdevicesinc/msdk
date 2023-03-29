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

ifeq "$(BARCODE_DECODER_DIR)" ""
$(error BARCODE_DECODER_DIR must be specified")
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
BARCODE_BUILD_DIR=$(CURDIR)/build/lib_barcode_decoder
else
BARCODE_BUILD_DIR=$(BUILD_DIR)/lib_barcode_decoder
endif

# Export paths needed by the peripheral driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
BARCODE_BUILD_DIR := ${abspath ${BARCODE_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export PERIPH_DRIVER_DIR := ${abspath ${PERIPH_DRIVER_DIR}}


# Add to library list
LIBS += ${BARCODE_BUILD_DIR}/libbarcode_decoder.a

# Add to include directory list
IPATH += ${BARCODE_DECODER_DIR}/include
IPATH += ${BARCODE_DECODER_DIR}/src
IPATH += ${BARCODE_DECODER_DIR}/src/decoder
IPATH += ${BARCODE_DECODER_DIR}/src/qrcode

# Add rule to build the Barcode decoder Library
${BARCODE_BUILD_DIR}/libbarcode_decoder.a: FORCE
	$(MAKE) -C ${BARCODE_DECODER_DIR} lib BUILD_DIR="${BARCODE_BUILD_DIR}" TARGET="${TARGET}" COMPILER="${COMPILER}" PROJ_CFLAGS="${PROJ_CFLAGS} -Wno-parentheses" PROJ_LDFLAGS="${PROJ_LDFLAGS}" MXC_OPTIMIZE_CFLAGS="${MXC_OPTIMIZE_CFLAGS}" BARCODE_DECODER_DIR="${BARCODE_DECODER_DIR}"
