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
