################################################################################
 # Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
ifeq "$(MFLOAT_ABI)" ""
LWIP_BUILD_DIR = ${LWIP_DIR}/bin
else
LWIP_BUILD_DIR = ${LWIP_DIR}/bin/$(MFLOAT_ABI)
endif
else
LWIP_BUILD_DIR=$(BUILD_DIR)/lwIP
endif

# Set the output filename
# This is used in the PROJECT variable in the recursive make rule
ifneq "$(MFLOAT_ABI)" ""
LWIP_LIBRARY_NAME = liblwip_$(MFLOAT_ABI)
else
LWIP_LIBRARY_NAME = liblwip
endif

# Form the filepath to the output library file (note the addition of the .a extension here)
LWIP_LIBRARY_FILE = $(LWIP_BUILD_DIR)/$(LWIP_LIBRARY_NAME).a

# Add to library list
LIBS += $(LWIP_LIBRARY_FILE)

# Add include paths to the build.  This file is used to add lwIP as a library,
# so we just need to add relevant include paths...
include ${LWIP_DIR}/lwip_files.mk
IPATH += ${LWIP_INCLUDE_DIR}
# ... ^ here

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS
export MXC_OPTIMIZE_CFLAGS

# Add rule to build the Driver Library
${LWIP_BUILD_DIR}/${LWIP_LIBRARY_NAME}.a: ${LWIP_C_FILES} ${LWIP_H_FILES}
	$(MAKE) -f ${LWIP_DIR}/Makefile  lib BUILD_DIR=${LWIP_BUILD_DIR} PROJECT=${LWIP_LIBRARY_NAME}

clean.lwip:
	@rm -rf ${LWIP_BUILD_DIR}/*