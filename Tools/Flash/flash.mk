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

# Makefile targets to flash with JLinkExe or OpenOCD

# ADAPTER_SN is debugger serial number
# This variable is optional, useful when multiple debuggers are connected
# It can be defined as an environment variable or in the Makefiles
#
# Example for JLINK: 229002239
# Example for CMSIS-DAP: 04090000068682d300000000000000000000000097969906
#
# ADAPTER_SN=229002239
# ADAPTER_SN=04090000068682d300000000000000000000000097969906

# Location of the OpenOCD install directory, tcl or scripts folder
OPENOCD_SCRIPTS ?= ${MAXIM_PATH}/Tools/OpenOCD/scripts

# Update with the connected adapter
OPENOCD_ADAPTER ?= cmsis-dap.cfg
# OPENOCD_ADAPTER ?= jlink.cfg

HEX_FILE        := ${BUILD_DIR}/${PROJECT}.hex
HEX_FILE_PATH   := ${HEX_FILE}
ifeq ($(OS),Windows_NT)
JLINKEXE        ?= JLink.exe
ECHO            ?= echo -e
OPENOCDEXE      ?= openocd.exe

# Determine if we can use cygpath to convert the path name
CYGPATH_AVAILABLE := 0
ifneq ($(findstring MSYS, $(UNAME)), )
CYGPATH_AVAILABLE := 1
endif

ifneq ($(findstring MINGW, $(UNAME)), )
CYGPATH_AVAILABLE := 1
endif

ifneq ($(findstring CYGWIN, $(UNAME)), )
CYGPATH_AVAILABLE := 1
endif

# Use cygpath to convert the path name
ifeq ($(CYGPATH_AVAILABLE),1)
HEX_FILE_PATH   := $(shell cygpath -wa $(HEX_FILE))
HEX_FILE_PATH   := $(subst \,/, $(HEX_FILE_PATH))
else
# $(warning Warning: cygpath unavailable to convert path name to Windows format)
endif

else
# Not Windows_NT
JLINKEXE        ?= JLinkExe
ECHO            ?= echo
OPENOCDEXE      ?= openocd
endif

JLINKEXE     += -if SWD -device ${TARGET_UC} -speed 10000
COMMAND_FILE := flash.jlinkexe

PHONY: flash.jlink
flash.jlink: mkbuildir ${HEX_FILE}
	@$(ECHO) "$(if $(ADAPTER_SN), "SelectEmuBySN $(ADAPTER_SN)",)\nr\nhalt\nLoadFile \
		${HEX_FILE_PATH},0\nr\ng\nexit\n" > ${COMMAND_FILE}
	@$(JLINKEXE) -NoGui 1 -CommandFile ${COMMAND_FILE}

PHONY: flash.openocd
flash.openocd: mkbuildir ${HEX_FILE}
	@$(OPENOCDEXE) -s ${OPENOCD_SCRIPTS} -f interface/${OPENOCD_ADAPTER} -f target/${TARGET_LC}.cfg \
		$(if $(ADAPTER_SN), "-c adapter serial $(ADAPTER_SN)",) \
		-c "program ${HEX_FILE_PATH} verify reset exit"
