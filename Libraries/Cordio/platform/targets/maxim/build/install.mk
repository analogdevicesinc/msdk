###################################################################################################
#
# Install make targets
#
# Copyright (c) 2019 Packetcraft, Inc.  All rights reserved.
# Packetcraft, Inc. confidential and proprietary.
# 
# IMPORTANT.  Your use of this file is governed by a Software License Agreement
# ("Agreement") that must be accepted in order to download or otherwise receive a
# copy of this file.  You may not use or copy this file for any purpose other than
# as described in the Agreement.  If you do not agree to all of the terms of the
# Agreement do not use this file and delete all copies in your possession or control;
# if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
# to any use, copying or further distribution of this software.
#
###################################################################################################

#--------------------------------------------------------------------------------------------------
#     Project
#--------------------------------------------------------------------------------------------------

# Toolchain
GDB             := $(CROSS_COMPILE)gdb
GDBSERVER       := JLinkGDBServerCL
ifeq ($(OS),Windows_NT)
JLINKEXE        := JLink.exe
ECHO            := echo -e
else
JLINKEXE        := JLinkExe
ECHO            := echo
endif

GDBSERVER       += -if SWD -device $(CHIP) -speed 10000kHz
JLINKEXE        += -if SWD -device $(CHIP) -speed 10000kHz

#--------------------------------------------------------------------------------------------------
#     Targets
#--------------------------------------------------------------------------------------------------

install: install.flash

install.flash:
	@$(ECHO) "$(if $(SN), "SelectEmuBySN $(SN)",)\nhalt\nloadfile $(BIN:.elf=.hex)\nr\ng\nexit\n" > loadbin.jlinkexe
	@$(JLINKEXE) -CommandFile loadbin.jlinkexe

install.flashall:
	@rm -f loadbin.jlinkexe
	@for sn in $(shell $(MAKE) PLATFORM=$(PLATFORM) device.sn); do \
		$(ECHO) "SelectEmuBySN $$sn\nhalt\nloadfile $(BIN:.elf=.hex)\nr\ng\n" >> loadbin.jlinkexe; \
		done;
	@test -s loadbin.jlinkexe || { $(ECHO) "error: no devices found" >&2; exit 1; }
	@$(ECHO) "exit\n" >> loadbin.jlinkexe
	@$(JLINKEXE) -CommandFile loadbin.jlinkexe

install.server:
	@$(GDBSERVER) $(if $(SN),-select usb=$(SN),)

device.sn:
	@$(ECHO) "ShowEmuList\nexit\n" > showlist.jlinkexe
	@$(JLINKEXE) -CommandFile showlist.jlinkexe | sed -n 's/^.*Serial number: \([0-9]*\).*/\1/p' | grep [0-9]

device.reset:
	@$(ECHO) $(if $(SN), "SelectEmuBySN $(SN)",)  > reset.jlinkexe
	@$(ECHO) "h"                                 >> reset.jlinkexe
	@$(ECHO) "r"                                 >> reset.jlinkexe
	@$(ECHO) "g"                                 >> reset.jlinkexe
	@$(ECHO) "exit"                              >> reset.jlinkexe
	@$(JLINKEXE) -CommandFile reset.jlinkexe

device.resetall:
	@rm -f reset.jlinkexe
	@for sn in $(shell $(MAKE) PLATFORM=$(PLATFORM) device.sn); do \
		$(ECHO) "SelectEmuBySN $$sn" >> reset.jlinkexe; \
		$(ECHO) "h"                  >> reset.jlinkexe; \
		$(ECHO) "r"                  >> reset.jlinkexe; \
		$(ECHO) "g"                  >> reset.jlinkexe; \
		done
	@test -s reset.jlinkexe || { $(ECHO) "error: no devices found" >&2; exit 1; }
	@$(ECHO)    "exit"               >> reset.jlinkexe
	@$(JLINKEXE) -CommandFile reset.jlinkexe

.PHONY: install install.flash install.flashall install.server \
        device.sn device.reset device.resetall
