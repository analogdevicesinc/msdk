################################################################################
# Copyright (C) 2017-2018 Maxim Integrated Products, Inc., All rights Reserved.
# 
# This software is protected by copyright laws of the United States and
# of foreign countries. This material may also be protected by patent laws
# and technology transfer regulations of the United States and of foreign
# countries. This software is furnished under a license agreement and/or a
# nondisclosure agreement and may only be used or reproduced in accordance
# with the terms of those agreements. Dissemination of this information to
# any party or parties not specified in the license agreement and/or
# nondisclosure agreement is expressly prohibited.
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
################################################################################
#
# Author : Benjamin VINOT <benjamin.vinot@maximintegrated.com>
#


PROGRAM:= build_scp_session


CFLAGS		+= -Wall -Werror -DWORD32 -Wextra -Wshadow -Wno-format-overflow -Wno-strict-aliasing
LDFLAGS		+= -static
STATICLIB   += -lucl


ifdef HSM
 STATICLIB += -lHSM
 CFLAGS += -D_MAXIM_HSM
endif

ifdef DEBUG
CFLAGS		+=	-Og -g
endif

ifndef ARCH
ARCH := 64
endif

ifeq ($(ARCH),32)
  CFLAGS += -m32
  LDFLAGS += -m32
endif

ifeq ($(OS),win)
  EXEC_EXT := .exe
  TARGET := windows
  CFLAGS += -D__WIN
  STATICLIB  += -lregex
else ifeq ($(OS),cygwin)
  EXEC_EXT := .exe
  TARGET := cygwin
else
  EXEC_EXT := 
  TARGET := linux
endif


ifndef CROSS_COMPIL
 CROSS_COMPIL = 
endif

DISTDIR     := bin/$(TARGET)/$(ARCH)
BUILDDIR	:= obj
LIBDIR      := lib/$(TARGET)_$(ARCH)
SRCDIR		:= src
TESTDIR		:= test
INCDIR      := include

SRCEXT		:= c
DEPEXT      := d
OBJEXT      := o
DOCSRCEXT	:= md
DOCOUTEXT	:= pdf

Q			:= 

INC         := -I$(INCDIR)
INCDEP      := -I$(INCDIR)

TEST_LFLAG	:= -lcunit

CC			:=	$(CROSS_COMPIL)gcc

EXEC		:=$(PROGRAM)$(EXEC_EXT)

SOURCES     := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS     := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.$(OBJEXT)))

TEST_EXEC	:= test_$(PROGRAM)$(EXEC_EXT)
TEST_SRC	:= $(shell find $(TESTDIR) -type f -name *.$(SRCEXT))

TEST_OBJECTS     := $(patsubst $(TESTDIR)/%,$(BUILDDIR)/%,$(TEST_SRC:.$(SRCEXT)=.$(OBJEXT)))
TEST_OBJECTS     += $(subst $(BUILDDIR)/$(PROGRAM).$(OBJEXT),,$(OBJECTS))

DOCSSRC	    := $(shell find \. -type f -name '*.$(DOCSRCEXT)')
DOCS	    := $(DOCSSRC:.$(DOCSRCEXT)=.$(DOCOUTEXT))

#
# Rules
#

.PHONY: all clean distclean install resources
all:	directories $(EXEC)

doc: $(DOCS)

clean:
	@echo '[RM] $(BUILDDIR)'
	$(Q)rm -rf -- $(BUILDDIR)

distclean: clean
	@echo '[RM] $(DISTDIR)'
	$(Q)rm -rf -- $(DISTDIR)

uninstall: clean

#install: ;


directories: 
	$(Q)mkdir -p $(DISTDIR)
	$(Q)mkdir -p $(BUILDDIR)

test: directories $(TEST_EXEC)

staticcheck :
	$(Q)clang-tidy  $(SRCDIR)/* -checks=clang-analyzer-\*,cert\*,readability\*,google\* -- $(INC) -DWORD32

#
# Dependencies
# 
$(TEST_EXEC): $(TEST_OBJECTS)
	@echo '[LD] $@'
	$(Q)$(CC) -L$(LIBDIR) -o $(DISTDIR)/$(TEST_EXEC) $^ $(STATICLIB) $(TEST_LFLAG)
	@echo '[BUILD] SUCCESS'

$(EXEC): $(OBJECTS)
	@echo '[LD] $@'
	$(Q)$(CC) $(LDFLAGS) -L$(LIBDIR) -o $(DISTDIR)/$(EXEC) $^ $(STATICLIB)
	@echo '[BUILD] SUCCESS'
    

	
$(BUILDDIR)/test_%.$(OBJEXT): $(TESTDIR)/test_%.$(SRCEXT)
	@echo '[CC] $<'
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	$(Q)$(CC) $(CFLAGS) $(INCDEP) -MM $(TESTDIR)/test_$*.$(SRCEXT) > $(BUILDDIR)/test_$*.$(DEPEXT)
	@cp -f $(BUILDDIR)/test_$*.$(DEPEXT) $(BUILDDIR)/test_$*.$(DEPEXT).tmp
	@sed -e 's|.*:|$(BUILDDIR)/test_$*.$(OBJEXT):|' < $(BUILDDIR)/test_$*.$(DEPEXT).tmp > $(BUILDDIR)/test_$*.$(DEPEXT)
	@sed -e 's/.*://' -e 's/\\$$//' < $(BUILDDIR)/test_$*.$(DEPEXT).tmp | fmt -1 | sed -e 's/^ *//' -e 's/$$/:/' >> $(BUILDDIR)/test_$*.$(DEPEXT)
	@rm -f $(BUILDDIR)/test_$*.$(DEPEXT).tmp

$(BUILDDIR)/%.$(OBJEXT): $(SRCDIR)/%.$(SRCEXT)
	@echo '[CC] $<'
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	$(Q)$(CC) $(CFLAGS) $(INCDEP) -MM $(SRCDIR)/$*.$(SRCEXT) > $(BUILDDIR)/$*.$(DEPEXT)
	@cp -f $(BUILDDIR)/$*.$(DEPEXT) $(BUILDDIR)/$*.$(DEPEXT).tmp
	@sed -e 's|.*:|$(BUILDDIR)/$*.$(OBJEXT):|' < $(BUILDDIR)/$*.$(DEPEXT).tmp > $(BUILDDIR)/$*.$(DEPEXT)
	@sed -e 's/.*://' -e 's/\\$$//' < $(BUILDDIR)/$*.$(DEPEXT).tmp | fmt -1 | sed -e 's/^ *//' -e 's/$$/:/' >> $(BUILDDIR)/$*.$(DEPEXT)
	@rm -f $(BUILDDIR)/$*.$(DEPEXT).tmp


%.$(DOCOUTEXT): %.$(DOCSRCEXT)
	@echo '[DOC] $<'
	$(Q)pandoc $< --pdf-engine=xelatex -o $@ --template eisvogel --number-sections --listings --highlight-style tango

# End Of Makefile

