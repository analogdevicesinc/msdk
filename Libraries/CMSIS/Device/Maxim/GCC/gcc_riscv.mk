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

# The build directory
ifeq "$(BUILD_DIR)" ""
BUILD_DIR=$(CURDIR)/buildrv
endif

# Create output object file names
SRCS_NOPATH := $(foreach NAME,$(SRCS),$(basename $(notdir $(NAME))).c)
OBJS_NOPATH := $(SRCS_NOPATH:.c=.o)
OBJS        := $(OBJS_NOPATH:%.o=$(BUILD_DIR)/%.o)

################################################################################
# Goals

# The default goal, which causes the example to be built.
.DEFAULT_GOAL :=
.PHONY: all
all: mkbuildir
all: ${BUILD_DIR}/${PROJECT}.elf
all: project_defines

# Goal to build for release without debug
.PHONY: release
release: mkbuildir
release: ${BUILD_DIR}/${PROJECT}.elf
release: ${BUILD_DIR}/${PROJECT}.srec
release: ${BUILD_DIR}/${PROJECT}.hex
release: ${BUILD_DIR}/${PROJECT}.bin
release: ${BUILD_DIR}/${PROJECT}.dasm

# The goal to build as a library
.PHONY: lib
lib: mkbuildir
lib: ${BUILD_DIR}/${PROJECT}.a

# The goal to create the target directory.
.PHONY: mkbuildir
mkbuildir:
	@mkdir -p ${BUILD_DIR}

# The goal to clean out all the build products.
.PHONY: clean
clean:
	@rm -rf ${BUILD_DIR} ${wildcard *~}

${BUILD_DIR}/${PROJECT}.elf: ${LIBS} ${OBJS} ${LINKERFILE}
${BUILD_DIR}/${PROJECT}.a: ${OBJS}

# Create a goal to exercise the library build dependencies
.PHONY: FORCE
FORCE:

# Include the automatically generated dependency files.
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${BUILD_DIR}/*.d} __dummy__
endif

################################################################################
# Get the operating system name.  If this is Cygwin, the .d files will be
# munged to convert c: into /cygdrive/c so that "make" will be happy with the
# auto-generated dependencies. Also if this is Cygwin, file paths for ARM GCC
# will be converted from /cygdrive/c to C:.
################################################################################
ifneq ($(findstring CYGWIN, ${shell uname -s}), )
CYGWIN=True
endif

# Set the toolchain prefix.  Top-level makefiles can specify RISCV_PREFIX or
# PREFIX directly.  RISCV_PREFIX is given to improve dual-core projects
ifeq "$(RISCV_PREFIX)" ""
PREFIX ?= riscv-none-elf
else
PREFIX ?= $(RISCV_PREFIX)
endif

# Set absolute path to tools if TOOL_DIR is specified
ifneq "$(TOOL_DIR)" ""
PREFIX=$(TOOL_DIR)/$(PREFIX)
endif

# The command for calling the assembler
AS=${PREFIX}-as
# The command for calling the library archiver.
AR=${PREFIX}-ar
# The command for calling the linker.
LD=${PREFIX}-gcc
# The command for calling the compiler.
CC=${PREFIX}-gcc
CPP=${PREFIX}-g++
# The command for extracting images from the linked executables.
OBJCOPY=${PREFIX}-objcopy
OBJDUMP=${PREFIX}-objdump

# Discover if we are using GCC > 4.8.0
GCCVERSIONGTEQ4 := $(shell expr `$(CC) -dumpversion | cut -f1 -d.` \> 4)
ifeq "$(GCCVERSIONGTEQ4)" "0"
GCCVERSIONGTEQ4 := $(shell expr `$(CC) -dumpversion | cut -f1 -d.` \>= 4)
	
ifeq "$(GCCVERSIONGTEQ4)" "1"
GCCVERSIONGTEQ4 := $(shell expr `$(CC) -dumpversion | cut -f2 -d.` \>= 8)
endif

endif

# Set -march, which defines the instruction set the compiler is allowed to use
# -march=[BASE][EXTENSION][EXTRAS]
# Options for the [BASE] of the -march string are:
# - RV32I: A load-store ISA with 32 (32-bit) general purpose integer registers
# - RV32E: RV32 "embedded" with 16 (32-bit) integer registers
# - RV64I: 64-bit version of RV32I with 32 (64-bit) general purpose integer registers
# Options for EXTENSIONS can be one or all (combined in order):
# - M: Integer multiplication and division
# - A: Atomic instructions
# - F: Single precision floating-point
# - D: Double precision floating point
# - C: Compressed instructions
# EXTRAS are additional options available per the compiler.  I'm not sure if there is a
# comprehensive list of these anywhere.  Try "gcc -dumpspecs" *******************************************************************************
ifeq "$(PREFIX)" "riscv-none-elf"
# With the upgrade to riscv-none-elf came a new ISA spec
# See https://groups.google.com/a/groups.riscv.org/g/sw-dev/c/aE1ZeHHCYf4
# BASE = RV32I
# EXTENSION = M
# EXTRAS = _zicsr_zifencei as recommended above
MARCH ?= rv32im_zicsr_zifencei
endif

# Default option (riscv-none-embed)
ifeq "$(RISCV_NOT_COMPRESSED)" ""
# The RISCV_NOT_COMPRESSED option is a legacy option that I'm not even sure anyone uses...
# but it has existed since the beginning to make the default builds use the compressed
# instruction set.
MARCH ?= rv32imc
else
MARCH ?= rv32im
endif
# *******************************************************************************
# Set -mabi, which defines the instruction set the linker is allowed to link
# against, calling convention, and layout of data in memory
MABI ?= ilp32

# *******************************************************************************
# Set default optimization flags
ifeq "$(MXC_OPTIMIZE_CFLAGS)" ""
# Default is optimize for size
MXC_OPTIMIZE_CFLAGS = -Os
endif

# The flags passed to the assembler.
AFLAGS += -march=$(MARCH)
ifneq "$(HEAP_SIZE)" ""
AFLAGS+=-D__HEAP_SIZE=$(HEAP_SIZE)
endif
ifneq "$(STACK_SIZE)" ""
AFLAGS+=-D__STACK_SIZE=$(STACK_SIZE)
endif
AFLAGS+=$(PROJ_AFLAGS)

# The flags passed to the compiler.
CFLAGS+= \
         -march=$(MARCH)		 \
	     -mabi=$(MABI)           \
	     -ffunction-sections     \
	     -fdata-sections         \
	     -MD                     \
	     -Wall                   \
	     -Wno-format             \
	     $(MXC_OPTIMIZE_CFLAGS)  \
	     -c

# fno-isolate-erroneous-paths-dereference disables the check for pointers with the value of 0
#  add this below when riscv-none-embed version is past 4.8 -fno-isolate-erroneous-paths-dereference
ifeq "$(GCCVERSIONGTEQ4)" "1"
CFLAGS += -fno-isolate-erroneous-paths-dereference
endif

ifneq "$(TARGET)" ""
CFLAGS+=-DTARGET=$(TARGET)
endif

ifneq "$(TARGET_REV)" ""
CFLAGS+=-DTARGET_REV=$(TARGET_REV)
endif

# Exclude debug for 'release' builds
ifneq (${MAKECMDGOALS},release)
ifneq (${DEBUG},0)
CFLAGS+=-g3 -ggdb -DDEBUG
CXX_FLAGS+=-g3 -ggdb -DDEBUG
endif
endif

CFLAGS+=$(PROJ_CFLAGS)

# NOTE(JC): I'm leaving this commented because it's weird.  We used
# to pass the linker **all** of the available extensions and no -mabi
# option...  I don't think that was correct.  I have updated LDFLAGS
# to match what we give the compiler but I will leave this relic for
# future civilizations
# ----
# ifeq "$(RISCV_NOT_COMPRESSED)" ""
# LDFLAGS=-march=rv32imafdc
# else
# LDFLAGS=-march=rv32imafd 
# endif
# ----

# The flags passed to the linker
LDFLAGS+=-Xlinker --gc-sections       \
      -nostartfiles 	\
	  -march=$(MARCH) 	\
	  -mabi=$(MABI)		\
      -Xlinker -Map -Xlinker ${BUILD_DIR}/$(PROJECT).map
LDFLAGS+=$(PROJ_LDFLAGS)

# Include math library
STD_LIBS=-lc_nano -lm

# Determine if any C++ files are in the project sources, and add libraries as appropriate
ifneq "$(findstring .cpp, ${SRCS})" ""
STD_LIBS+=-lsupc++ -lstdc++
endif

# Finally, resolve any newlib system calls with libnosys
STD_LIBS+=-lnosys

PROJ_LIBS:=$(addprefix -l, $(PROJ_LIBS))

ifeq "$(CYGWIN)" "True"
fixpath=$(shell echo $(1) | sed -r 's/\/cygdrive\/([A-Na-n])/\U\1:/g' )
else
fixpath=$(1)
endif

# Add the include file paths to AFLAGS and CFLAGS.
AFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
CFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
LDFLAGS+=${patsubst %,-L%,$(call fixpath,$(LIBPATH))}

# Add an option for stripping unneeded symbols from archive files
STRIP_LIBRARIES ?= 0
# The command for stripping objects.
STRIP = $(PREFIX)-strip

################################################################################
# The rule for building the object file from each C source file.
${BUILD_DIR}/%.o: %.c $(PROJECTMK)
	@if [ '${ECLIPSE}' != '' ]; 																			\
	then 																									\
		echo ${CC} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<}) | sed 's/-I\/\(.\)\//-I\1:\//g' ; \
	elif [ '${VERBOSE}' != '' ];                                               								\
	then 																									\
	    echo ${CC} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<});     								\
	else                                                                       								\
	    echo "  CC    ${<}";                                                   								\
	fi

	@${CC} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<})
ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

# The rule to build an object file from a C++ source file
${BUILD_DIR}/%.o: %.cpp $(PROJECTMK)
	@if [ '${ECLIPSE}' != '' ]; 																			\
	then 																									\
		echo ${CPP} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<}) | sed 's/-I\/\(.\)\//-I\1:\//g' ; \
	elif [ '${VERBOSE}' != '' ];                                               								\
	then 																									\
	    echo ${CPP} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<});     								\
	else                                                                       								\
	    echo "  CC    ${<}";                                                   								\
	fi
	
	@${CPP} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<})
ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

# The rule for building the object file from each assembly source file.
${BUILD_DIR}/%.o: %.S $(PROJECTMK)
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "  AS    ${<}";                                                    \
	 else                                                                        \
	     echo ${CC} ${AFLAGS} -o $(call fixpath,${@}) -c $(call fixpath,${<});   \
	 fi
	@${CC} ${AFLAGS} -o $(call fixpath,${@}) -c $(call fixpath,${<})
ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

# The rule for creating an object library.
${BUILD_DIR}/%.a: $(PROJECTMK)
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "  AR    ${@}";                                                    \
	 else                                                                        \
	     echo ${AR} -cr $(call fixpath,${@}) $(call fixpath,${^});               \
	 fi
	@${AR} -cr $(call fixpath,${@}) $(call fixpath,${^})
ifeq ($(STRIP_LIBRARIES),1)
	@if [ 'x${ECLIPSE}' != x ];                                                 \
	 then                                                                       \
	    echo ${STRIP} $(call fixpath,${@}) | sed 's/-I\/\(.\)\//-I\1:\//g' ;    \
	elif [ 'x${VERBOSE}' != x ];                                                \
	then                                                                        \
	    echo ${STRIP} --strip-unneeded $(call fixpath,${@});                    \
	elif [ 'x${QUIET}' != x ];                                                  \
	then                                                                        \
	    :;                                                                      \
	else                                                                        \
	    echo "  STRIP ${@}";                                                    \
	fi
	@${STRIP} --strip-unneeded $(call fixpath,${@})
endif

# The rule for linking the application.
${BUILD_DIR}/%.elf: $(PROJECTMK)
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "  LD    ${@} ${LNK_SCP}";                                         \
	 else                                                                        \
	     echo ${LD} -T $(call fixpath,${LINKERFILE})                             \
	          --entry ${ENTRY}                                                   \
	          $(call fixpath,${LDFLAGS})                                         \
	          -o $(call fixpath,${@})                                            \
	          $(call fixpath,$(filter %.o, ${^}))                                \
	          -Xlinker --start-group                                             \
	          $(call fixpath,$(filter %.a, ${^}))                                \
	          ${PROJ_LIBS}                                                       \
	          ${STD_LIBS}                                                        \
	          -Xlinker --end-group;                                              \
	 fi;                                                                         \
	${LD} -T $(call fixpath,${LINKERFILE})                                       \
	      --entry ${ENTRY}                                                       \
	      $(call fixpath,${LDFLAGS})                                             \
	      -o $(call fixpath,${@})                                                \
	      $(call fixpath,$(filter %.o, ${^}))                                    \
	      -Xlinker --start-group                                                 \
	      $(call fixpath,$(filter %.a, ${^}))                                    \
	      ${PROJ_LIBS}                                                           \
	      ${STD_LIBS}                                                            \
	      -Xlinker --end-group

# Create S-Record output file
%.srec: %.elf
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "Creating ${@}";                                                   \
	 else                                                                        \
	     echo ${OBJCOPY} -O srec $(call fixpath,${<}) $(call fixpath,${@});      \
	 fi
	@$(OBJCOPY) -O srec $< $(call fixpath,${@})

# Create Intex Hex output file
%.hex: %.elf
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "Creating ${@}";                                                   \
	 else                                                                        \
	     echo ${OBJCOPY} -O ihex $(call fixpath,${<}) $(call fixpath,${@});      \
	 fi
	@$(OBJCOPY) -O ihex $< $(call fixpath,${@})

# Create binary output file
%.bin: %.elf
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "Creating ${@}";                                                   \
	 else                                                                        \
	     echo ${OBJCOPY} -O binary $(call fixpath,${<}) $(call fixpath,${@});    \
	 fi
	@$(OBJCOPY) -O binary $< $(call fixpath,${@})

# Create disassembly file
%.dasm: %.elf
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "Creating ${@}";                                                   \
	 else                                                                        \
	     echo $(OBJDUMP) -S $(call fixpath,${<}) $(call fixpath,${@});        \
	 fi
	@$(OBJDUMP) -S $< > $(call fixpath,${@})

################################################################################
.PHONY: debug
debug:
	@echo CYGWIN = ${CYGWIN}
	@echo
	@echo CC = ${CC}
	@echo
	@echo AS = ${AS}
	@echo
	@echo LD = ${LD}
	@echo
	@echo TARGET = ${TARGET}
	@echo
	@echo BOARD = ${BOARD}
	@echo
	@echo BUILD_DIR = ${BUILD_DIR}
	@echo
	@echo SRCS = ${SRCS}
	@echo
	@echo SRCS_NOPATH = ${SRCS_NOPATH}
	@echo
	@echo OBJS_NOPATH = ${OBJS_NOPATH}
	@echo
	@echo OBJS = ${OBJS}
	@echo
	@echo LIBS = ${LIBS}
	@echo
	@echo VPATH = ${VPATH}
	@echo
	@echo IPATH = ${IPATH}
	@echo
	@echo CFLAGS = ${CFLAGS}
	@echo
	@echo AFLAGS = ${AFLAGS}
	@echo
	@echo LDFLAGS = ${LDFLAGS}

################################################################################
# Add a rule for generating a header file containing compiler definitions
# that come from the build system and compiler itself.  This generates a
# "project_defines.h" header file inside the build directory that can be
# force included by VS Code to improve the intellisense engine.
.PHONY: project_defines
project_defines: $(BUILD_DIR)/project_defines.h
$(BUILD_DIR)/project_defines.h: mkbuildir
	$(file > $(BUILD_DIR)/empty.c,)
	$(file > $(BUILD_DIR)/project_defines.h,// This is a generated file that's used to detect definitions that have been set by the compiler and build system.)
	@$(CC) -E -P -dD $(BUILD_DIR)/empty.c $(CFLAGS) >> $(BUILD_DIR)/project_defines.h
	@rm $(BUILD_DIR)/empty.c
	@rm empty.d