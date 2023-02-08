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

# Get the prefix for the tools to use.
ifeq "$(TOOL_DIR)" ""
PREFIX=riscv-none-embed
else
PREFIX=$(TOOL_DIR)/riscv-none-embed
endif

# The command for calling the compiler.
CC=${PREFIX}-gcc
CPP=${PREFIX}-g++

# Discover if we are using GCC > 4.8.0
GCCVERSIONGTEQ4 := $(shell expr `gcc -dumpversion | cut -f1 -d.` \> 4)
ifeq "$(GCCVERSIONGTEQ4)" "0"
GCCVERSIONGTEQ4 := $(shell expr `gcc -dumpversion | cut -f1 -d.` \>= 4)
	
ifeq "$(GCCVERSIONGTEQ4)" "1"
GCCVERSIONGTEQ4 := $(shell expr `gcc -dumpversion | cut -f2 -d.` \>= 8)
endif

endif

# The flags passed to the assembler.
#AFLAGS+=-v
ifneq "$(HEAP_SIZE)" ""
AFLAGS+=-D__HEAP_SIZE=$(HEAP_SIZE)
endif
ifneq "$(STACK_SIZE)" ""
AFLAGS+=-D__STACK_SIZE=$(STACK_SIZE)
endif
AFLAGS+=$(PROJ_AFLAGS)

ifeq "$(MXC_OPTIMIZE_CFLAGS)" ""
# Default is optimize for size
MXC_OPTIMIZE_CFLAGS = -Os
endif

# The flags passed to the compiler.
# fno-isolate-erroneous-paths-dereference disables the check for pointers with the value of 0
#  add this below when riscv-none-embed version is past 4.8 -fno-isolate-erroneous-paths-dereference  \

ifeq "$(RISCV_NOT_COMPRESSED)" ""
CFLAGS = -march=rv32imc                                                     
else
CFLAGS = -march=rv32im                                                     
endif
CFLAGS+= \
	     -mabi=ilp32                                                             \
	     -ffunction-sections                                                     \
	     -fdata-sections                                                         \
	     -MD                                                                     \
	     -Wall                                                                   \
	     -Wno-format                                                             \
	     -c

# On GCC version > 4.8.0 use the -fno-isolate-erroneous-paths-dereference flag
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
endif
endif

CFLAGS+=$(PROJ_CFLAGS)
AS=${PREFIX}-as
# The command for calling the library archiver.
AR=${PREFIX}-ar

# The command for calling the linker.
LD=${PREFIX}-gcc

# The flags passed to the linker.
ifeq "$(RISCV_NOT_COMPRESSED)" ""
LDFLAGS=-march=rv32imafdc 
else
LDFLAGS=-march=rv32imafd 
endif
LDFLAGS+=-Xlinker --gc-sections       \
      -nostartfiles \
      -Xlinker -Map -Xlinker ${BUILD_DIR}/$(PROJECT).map
LDFLAGS+=$(PROJ_LDFLAGS)

# Include math library
STD_LIBS=-lc_nano -lm

# Determine if any C++ files are in the project sources, and add libraries as appropriate
ifneq "$(findstring cpp, ${SRCS})" ""
STD_LIBS+=-lsupc++ -lstdc++
endif

# Finally, resolve any newlib system calls with libnosys
STD_LIBS+=-lnosys

PROJ_LIBS:=$(addprefix -l, $(PROJ_LIBS))

# The command for extracting images from the linked executables.
OBJCOPY=${PREFIX}-objcopy
OBJDUMP=${PREFIX}-objdump

ifeq "$(CYGWIN)" "True"
fixpath=$(shell echo $(1) | sed -r 's/\/cygdrive\/([A-Na-n])/\U\1:/g' )
else
fixpath=$(1)
endif

# Add the include file paths to AFLAGS and CFLAGS.
AFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
CFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
LDFLAGS+=${patsubst %,-L%,$(call fixpath,$(LIBPATH))}

################################################################################
# The rule for building the object file from each C source file.
${BUILD_DIR}/%.o: %.c
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
${BUILD_DIR}/%.o: %.cpp
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
${BUILD_DIR}/%.o: %.S
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
${BUILD_DIR}/%.a:
	@if [ '${VERBOSE}' = '' ];                                                   \
	 then                                                                        \
	     echo "  AR    ${@}";                                                    \
	 else                                                                        \
	     echo ${AR} -cr $(call fixpath,${@}) $(call fixpath,${^});               \
	 fi
	@${AR} -cr $(call fixpath,${@}) $(call fixpath,${^})

# The rule for linking the application.
${BUILD_DIR}/%.elf:
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
	@echo BUILD_DIR = ${BUILD_DIR}
	@echo
	@echo SRCS = ${SRCS}
	@echo
	@echo SRCS_NOPATH = ${SRCS_NOPATH}
	@echo
	@echo CC = ${CC}
	@echo
	@echo AS = ${AS}
	@echo
	@echo LD = ${LD}
	@echo
	@echo CFLAGS = ${CFLAGS}
	@echo
	@echo AFLAGS = ${AFLAGS}
	@echo
	@echo LFLAGS = ${LFLAGS}
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

