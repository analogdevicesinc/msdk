###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2025 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #	 http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################

################################################################################
# Detect whether we're working from the Github repo or not.
# If so, attempt to update the version number files every time we build.

ifeq "$(PYTHON_CMD)" ""
# Try python
ifneq "$(wildcard $(MAXIM_PATH)/.github)" ""
PYTHON_VERSION := $(shell python --version)
ifneq ($(.SHELLSTATUS),0)
PYTHON_CMD := none
else
PYTHON_CMD := python
endif

# Try python3
ifeq "$(PYTHON_CMD)" "none"
PYTHON_VERSION := $(shell python3 --version)
ifneq ($(.SHELLSTATUS),0)
PYTHON_CMD := none
else
PYTHON_CMD := python
endif
endif

# Export PYTHON_CMD so we don't check for it again unnecessarily
export PYTHON_CMD
endif

# Make sure script exists before running. This won't run when working in the official MSDK release (non-GitHub)
ifneq ("$(wildcard $(MAXIM_PATH)/.github)", "")
# Run script if exists.
ifneq "$(PYTHON_CMD)" "none"
UPDATE_VERSION_OUTPUT := $(shell python $(MAXIM_PATH)/.github/workflows/scripts/update_version.py)
else
$(warning No Python installation detected on your system!  Will not automatically update version info.)
endif 
endif
endif

ifneq "$(wildcard $(MAXIM_PATH)/Libraries/CMSIS/Device/Maxim/GCC/mxc_version.mk)" ""
include $(MAXIM_PATH)/Libraries/CMSIS/Device/Maxim/GCC/mxc_version.mk
endif
################################################################################

SUPPRESS_HELP ?= 0
ifeq "$(SUPPRESS_HELP)" "0"
ifneq "$(HELP_COMPLETE)" "1"

$(info ****************************************************************************)
$(info * Analog Devices MSDK)
ifneq "$(MSDK_VERSION_STRING)" ""
$(info * $(MSDK_VERSION_STRING))
endif
$(info * - User Guide: https://analogdevicesinc.github.io/msdk/USERGUIDE/)
$(info * - Get Support: https://www.analog.com/support/technical-support.html)
$(info * - Report Issues: https://github.com/analogdevicesinc/msdk/issues)
$(info * - Contributing: https://analogdevicesinc.github.io/msdk/CONTRIBUTING/)
$(info ****************************************************************************)
# export HELP_COMPLETE so that it's only printed once.
HELP_COMPLETE = 1
export HELP_COMPLETE
endif
endif

################################################################################

# The build directory
ifeq "$(BUILD_DIR)" ""
BUILD_DIR?=$(CURDIR)/build
endif

# Make sure VPATH has the location of any absolute paths given to SRCS
# This allows users to specify SRCS += /absolute/path/to/file.c to add a single file to the build
# without also having to add VPATH += /absolute/path/to
# This is necessary because we create our object file definitions with OBJS_NOPATH.
VPATH += $(sort $(abspath $(dir $(SRCS))))

# Create output object file names
SRCS_NOPATH := $(foreach NAME,$(SRCS),$(basename $(notdir $(NAME))).c)
BINS_NOPATH := $(foreach NAME,$(BINS),$(basename $(notdir $(NAME))).bin)
OBJS_NOPATH := $(SRCS_NOPATH:.c=.o)
OBJS_NOPATH += $(BINS_NOPATH:.bin=.o)
OBJS		:= $(OBJS_NOPATH:%.o=$(BUILD_DIR)/%.o)
OBJS		+= $(PROJ_OBJS)


################################################################################
# Goals
################################################################################
# Get the operating system name.  If this is Cygwin, the .d files will be
# munged to convert c: into /cygdrive/c so that "make" will be happy with the
# auto-generated dependencies. Also if this is Cygwin, file paths for ARM GCC
# will be converted from /cygdrive/c to C:.
################################################################################

# Detect target OS
# windows : native windows
# windows_msys : MSYS2 on windows
# windows_cygwin : Cygwin on windows (legacy config from old sdk)
# linux : Any linux distro
# macos : MacOS
ifeq "$(_OS)" ""

ifeq "$(OS)" "Windows_NT"
_OS = windows

UNAME_RESULT := $(shell uname -s 2>&1)
# MSYS2 may be present on Windows.  In this case,
# linux utilities should be used.  However, the OS environment
# variable will still be set to Windows_NT since we configure
# MSYS2 to inherit from Windows by default.
# Here we'll attempt to call uname (only present on MSYS2)
# while routing stderr -> stdout to avoid throwing an error
# if uname can't be found.
ifneq ($(findstring CYGWIN, $(UNAME_RESULT)), )
CYGWIN=True
_OS = windows_cygwin
endif

ifneq ($(findstring MSYS, $(UNAME_RESULT)), )
MSYS=True
_OS = windows_msys
endif
ifneq ($(findstring MINGW, $(UNAME_RESULT)), )
MSYS=True
_OS = windows_msys
endif

else # OS
$(info "")
UNAME_RESULT := $(shell uname -s)
ifeq "$(UNAME_RESULT)" "Linux"
_OS = linux
endif
ifeq "$(UNAME_RESULT)" "Darwin"
_OS = macos
endif

endif

endif

# The default goal, which causes the example to be built.
.DEFAULT_GOAL :=
.PHONY: all
all: $(BUILD_DIR)
all: ${BUILD_DIR}/${PROJECT}.elf
all: $(BUILD_DIR)/project_defines.h

# Goal to build for release without debug
.PHONY: release
release: $(BUILD_DIR)
release: ${BUILD_DIR}/${PROJECT}.elf
release: ${BUILD_DIR}/${PROJECT}.srec
release: ${BUILD_DIR}/${PROJECT}.hex
release: ${BUILD_DIR}/${PROJECT}.bin
release: ${BUILD_DIR}/${PROJECT}.dasm

# The goal to build as a library
.PHONY: lib
lib: $(BUILD_DIR)
lib: ${BUILD_DIR}/${PROJECT}.a

# The goal to create the target directory.
.PHONY: mkbuildir
mkbuildir: $(BUILD_DIR)
$(BUILD_DIR):
	@echo -  MKDIR $(BUILD_DIR)
ifeq "$(_OS)" "windows"
# Make run on native Windows will yield C:/-like paths, but the mkdir commands needs
# paths with backslashes.
	@if not exist ${subst /,\,${BUILD_DIR}} mkdir ${subst /,\,${BUILD_DIR}}
else
	@mkdir -p ${BUILD_DIR}
endif

# The goal to clean out all the build products.
.PHONY: clean
clean:
	@echo -  RMDIR $(BUILD_DIR)
ifneq "$(_OS)" "windows"
	@rm -rf ${BUILD_DIR} ${wildcard *~}
else
	@if exist ${subst /,\,${BUILD_DIR}} rmdir /s /q ${subst /,\,${BUILD_DIR}}
endif

${BUILD_DIR}/${PROJECT}.elf: ${LIBS} ${OBJS} ${LINKERFILE}
${BUILD_DIR}/${PROJECT}.a: ${OBJS}

# Create a goal to exercise the library build dependencies
.PHONY: FORCE
FORCE:

# Include the automatically generated dependency files.
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${BUILD_DIR}/*.d} __dummy__
endif

# Set the toolchain prefix.  Top-level makefiles can specify ARM_PREFIX or
# PREFIX directly.  ARM_PREFIX is given to improve dual-core projects
ifeq "$(ARM_PREFIX)" ""
PREFIX ?= arm-none-eabi
else
PREFIX ?= $(ARM_PREFIX)
endif

# Set absolute path to tools if TOOL_DIR is specified
ifneq "$(TOOL_DIR)" ""
PREFIX=$(TOOL_DIR)/$(PREFIX)
endif

# The command for calling the compiler.
CC=${PREFIX}-gcc
CXX=${PREFIX}-g++

# Discover if we are using GCC > 4.8.0
GCCVERSIONGTEQ4 := 1
# GCCVERSIONGTEQ4 := $(shell expr `$(CC) -dumpversion | cut -f1 -d.` \> 4)
# ifeq "$(GCCVERSIONGTEQ4)" "0"
# GCCVERSIONGTEQ4 := $(shell expr `$(CC) -dumpversion | cut -f1 -d.` \>= 4)
# ifeq "$(GCCVERSIONGTEQ4)" "1"
# GCCVERSIONGTEQ4 := $(shell expr `$(CC) -dumpversion | cut -f2 -d.` \>= 8)
# endif

# endif

ifeq "$(MXC_OPTIMIZE_CFLAGS)" ""
# Default is optimize for size
MXC_OPTIMIZE_CFLAGS = -Os
endif

# Select the target ARM processor.
# Permissible options can be found under the "-mtune" documentation in the GCC manual
# https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
# Our hardware currently supports
# - cortex-m4 (default)
# - cortex-m33
MCPU ?= cortex-m4

ifeq "$(MCPU)" "cortex-m33"

# Build a project with TrustZone.
# Acceptable values are
# - 0 (default) : The project builds as a Secure-only project (development similar to Cortex-M4 examples).
# - 1 			: The project builds with both Secure and Non-Secure context.
#
# When "0" is selected, the project will build as a Secure-only project with no TrustZone features enabled.
# Development is similar to other Cortex-M4 examples.
# 
# When "1" is selected, the build system will build a Secure and Non-Secure project separately, then link
# the two images into one combined image.
TRUSTZONE ?= 0

# Security mode for the target processor.
# Acceptable values are
# - SECURE
# - NONSECURE
#
# The core, by default, starts up in Secure mode.
#
# When "SECURE" is selected, the build system will link the program binary into the secure
# memory sections and map peripheral instances onto their corresponding secure
# address aliases.  "MSECURITY_MODE_SECURE" will be defined at compile time.
#
# When "NONSECURE" is selected, the program binary will be linked into the non-secure memory
# sections and peripherals will be mapped onto the non-secure address aliases.
# It should be noted that the M33 will boot into secure mode by default, which has access to
# both the secure and non-secure addresses and aliases.  "MSECURITY_MODE_NONSECURE" will be defined
# at compile time.
MSECURITY_MODE ?= SECURE

# Not accessible in {device}_files.mk without this.
export MSECURITY_MODE

ifeq "$(MSECURITY_MODE)" "SECURE"
# Align with Zephyr flags.
PROJ_AFLAGS += -DCONFIG_TRUSTED_EXECUTION_SECURE=1
PROJ_CFLAGS += -DCONFIG_TRUSTED_EXECUTION_SECURE=1

# Leaving these to support initial development.
PROJ_AFLAGS += -DIS_SECURE_ENVIRONMENT=1
PROJ_CFLAGS += -DIS_SECURE_ENVIRONMENT=1
else
# Align with Zephyr flags.
# Must be defined for the BLE build system.
PROJ_AFLAGS += -DCONFIG_TRUSTED_EXECUTION_SECURE=0
PROJ_CFLAGS += -DCONFIG_TRUSTED_EXECUTION_SECURE=0

# Leaving these to support initial development.
PROJ_AFLAGS += -DIS_SECURE_ENVIRONMENT=0
PROJ_CFLAGS += -DIS_SECURE_ENVIRONMENT=0
endif

ifeq ($(TRUSTZONE),1)

# Select whether the CMSE importlib object file is generated.
# - 0 (default) : Do no generate object file.
# - 1			: Generate object file.
#
# The Secure project needs to generate the object file with empty definitions of
# secure symbols at the right locations. The Non-Secure project needs to be linked
# with the generated object file.
GEN_CMSE_IMPLIB_OBJ ?= 0

ifeq "$(MSECURITY_MODE)" "SECURE"
# Tell the compiler we are building a secure project.  This is required to satisfy the requirements
# defined in "Armv8-M Security Extension: Requirements on Developments Tools"
# https://developer.arm.com/documentation/ecm0359818/latest
PROJ_CFLAGS += -mcmse

# Generate an object file with empty definitions of the secure image symbols at the correct locations.
# The object file needs to be linked with the non-secure image.
ifeq ($(GEN_CMSE_IMPLIB_OBJ),1)
# Linker arguments are placed into ln_args.txt and must be formatted with full windows path for
# MSYS environments because not all tools perform automatic path translations from UNIX to 
# native Windows paths.
LDFLAGS+=-Xlinker --cmse-implib	\
		 -Xlinker --out-implib=$(if $(filter "${_OS}","windows_msys"),						\
		 						 $(shell cygpath -m ${BUILD_DIR}/build_s/secure_implib.o),	\
								 ${BUILD_DIR}/build_s/secure_implib.o						\
								)
endif # GEN_CMSE_IMPLIB_OBJ
endif # MSECURITY_MODE
endif # TRUSTZONE
endif # MCPU

# Float ABI options:
# See https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html (-mfloat-abi)
# Specifies which floating-point ABI to use. Permissible values are: ‘soft’, ‘softfp’ and ‘hard’.

# Specifying ‘soft’ causes GCC to generate output containing library calls for floating-point
# operations. ‘softfp’ allows the generation of code using hardware floating-point
# instructions, but still uses the soft-float calling conventions. ‘hard’ allows generation of
# floating-point instructions and uses FPU-specific calling conventions.

# The default depends on the specific target configuration. Note that the hard-float and
# soft-float ABIs are not link-compatible; you must compile your entire program with the same
# ABI, and link with a compatible set of libraries.
ifeq "$(MFLOAT_ABI)" ""
MFLOAT_ABI = softfp
endif

# (deprecated) MFLOAT_FLAGS
ifneq "$(MFLOAT_FLAGS)" ""
$(warning MFLOAT_FLAGS has been deprecated!  Please use MFLOAT_ABI instead.)
MFLOAT_ABI = $(MFLOAT_FLAGS) # Copy over to new option for backwards compatability
endif

# Option for setting the FPU to use
MFPU ?= fpv4-sp-d16

# (deprecated) MFPU_FLAGS
# The old option implied multiple values could be set, so it was renamed to MFPU
ifneq "$(MFPU_FLAGS)" ""
$(warning MFPU_FLAGS has been deprecated!  Used MFPU instead.)
MFPU := $(MFPU_FLAGS) # Copy over to the new option for backwards compatability
endif

# The flags passed to the compiler.
# fno-isolate-erroneous-paths-dereference disables the check for pointers with the value of 0
# add this below when arm-none-eabi-gcc version is past 4.8 -fno-isolate-erroneous-paths-dereference

# Universal optimization flags added to all builds
DEFAULT_OPTIMIZE_FLAGS ?= -ffunction-sections -fdata-sections -fsingle-precision-constant
DEFAULT_WARNING_FLAGS ?= -Wall -Wno-format -Wdouble-promotion

# The flags passed to the assembler.
AFLAGS=-mthumb		 \
	   -mcpu=$(MCPU) \
	   -MD
ifneq "$(HEAP_SIZE)" ""
AFLAGS+=-D__HEAP_SIZE=$(HEAP_SIZE)
endif
ifneq "$(STACK_SIZE)" ""
AFLAGS+=-D__STACK_SIZE=$(STACK_SIZE)
endif
ifneq "$(SRAM_SIZE)" ""
AFLAGS+=-D__SRAM_SIZE=$(SRAM_SIZE)
endif
AFLAGS+=$(PROJ_AFLAGS)

CFLAGS=-mthumb																\
	   -mcpu=$(MCPU)														\
	   -mfloat-abi=$(MFLOAT_ABI)											\
	   -mfpu=$(MFPU)														\
	   -Wa,-mimplicit-it=thumb												\
	   $(MXC_OPTIMIZE_CFLAGS)   											\
	   $(DEFAULT_OPTIMIZE_FLAGS)   											\
	   $(DEFAULT_WARNING_FLAGS)   											\
	   -MD																	\
	   -c

# The flags passed to the C++ compiler.
CXXFLAGS := $(CFLAGS)
CXX_STANDARD?=17
CXXFLAGS += \
	-fno-rtti				\
	-fno-exceptions			\
	-std=c++$(CXX_STANDARD)

# On GCC version > 4.8.0 use the -fno-isolate-erroneous-paths-dereference flag
ifeq "$(GCCVERSIONGTEQ4)" "1"
CFLAGS += -fno-isolate-erroneous-paths-dereference
endif

ifneq "$(TARGET)" ""
CFLAGS+=-DTARGET=$(TARGET)
endif

ifneq "$(TARGET_UC)" ""
# Define a flag that the pre-processor can actually work with
# (i.e. #ifdef MAX78000 ...)
# TARGET_UC typically comes from the project core Makefile
CFLAGS += -D$(TARGET_UC)
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
CXXFLAGS+=$(CFLAGS)

C_WARNINGS_AS_ERRORS ?= implicit-function-declaration
CFLAGS += -Werror=$(C_WARNINGS_AS_ERRORS)
CFLAGS += -Wstrict-prototypes
# ^ Add strict-prototypes after CXX_FLAGS so it's only added for C builds

# The command for calling the library archiver.
AR=${PREFIX}-ar

# The command for calling the linker.
LD=${PREFIX}-gcc

# The flags passed to the linker.
# Including any file paths to linker flags must be handled separately for proper
# path handling in different build environments.
# The linker arguments are stored into ln_args.txt, and ln_args.txt is passed
# into the linker as an input. For isolated, MSYS build environments, the linker
# does not perform automatic path translations after reading an input file which
# can cause incorrect or missing file path errors. The conditional for the Map ensures
# the full, extended path is used with the linker.
LDFLAGS=-mthumb																			\
		-mcpu=$(MCPU)																	\
		-mfloat-abi=$(MFLOAT_ABI)														\
		-mfpu=$(MFPU)																	\
		-Xlinker --gc-sections															\
		-Xlinker --print-memory-usage													\
		-Xlinker -Map -Xlinker $(if $(filter "${_OS}","windows_msys"),					\
									$(shell cygpath -m ${BUILD_DIR}/$(PROJECT).map),	\
									${BUILD_DIR}/$(PROJECT).map)


# Add --no-warn-rwx-segments on GCC 12+
# This is not universally supported or enabled by default, so we need to check whether the linker supports it first
ARM_RWX_SEGMENTS_SUPPORTED ?=
ifeq "$(ARM_RWX_SEGMENTS_SUPPORTED)" "" # -------------------------------------
# Print the linker's help string and parse it for --no-warn-rwx-segments
# Note we invoke the linker through the compiler "-Xlinker" because ld may not
# be on the path, and that's how we invoke the linker for our implicit rules
LINKER_OPTIONS := $(shell $(CC) -Xlinker --help)
ifneq "$(findstring --no-warn-rwx-segments,$(LINKER_OPTIONS))" ""
ARM_RWX_SEGMENTS_SUPPORTED := 1
else
ARM_RWX_SEGMENTS_SUPPORTED := 0
endif

# export the variable for sub-make calls, so we don't need to interact with the shell again (it's slow).
export ARM_RWX_SEGMENTS_SUPPORTED
endif # ------------------------------------------------------------------

ifeq "$(ARM_RWX_SEGMENTS_SUPPORTED)" "1"
LDFLAGS += -Xlinker --no-warn-rwx-segments
endif

# Add project-specific linker flags
LDFLAGS+=$(PROJ_LDFLAGS)
space := $(subst ,, )
ifneq "$(strip $(PROJ_LDFLAGS))" ""
ifeq "$(_OS)" "windows_msys"
L_PATH := $(patsubst -L%,%,$(PROJ_LDFLAGS))
ifneq "$(or $(filter $(PROJ_LDFLAGS),$(L_PATH)),$(if $(findstring :,$(L_PATH)),,fail),$(findstring \,$(L_PATH)),$(findstring $(space),$(PROJ_LDFLAGS)))" ""
$(info [NOTE]: 'PROJ_LDFLAGS' additions detected. Any paths added to 'PROJ_LDFLAGS' MUST use the full \
	Windows drive path with forward slashes (e.g. C:/path/used/in/proj_ldflags))
endif
endif
endif

# Include math library
STD_LIBS=-lc -lm

# Determine if any C++ files are in the project sources, and add libraries as appropriate
ifneq "$(findstring .cpp, ${SRCS})" ""
STD_LIBS+=-lsupc++ -lstdc++
endif

# Finally, resolve any newlib system calls with libnosys
STD_LIBS+=-lnosys

PROJ_LIBS:=$(addprefix -l, $(PROJ_LIBS))

# The command for extracting images from the linked executables.
OBJCOPY=${PREFIX}-objcopy
OBJDUMP=${PREFIX}-objdump

# cygpath utility ships by default with Cygwin and MSYS2
# This change minimizes the different fixpath calls
# made from MSYS, CYGWIN, and other OS environments.
ifeq "$(or $(MSYS),$(CYGWIN))" "True"
# The GNU utilities require Windows-style paths (e.g., C:/...) when arguments are
# passed via an input text file (e.g., ln_args.txt). However, with this implementation,
# the tools do not perform automatic path translation, so the paths must be
# preformatted before its written to the text files.
# 
# cygpath is the most reliable utility to ensure the paths stored into the
# input txt files are formatted properly.
fixpath=$(shell cygpath -m $(1))
else
fixpath=$(1)
endif

# Add the include file paths to AFLAGS and CFLAGS.
AFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
CFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
CXXFLAGS+=${patsubst %,-I%,$(call fixpath,$(IPATH))}
LDFLAGS+=${patsubst %,-L%,$(call fixpath,$(LIBPATH))}

# Add an option for stripping unneeded symbols from archive files
STRIP_LIBRARIES ?= 0
# The command for stripping objects.
STRIP = $(PREFIX)-strip

################################################################################
# The rule for building the object file from each C source file.
${BUILD_DIR}/%.o: %.c $(PROJECTMK) | $(BUILD_DIR)
ifneq "${ECLIPSE}" ""
	@echo ${CC} ${CFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<}) | sed 's/-I\/\(.\)\//-I\1:\//g'
else
ifneq "${VERBOSE}" ""
	@echo ${CC} ${CFLAGS} -o ${@} ${<}
else
	@echo -  CC	${<}
endif
endif

	@${CC} ${CFLAGS} -o ${@} ${<}

ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

# The rule to build an object file from a C++ source file
${BUILD_DIR}/%.o: %.cpp $(PROJECTMK) | $(BUILD_DIR)
ifneq "${ECLIPSE}" ""
	@echo ${CXX} ${CXXFLAGS} -o $(call fixpath,${@}) $(call fixpath,${<}) | sed 's/-I\/\(.\)\//-I\1:\//g'
else
ifneq "${VERBOSE}" ""
	@echo ${CXX} ${CXXFLAGS} -o ${@} ${<}
else
	@echo -  CXX	${<}
endif
endif

	@${CXX} ${CXXFLAGS} -o ${@} ${<}

ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

# The rule for building the object file from each assembly source file.
${BUILD_DIR}/%.o: %.S $(PROJECTMK) | $(BUILD_DIR)
ifneq "${VERBOSE}" ""
	@echo ${CC} ${AFLAGS} -o ${@} -c ${<}
else
	@echo -  AS	${<}
endif

	@${CC} ${AFLAGS} -o ${@} -c ${<}

ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

# The rule for creating an object library.
${BUILD_DIR}/%.a: $(PROJECTMK)
ifeq "$(_OS)" "windows_msys"
# Certain MinGW tools and programs running in Windows MSYS are not guaranteed to perform
# automatic path translations from UNIX-style to native Windows paths especially when
# input file paths are read from a text file (e.g. ar_args.txt). This can cause missing or
# incorrect file path errors. Using 'cygpath' ensures proper path handling.
# Notes:
#	- 'cygpath' is a default utility that comes with MSYS
#	- The '-m' option format conversions to Windows path with forward slahes (e.g. C:/windows/path)
#   - Does not impact files that are generated (ouput binaries, .map files, etc)
	@echo -cr $(shell cygpath -m ${@}) $(shell cygpath -m ${^}) > ${BUILD_DIR}/ar_args.txt
else
	@echo -cr ${@} ${^} > ${BUILD_DIR}/ar_args.txt
endif

ifneq "$(VERBOSE)" ""
ifeq "$(_OS)" "windows_msys"
	@echo ${AR} -cr $(shell cygpath -m ${@}) $(shell cygpath -m ${^})
else
	@echo ${AR} -cr ${@} ${^}
endif
else
	@echo -  AR	${@}
endif
	@${AR} @${BUILD_DIR}/ar_args.txt

ifeq ($(STRIP_LIBRARIES),1)
ifneq "$(ECLIPSE)" ""
	@echo ${STRIP} $(call fixpath,${@}) | sed 's/-I\/\(.\)\//-I\1:\//g'
else

ifneq "$(VERBOSE)" ""
	@echo ${STRIP} --strip-unneeded ${@}
else
	@echo -  STRIP ${@}
endif # VERBOSE

endif # ECLIPSE

	@${STRIP} --strip-unneeded ${@}

endif # STRIP_LIBRARIES

# The rule for building the object file from binary source file.
# Resulting object will have the following symbols
# _binary_<file_name>_bin_start
# _binary_<file_name>_bin_end
# _binary_<file_name>_bin_size
${BUILD_DIR}/%.o: %.bin $(PROJECTMK) | $(BUILD_DIR)
ifneq "$(VERBOSE)" ""
	echo ${OBJCOPY} -I binary -B arm -O elf32-littlearm --rename-section	\
	    .data=.text ${<} ${@}
else
	echo -  CP	${<}
endif

	@${OBJCOPY} -I binary -B arm -O elf32-littlearm --rename-section		\
	.data=.text ${<} ${@}

ifeq "$(CYGWIN)" "True"
	@sed -i -r -e 's/([A-Na-n]):/\/cygdrive\/\L\1/g' -e 's/\\([A-Za-z])/\/\1/g' ${@:.o=.d}
endif

##
# Macro for linker arguments for easier argument handing with linking recipe.
# The default path format is UNIX-style.
#
# Note: The parameters are set up to allow for easier and proper path handling
# when automatic path translations can not be assumed for Windows MSYS environments.
#
# Parameters:
#	$(1) <= Linker rule target (${@})
#	$(2) <= File inputs for linker (${^})
define get_ln_args
	-T $(if $(filter "${_OS}","windows_msys"),$(shell cygpath -m ${LINKERFILE}),${LINKERFILE})	\
	--entry ${ENTRY}																			\
	${LDFLAGS}																					\
	-o ${1}																						\
	$(filter %.o, ${2})																			\
	-Xlinker --start-group																		\
	$(filter %.a, ${2})																			\
	${PROJ_LIBS}																				\
	${STD_LIBS}																					\
	-Xlinker --end-group
endef

# The rule for linking the application.
${BUILD_DIR}/%.elf: $(PROJECTMK) | $(BUILD_DIR)
# This rule parses the linker arguments into a text file to work around issues
# with string length limits on the command line
ifeq "$(_OS)" "windows_msys"
# Certain MinGW tools and programs running in Windows MSYS are not guaranteed to run
# automatic path translations from UNIX-style to native Windows paths especially when
# input file paths are read from a text file (e.g. ln_args.txt). This can cause missing
# or incorrect file path errors. Using 'cygpath' ensures proper path handling.
#
# Notes:
#	- 'cygpath' is a default utility for MSYS and Cygwin
#	- The '-m' option format conversions to Windows path with forward slahes (e.g. C:/windows/path)
# 	- Does not impact files that are generated (ouput binaries, .map files, etc)
	@echo $(call get_ln_args,$(shell cygpath -m ${@}),$(shell cygpath -m ${^}))	\
			> ${BUILD_DIR}/ln_args.txt
else
	@echo $(call get_ln_args,${@},${^}) > ${BUILD_DIR}/ln_args.txt
endif

ifneq "$(VERBOSE)" ""
ifeq "$(_OS)" "windows_msys"
	@echo ${LD} $(call get_ln_args,$(shell cygpath -m ${@}),$(shell cygpath -m ${^}))
else
	@echo ${LD} $(call get_ln_args,${@},${^})
endif
else
	@echo -  LD	${@}
endif

	@${LD} @${BUILD_DIR}/ln_args.txt

# Create S-Record output file
%.srec: %.elf
ifneq "$(VERBOSE)" ""
	@echo ${OBJCOPY} -O srec ${<} ${@}
else
	@echo Creating ${@}
endif

	@$(OBJCOPY) -O srec $< ${@}

# Create Intex Hex output file
%.hex: %.elf
ifneq "$(VERBOSE)" ""
	@echo ${OBJCOPY} -O ihex ${<} ${@}
else
	@echo Creating ${@}
endif

	@$(OBJCOPY) -O ihex $< ${@}

# Create binary output file
%.bin: %.elf
ifneq "$(VERBOSE)" ""
	@echo ${OBJCOPY} -O binary ${<} ${@}
else
	@echo Creating ${@}
endif

	@$(OBJCOPY) -O binary $< ${@}

# Create disassembly file
%.dasm: %.elf
ifneq "$(VERBOSE)" ""
	@echo $(OBJDUMP) -S ${<} ${@}
else
	@echo Creating ${@}
endif

	@$(OBJDUMP) -S $< > ${@}

################################################################################
.PHONY: debug
debug:
	@echo OS = ${_OS}
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
$(BUILD_DIR)/_empty_tmp_file.c: | $(BUILD_DIR)
	@echo "// Temp file used to detect compiler defs at build time.  Safely ignore/delete" > $(BUILD_DIR)/_empty_tmp_file.c

.PHONY: project_defines
project_defines: $(BUILD_DIR)/project_defines.h

$(BUILD_DIR)/project_defines.h: $(BUILD_DIR)/_empty_tmp_file.c $(PROJECTMK) | $(BUILD_DIR)
	@echo "// This is a generated file that's used to detect definitions that have been set by the compiler and build system." > $@
	@$(CC) -E -P -dD $(BUILD_DIR)/_empty_tmp_file.c $(filter-out -MD,$(CFLAGS)) >> $@

################################################################################
# Add a rule for querying the value of any Makefile variable.  This is useful for
# IDEs when they need to figure out include paths, value of the target, etc. for a
# project
# Set QUERY_VAR to the variable to inspect.
# The output must be parsed, since other Makefiles may print additional info strings.
# The relevant content will be on its own line, and separated by an '=' character.
# Ex: make query QUERY_VAR=TARGET
# will return
# TARGET=MAXxxxxx
ifeq "$(MAKECMDGOALS)" "query"
SUPPRESS_HELP := 1
endif
.PHONY: query
query:
	@echo
ifneq "$(QUERY_VAR)" ""
	$(foreach QUERY_VAR,$(QUERY_VAR),$(info $(QUERY_VAR)=$($(QUERY_VAR))))
else
	$(MAKE) debug
endif
