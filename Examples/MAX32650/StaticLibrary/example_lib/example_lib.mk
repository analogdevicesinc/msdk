# Example static library Makefile

# **********************************************************
# Set the name of your library here. It will be built as CUSTOM_LIB_NAME_softfp.a,
# LIB_NAME_hardfp.a or just LIB_NAME.a under the project build directory.
CUSTOM_LIB_NAME=libExample
# Insert your source files below
CUSTOM_LIB_SRC_FILES = example.c
#CUSTOM_LIB_SRC_FILES += example_2.c

# Variables below do not need to be modified in general. 
CUSTOM_LIB_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CUSTOM_LIB_OUT_DIR = $(abspath $(CUSTOM_LIB_DIR)/../build)
CUSTOM_LIB_SRCS = $(addprefix $(CUSTOM_LIB_DIR),$(CUSTOM_LIB_SRC_FILES))

ifeq "$(LIBRARY_VARIANT)" ""
CUSTOM_LIB_OUT := $(CUSTOM_LIB_NAME).a
else
CUSTOM_LIB_OUT := $(CUSTOM_LIB_NAME)_$(LIBRARY_VARIANT).a
endif

# Add the library to the list of libraries
LIBS += $(CUSTOM_LIB_OUT_DIR)/$(CUSTOM_LIB_OUT)

# Add rule to build the library
$(CUSTOM_LIB_OUT_DIR)/$(CUSTOM_LIB_OUT): $(CUSTOM_LIB_SRCS)
	make -f $(abspath $(CUSTOM_LIB_DIR)/$(CUSTOM_LIB_NAME).make) lib BUILD_DIR=$(CUSTOM_LIB_OUT_DIR) \
		PROJECT_NAME=$(CUSTOM_LIB_NAME) SRC_FILES="$(CUSTOM_LIB_SRCS)" \
		PROJ_CFLAGS="$(PROJ_CFLAGS)" PROJ_LDFLAGS="$(PROJ_LDFLAGS)" \
		MXC_OPTIMIZE_CFLAGS=$(MXC_OPTIMIZE_CFLAGS) MFLOAT_ABI=$(MFLOAT_ABI) \
		DUAL_CORE=$(DUAL_CORE) RISCV_CORE=$(RISCV_CORE)
