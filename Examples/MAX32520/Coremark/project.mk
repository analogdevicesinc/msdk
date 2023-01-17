# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Add your config here!

IPATH+=$(MAXIM_PATH)/Libraries/Coremark
VPATH+=$(MAXIM_PATH)/Libraries/Coremark

MXC_OPTIMIZE_CFLAGS=-O2

PROJ_CFLAGS+=-funroll-all-loops
PROJ_CFLAGS+=-fgcse-sm
PROJ_CFLAGS+=-fgcse-las
PROJ_CFLAGS+=-finline-functions
PROJ_CFLAGS+=-finline-limit=1000
PROJ_CFLAGS+=-DPERFORMANCE_RUN=1
PROJ_CFLAGS+=-DITERATIONS=4000

MFLOAT_ABI=soft

SRCS+=core_main.c
SRCS+=core_list_join.c
SRCS+=core_matrix.c
SRCS+=core_state.c
SRCS+=core_util.c
SRCS+=core_portme.c

AUTOSEARCH=0
