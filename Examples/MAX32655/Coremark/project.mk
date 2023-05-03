# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!

# Include Coremark library in search paths.
IPATH+=$(MAXIM_PATH)/Libraries/Coremark
VPATH+=$(MAXIM_PATH)/Libraries/Coremark

# To comply with the Coremark rules, these source files
# must be unmodified. They are located in the Coremark
# library.
SRCS+=core_main.c
SRCS+=core_list_join.c
SRCS+=core_matrix.c
SRCS+=core_state.c
SRCS+=core_util.c

# This source file can be modified, however it has
# already been set up for the MAX32xxx and MAX78xxx
# series microcontrollers.
SRCS+=core_portme.c

# CoreMark build flags
PROJ_CFLAGS+=-DPERFORMANCE_RUN=1  # Run the CoreMark test with performance parameters
PROJ_CFLAGS+=-DITERATIONS=4000    # Number of times the coremark test is executed

PROJ_CFLAGS+=-funroll-all-loops
PROJ_CFLAGS+=-fgcse-sm
PROJ_CFLAGS+=-fgcse-las
PROJ_CFLAGS+=-finline-functions
PROJ_CFLAGS+=-finline-limit=1000

MFLOAT_ABI=soft
MXC_OPTIMIZE_CFLAGS=-O2
AUTOSEARCH=0
