# Makefile for linking against the CMSIS-DSP library.

ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../../
endif

# Include paths...
# DSP files
IPATH += $(CMSIS_ROOT)/5.9.0/DSP/Include
# Some newer CMSIS5 core include files, such as cmsis_compiler.h, etc.
IPATH += $(CMSIS_ROOT)/5.9.0/Core/Include

# TODO: Add target check for M3 core micros
# Add processor flag for arm_math.h
PROJ_CFLAGS+=-DARM_MATH_CM4

# Tell core_cm4.h that our CPU has an FPU by defining __FPU_PRESENT
PROJ_CFLAGS+=-D__FPU_PRESENT

# Where to find the DSP library file
PROJ_LDFLAGS += -L$(CMSIS_ROOT)/5.9.0/DSP/Lib

ifeq "$(MFLOAT_ABI)" ""
$(warning ***The 'MFLOAT_ABI' Makefile variable is not set!***  Using softfp CMSIS-DSP instructions by default.)
endif

# Link against hard or soft fp
ifeq "$(MFLOAT_ABI)" "hard"
PROJ_LIBS += arm_cortexM4lf_math
else
PROJ_LIBS += arm_cortexM4l_math
endif