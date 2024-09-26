###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################
# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

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
