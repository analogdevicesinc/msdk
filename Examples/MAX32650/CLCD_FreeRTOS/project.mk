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

#BOARD=FTHR_RevA
# ^ For example, you can uncomment this line to make the
# project build for the "FTHR_RevA" board.

# **********************************************************

MXC_OPTIMIZE_CFLAGS += -Og

PROJ_CFLAGS += -DUSAGE_GPIO_OUT -DUSAGE_TMR -DUSAGE_NO_IRQ
PROJ_CFLAGS += -DLCD_DISP_TASK_STATS -DLCD_DOUBLE_BUFFER

# If you have secure version of MCU (MAX32651), set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# Enable FreeRTOS library
LIB_FREERTOS=1
