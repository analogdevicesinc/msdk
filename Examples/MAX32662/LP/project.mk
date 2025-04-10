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

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

# This Low-Power example shuts down some SRAM instances.
# To ensure application code does not use the SRAM that will
# be shut down, the available SRAM size is reduced with a
# custom linkerfile.  The section below selects the
# appropriate file depending on whether the SBTs are used or not.
ifeq ($(SBT),1)
# This linkerfile contains extra sections for compatibility with the Secure Boot Tools (SBT).
override LINKERFILE = lp-sla.ld
else
# This linkerfile is for use with standard non-secure applications.
override LINKERFILE = lp-nonsecure.ld
endif # SBT
