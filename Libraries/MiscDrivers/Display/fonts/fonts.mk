###############################################################################
 #
 # Copyright 2023 Analog Devices, Inc.
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
# If FONTS_DIR is not specified, this Makefile will locate itself.
FONTS_DIR ?= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
FONTS_DIR := $(FONTS_DIR)
#         ^ immediate expansion fixes weird bug where IPATH would not get
# set properly after multiple levels of "include ..."s 

IPATH += $(FONTS_DIR)
VPATH += $(FONTS_DIR)

# Add fonts to build.  The FONT variable can hold a single font, or a list 
# of fonts to use if multiple are needed.
# Ex: FONTS = LiberationSans16x16 LiberationSans12x12
FONTS ?=
# Font implementation files should follow the naming convention:
# $(FONT)$(FONT_SIZE).c

ifneq "$(FONTS)" ""
FONT_FILES := $(foreach font,$(FONTS),$(FONTS_DIR)$(font).c)
SRCS += $(FONT_FILES)

# Add a compiler definition for each font (Ex: FONT_LiberationSans16x16)
PROJ_CFLAGS += $(foreach font,$(FONTS),-DFONT_$(font))
endif