###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
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

################################################################################
# This file can be included in a project makefile to build the library for the 
# project.
###############################################################################

ifeq "$(LIB_CLI_DIR)" ""
# If CLI_DIR is not specified, this Makefile will locate itself.
LIB_CLI_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

IPATH += ${LIB_CLI_DIR}/inc
VPATH += ${LIB_CLI_DIR}/src
SRCS += cli.c

# By default, with USE_CLI_LIB_IRQHANDLER defined, the CLI library will handle the
# UART interrupts internally. Users have the option to define their own UART IRQ
# Handler for the CLI UART in their application. If users choose to define their own
# IRQ handler they should delete this definition of USE_CLI_LIB_IRQHANDLER and call
# MXC_CLI_Handler in their handler function when the CLI is in use.
LIB_CLI_USE_DEFAULT_HANDLER ?= 1
ifeq "$(LIB_CLI_USE_DEFAULT_HANDLER)" "1"
PROJ_CFLAGS += -DUSE_CLI_LIB_IRQHANDLER
endif

# Use absolute paths if building within eclipse environment.
ifeq "$(ECLIPSE)" "1"
SRCS := $(abspath $(SRCS))
endif
