###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 # (now owned by Analog Devices, Inc.)
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 ##############################################################################
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
