###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
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

# 'include' this file to detect the current OS.
# The result will be stored in the '_OS' variable.

# windows : native windows
# windows_msys : MSYS2 on windows
# windows_cygwin : Cygwin on windows (legacy config from old sdk)
# linux : Any linux distro
# macos : MacOS
ifeq "$(_OS)" ""

ifeq "$(OS)" "Windows_NT"
_OS = windows

UNAME_RESULT := $(shell uname -s 2>&1)
# MSYS2 may be present on Windows.  In this case,
# linux utilities should be used.  However, the OS environment
# variable will still be set to Windows_NT since we configure
# MSYS2 to inherit from Windows by default.
# Here we'll attempt to call uname (only present on MSYS2)
# while routing stderr -> stdout to avoid throwing an error 
# if uname can't be found.
ifneq ($(findstring CYGWIN, $(UNAME_RESULT)), )
CYGWIN=True
_OS = windows_cygwin
endif

ifneq ($(findstring MSYS, $(UNAME_RESULT)), )
MSYS=True
_OS = windows_msys
endif
ifneq ($(findstring MINGW, $(UNAME_RESULT)), )
MSYS=True
_OS = windows_msys
endif

else # OS

UNAME_RESULT := $(shell uname -s)
ifeq "$(UNAME_RESULT)" "Linux"
_OS = linux
endif
ifeq "$(UNAME_RESULT)" "Darwin"
_OS = macos
endif

endif

endif
