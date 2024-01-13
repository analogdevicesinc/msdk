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

# If CAMERA_DIR is not specified, this Makefile will locate itself.
ifeq "$(CAMERA_DIR)" ""
CAMERA_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif

# Set empty default value.
CAMERA ?= 

ifneq "$(CAMERA)" ""
$(info Selected camera drivers: $(CAMERA))
# Add universal camera include/source path, and top-level camera driver file.
VPATH += $(CAMERA_DIR)
IPATH += $(CAMERA_DIR)
SRCS += camera.c
SRCS += sccb.c

# Function for converting upper-case to lower-case.  Believe it or not, this is better than relying on OS-specific shell utils... 
lowercase = $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$1))))))))))))))))))))))))))

# Add the camera-specific source file.  Drivers should follow the naming convention:
# 	[camera].c, where [camera] is the lower-case CAMERA variable to select the driver.
# Using this convention, we don't need to manually maintain a giant list of ifeq "$(CAMERA)" "..."  
SRCS += $(call lowercase,$(CAMERA)).c

# Add a compiler definition to the build that the app code can use to check the actively selected camera.
PROJ_CFLAGS += -DCAMERA_$(CAMERA)
endif
