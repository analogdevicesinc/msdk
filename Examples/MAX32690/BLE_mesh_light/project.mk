# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0
TRACE=1

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
TRACE = 1

# Add services directory to build
IPATH += common
VPATH += common

VPATH += $(shell find ../../../Libraries/Cordio/ble-mesh-apps/ -type d)
IPATH += $(shell find ../../../Libraries/Cordio/ble-mesh-apps/ -type d)

VPATH += $(shell find ../../../Libraries/Cordio/ble-mesh-model/ -type d)
IPATH += $(shell find ../../../Libraries/Cordio/ble-mesh-model/ -type d)

VPATH += $(shell find ../../../Libraries/Cordio/ble-mesh-profile/ -type d)
IPATH += $(shell find ../../../Libraries/Cordio/ble-mesh-profile/ -type d)




# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/include/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/include/
# IPATH += ../../../Libraries/Cordio/ble-mesh-model/include/
# IPATH += ../../../Libraries/Cordio/ble-mesh-model/sources/include/
# IPATH += ../../../Libraries/Cordio/ble-mesh-apps/include/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/bearer/gatt/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/ble-profiles/services/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/stack/include/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/ble-profiles/profiles/mprxs/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/ble-profiles/profiles/mprvs/
# IPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/bearer/adv/


# INC_DIRS += \
#     ../../../Libraries/Cordio/ble-mesh-apps/include \
#     ../../../Libraries/Cordio/ble-mesh-apps/sources/common \
#     ../../../Libraries/Cordio/ble-mesh-apps/sources/light

# C_FILES += \
#     $(sort $(wildcard ../../../Libraries/Cordio//ble-mesh-apps/sources/common/*.c)) \
#     $(sort $(wildcard ../../../Libraries/Cordio//ble-mesh-apps/sources/light/*.c)) \
#     ../../../Libraries/Cordio/ble-mesh-apps/build/light/main.c \
#     ../../../Libraries/Cordio/ble-mesh-apps/build/light/stack_light.c

# #--------------------------------------------------------------------------------------------------
# #     Mesh Models
# #--------------------------------------------------------------------------------------------------

# include ../../../Libraries/Cordio/ble-mesh-model/build/common/gcc/sources_models.mk

# #--------------------------------------------------------------------------------------------------
# #     Mesh Stack
# #--------------------------------------------------------------------------------------------------

# include ../../../Libraries/Cordio/ble-mesh-profile/build/common/gcc/sources_bearer.mk
# include ../../../Libraries/Cordio/ble-mesh-profile/build/common/gcc/sources_ble-profiles.mk
# include ../../../Libraries/Cordio/ble-mesh-profile/build/common/gcc/sources_stack.mk
# include ../../../Libraries/Cordio/ble-mesh-profile/build/common/gcc/sources_provisioning.mk

# VPATH += ../../../Libraries/Cordio/ble-mesh-model/sources/lighthslsr/
# VPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/
# VPATH += ../../../Libraries/Cordio/ble-mesh-model/sources/
# VPATH += ../../../Libraries/Cordio/ble-mesh-apps/sources/
# VPATH += ../../../Libraries/Cordio/ble-mesh-profile/sources/bearer/gatt/
# VPATH += ../../../Libraries/Cordio/ble-mesh-apps/sources/provisioner/