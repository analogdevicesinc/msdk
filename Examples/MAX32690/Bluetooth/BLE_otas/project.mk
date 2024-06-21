# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# If you have secure version of MCU, set SBT=1 to generate signed binary
# For more information on how sing process works, see
# https://www.analog.com/en/education/education-library/videos/6313214207112.html
SBT=0

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

BUILD_BOOTLOADER?=1

# Use local linkerfile
LINKERFILE = ota.ld

# build bootloader
ifeq ($(BUILD_BOOTLOADER), 1)
BOOTLOADER_DIR=../Bootloader

BUILD_DIR := $(abspath ./build)
BOOTLOADER_BUILD_DIR := $(BUILD_DIR)/buildbl

BOOTLOADER_BIN=$(BOOTLOADER_BUILD_DIR)/bootloader.bin
BOOTLOADER_OBJ=$(BOOTLOADER_BUILD_DIR)/bootloader.o

PROJ_OBJS = ${BOOTLOADER_OBJ}

.PHONY: bl_bin
bl_bin: $(BOOTLOADER_BIN)

${BOOTLOADER_BIN}:
	$(MAKE) -C ${BOOTLOADER_DIR} BUILD_DIR=$(BOOTLOADER_BUILD_DIR) PROJECT=bootloader
	$(MAKE) -C $(BOOTLOADER_DIR) BUILD_DIR=$(BOOTLOADER_BUILD_DIR) $(BOOTLOADER_BIN)

.PHONY: bl_obj
bl_obj: $(BOOTLOADER_OBJ)

${BOOTLOADER_OBJ}: bl_build.S ${BOOTLOADER_BIN}
	${CC} ${AFLAGS} -o ${@} -c bl_build.S
endif

ifeq ($(BOARD),FTHR)
$(error ERR_NOTSUPPORTED: This project is not supported for the FTHR board)
endif



# SET advertising name
PROJ_CFLAGS += -DADV_NAME=\"This\ is\ the\ real\ world\"


### CONFIGURE security
# /*! Authentication and bonding flags */
#PROJ_CFLAGS += -DAUTH_BOND='DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG | DM_AUTH_MITM_FLAG'

# /*! Initiator key distribution flags */
#PROJ_CFLAGS += -DINIT_KEY_DIST='DM_KEY_DIST_IRK'

# /*! Responder key distribution flags */
#PROJ_CFLAGS += -DRESP_KEY_DIST='DM_KEY_DIST_LTK | DM_KEY_DIST_IRK'

# /*! TRUE if Out-of band pairing data is present */
#PROJ_CFLAGS += -DOUT_OF_BAND=FALSE

# /*! TRUE to initiate security upon connection*/
PROJ_CFLAGS += -DINIT_SECURITY=TRUE
