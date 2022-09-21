# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

# Build directory for image that peer will be updated with
FW_UPDATE_DIR=../BLE_otas

# Firmware update files, do not rename
FW_UPDATE_BIN=fw_update.bin
FW_UPDATE_OBJ=build/fw_update.o

PROJ_OBJS = ${FW_UPDATE_OBJ}

# Target for creating the fw_update bin file
${FW_UPDATE_BIN}:
	$(MAKE) -C ${FW_UPDATE_DIR}
	$(MAKE) -C ${FW_UPDATE_DIR} build/${PROJECT}.bin
	@cp ${FW_UPDATE_DIR}/build/${PROJECT}.bin ${FW_UPDATE_BIN}

${FW_UPDATE_OBJ}: fw_update.S ${FW_UPDATE_BIN}
	@mkdir -p build
	@${CC} ${AFLAGS} -o ${@} -c fw_update.S

clean.ota:
	@rm -f ${FW_UPDATE_BIN}

clean: clean.ota
