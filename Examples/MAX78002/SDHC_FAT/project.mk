# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default project configuration isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available config variables.

SDHC_DRIVER_DIR=$(LIBS_DIR)/SDHC
FAT32_DRIVER_DIR=$(SDHC_DRIVER_DIR)/ff13
include $(FAT32_DRIVER_DIR)/fat32.mk
include $(SDHC_DRIVER_DIR)/sdhc.mk

