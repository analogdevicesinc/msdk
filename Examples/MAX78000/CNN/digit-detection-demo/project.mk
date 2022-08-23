# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default project configuration isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available config variables.

ifeq "$(BOARD)" "EvKit_V1"
VPATH += TFT/evkit/resources
PROJ_CFLAGS+=-DTFT_ENABLE
endif
ifeq "$(BOARD)" "FTHR_RevA"
VPATH += TFT/fthr
#PROJ_CFLAGS+=-DTFT_ENABLE    
endif

IPATH += TFT/evkit/resources

