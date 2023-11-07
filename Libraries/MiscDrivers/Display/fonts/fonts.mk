# If FONTS_DIR is not specified, this Makefile will locate itself.
FONTS_DIR ?= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# Add fonts to build.  The FONT variable can hold a single font, or a list 
# of fonts to use if multiple are needed.
# Ex: FONTS = LiberationSans16x16 LiberationSans12x12
FONTS ?=
# Font implementation files should follow the naming convention:
# $(FONT)$(FONT_SIZE).c

ifneq "$(FONTS)" ""
VPATH += $(FONTS_DIR)
IPATH += $(FONTS_DIR)

FONT_FILES := $(foreach font,$(FONT),$(FONTS_DIR)$(font).c)
SRCS += $(FONT_FILES)

# Add a compiler definition for each font (Ex: FONT_LiberationSans16x16)
PROJ_CFLAGS += $(foreach font,$(FONT),-DFONT_$(font))
endif