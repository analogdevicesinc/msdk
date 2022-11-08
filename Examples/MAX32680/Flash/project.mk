# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default setup isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available options.

LINKERFILE=$(TARGET_LC)_ram.ld
$(info This example executes out of RAM using a special linkerfile: $(LINKERFILE))
