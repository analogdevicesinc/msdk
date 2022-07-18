# This file can be used for project configuration.
# It's a sibling to the core "Makefile", which offers
# various configuration variables that you can set here
# if the default project configuration isn't suitable.

# See the comments in the "Makefile" for a detailed
# description of the default behavior and the full list of
# available config variables.

# This example is only compatible with the FTHR board,
# so we override the BOARD value to hard-set it.
override BOARD=FTHR_RevA
$(warning Warning: This project is forced to compile for the FTHR board only!)
