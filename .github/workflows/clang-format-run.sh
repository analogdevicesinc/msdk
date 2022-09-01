#!/bin/bash

# Remove the submodules
set +e
git submodule deinit --force --all
set -e

# Find the C files
CFILES=$(find . -iname "*.c")

# Remove single line ';', these confuse clang-format
parallel -j 8 perl -i -pe 's/\s+;\s/{}\n/' -- $CFILES

# Format the files
parallel -j 8 clang-format --verbose -style=file -i -- $CFILES

# Reformat all of the header files
HFILES=$(find . -iname "*.h" -not -name "*regs*")
parallel -j 8 clang-format --verbose -style=file -i $HFILES
