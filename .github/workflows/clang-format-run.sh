#!/bin/bash

# Remove the submodules
set +e
git submodule deinit --force --all
set -e

# Find the C files
CFILES=$(find . -iname "*.c")

for c_file in ${CFILES}
do
  # Remove single line ';', these confuse clang-format
  perl -i -pe 's/\s+;\s/{}\n/' ${c_file}

  # Format the files
  clang-format --verbose -style=file -i ${c_file}

done


# Reformat all of the header files
HFILES=$(find . -iname "*.h" -not -name "*regs*")
for h_file in ${HFILES}
do
  clang-format --verbose -style=file -i ${h_file}
done
