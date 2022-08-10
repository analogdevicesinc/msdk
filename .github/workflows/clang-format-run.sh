#!/bin/bash

# Remove the submodules
set +e
git submodule deinit --force --all
set -e

# Reformat all of the c files
CFILES=$(find . -iname "*.c")
for cFile in ${CFILES}
do
  clang-format --verbose -style=file -i ${cFile}
done

# Reformat all of the header files
HFILES=$(find . -iname "*.h" -not -name "*regs*")
for hFile in ${HFILES}
do
  clang-format --verbose -style=file -i ${hFile}
done
