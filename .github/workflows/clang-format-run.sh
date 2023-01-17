#!/bin/bash

CLANG_VERSION=14

# Find the C files
CFILES=$(find . -iname "*.c" -not -name "*cnn.c" -a -not -name "*softmax.c" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\|Coremark\)/.*")

for c_file in ${CFILES}
do
  # Format the files, this will turn while(1); into while(1)\n;
  clang-format-${CLANG_VERSION} --verbose -style=file -i ${c_file}

  # Remove single line ';' and replace with "{}"
  perl -i -pe 's/\s+;\s/{}\n/' ${c_file}

  # Re-format the files
  clang-format-${CLANG_VERSION} --verbose -style=file -i ${c_file}

done


# Reformat all of the header files
HFILES=$(find . -iname "*.h" -not -name "*regs*" -a -not -name "*weights.h" -a -not -name "*cnn.h" -a -not -name "*sampledata.h" -a -not -name "*sampleoutput.h" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\|Coremark\)/.*")
for h_file in ${HFILES}
do
  clang-format-${CLANG_VERSION} --verbose -style=file -i ${h_file}
done
