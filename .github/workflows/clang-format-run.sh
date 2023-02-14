#!/bin/bash

# Run this from the root of the msdk
# bash -ex .github/workflows/clang-format-run.sh
# 
# Optional arguments to include the changes file
# bash -ex .github/workflows/clang-format-run.sh main.c src/foo.c

CLANG_VERSION=14
CHANGES_NEEDED=0
FILES=""

if [[ $# -eq 0 ]]
then
  # Find the c files
  FILES=$(find . -iname "*.c" -not -name "*cnn.c" -a -not -name "*softmax.c" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\|LVGL\)/.*")

  # Find the header files
  FILES=$FILES+$(find . -iname "*.h" -not -name "*regs*" -a -not -name "*weights.h" -a -not -name "*cnn.h" -a -not -name "*sampledata.h" -a -not -name "*sampleoutput.h" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\|LVGL\)/.*")
else
  # Accumulate the input arguments into FILES
  FILES="$*"
fi

for file in ${FILES}
do
  # Determine if we need to make changes
  set +e
  clang-format-${CLANG_VERSION} --dry-run --Werror -style=file --verbose ${file}
  RETVAL=$?
  set -e

  if [[ $RETVAL -ne 0 ]]
  then
    CHANGES_NEEDED=1
  fi

  # Format the files, this will turn while(1); into while(1)\n;
  clang-format-${CLANG_VERSION} --verbose -style=file -i ${file}

  # Remove single line ';' and replace with "{}"
  perl -i -pe 's/\s+;\s/{}\n/' ${file}

  # Re-format the files
  clang-format-${CLANG_VERSION} --verbose -style=file -i ${file}

done

# Test if files have changed
exit $CHANGES_NEEDED
