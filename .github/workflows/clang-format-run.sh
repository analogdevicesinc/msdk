#!/bin/bash

# Run this from the root of the msdk
# bash -ex .github/workflows/clang-format-run.sh
# 
# Optional arguments to include the changes file
# bash -ex .github/workflows/clang-format-run.sh main.c src/foo.c

# Get CLANG_VERSION from environment if available
if [[ $CLANG_VERSION == "" ]]; then
  CLANG_VERSION=14
fi

CHANGES_NEEDED=0
FILES=""

if [[ $# -eq 0 ]]
then
  # Find the c files
  FILES=$(find . -iname "*.c" -not -name "*cnn.c" -a -not -name "*softmax.c" -a -not -regex ".*/Examples/.*/Coremark" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\|LVGL\|Coremark\)/.*")

  # Find the header files
  FILES=$FILES+$(find . -iname "*.h" -not -name "*regs*" -a -not -name "*weights.h" -a -not -name "*cnn.h" -a -not -name "*sampledata.h" -a -not -name "*sampleoutput.h" -a -not -regex ".*/Examples/.*/Coremark" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\|LVGL\|Coremark\)/.*")
else
  # Accumulate the input arguments into FILES
  FILES="$*"
fi

for file in ${FILES}
do
  if [ -f ${file} ];
  then

    set +e
    clang-format-${CLANG_VERSION} --verbose --style=file -n -Werror ${file}
    RETVAL=$?
    set -e

    if [ $RETVAL != 0 ];
    then
      echo "===================================================="

      # Format the files, this will turn while(1); into while(1)\n;
      clang-format-${CLANG_VERSION} -style=file ${file} > ${file}.clang

      # Remove single line ';' and replace with "{}"
      perl -i -pe 's/\s+;\s/{}\n/' ${file}.clang

      # Re-format the files
      clang-format-${CLANG_VERSION} -style=file -i ${file}.clang

      # Print the diff, for when this is run in a check
      set +e
      diff -u --color=always ${file} ${file}.clang
      set -e

      # Replace the temp file
      mv ${file}.clang ${file}

      CHANGES_NEEDED=1
      echo "===================================================="
      echo ""
    fi
  fi
done

# Test if files have changed
exit $CHANGES_NEEDED
