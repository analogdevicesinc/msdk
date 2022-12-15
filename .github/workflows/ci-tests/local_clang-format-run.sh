#!/usr/bin/bash

echo "#############################################################################################"
echo "# Match the test in clang-format-run.yml                                                    #"
echo "# Usage: local_clang-format-run.sh /path/of/msdk                                            #"
echo "#############################################################################################"
echo

#set -x
set -e

if [ $# != 1 ]; then
    echo "Usage: local_clang-format-change.sh /path/of/msdk"
    exit 1
fi

MSDK=$1

if [ ! -d "$1" ]; then
    echo "Invalid MSDK path."
    exit 2
fi

cd $MSDK
echo "PWD=$(pwd)"

#--------------------------------------------------------------------------------------------------
if [ $(hostname) == "yingcai-OptiPlex-790" ]; then 
    CLANG_VERSION=12
else
    CLANG_VERSION=14
fi

# Find the C files
CFILES=$(find . -iname "*.c" -not -name "*cnn.c" -a -not -name "*softmax.c" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\)/.*")

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
HFILES=$(find . -iname "*.h" -not -name "*regs*" -a -not -name "*weights.h" -a -not -name "*cnn.h" -a -not -name "*sampledata.h" -a -not -name "*sampleoutput.h" -a -not -regex ".*/Libraries/\(Cordio\|FCL\|FreeRTOS\|FreeRTOS\-Plus\|LC3\|littlefs\|lwIP\|MAXUSB\|SDHC\)/.*")
for h_file in ${HFILES}
do
  clang-format-${CLANG_VERSION} --verbose -style=file -i ${h_file}
done

echo "DONE! ---------------------------------------------------------------------------------------"