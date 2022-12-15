#!/usr/bin/bash

echo "#############################################################################################"
echo "# Match the test in Build_Examples.yml                                                      #"
echo "# Usage: local_Build_Examples.sh /path/of/msdk                                              #"
echo "#############################################################################################"
echo

#set -x
set -e

if [ $# != 1 ]; then
    echo "Usage: local_Build_Examples.sh /path/of/msdk"
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
# This environment variable is required for the SBTs.
# It must be set to the absolute path inside the Github repo.
export MAXIM_SBT_DIR=$(pwd)/Tools/SBT
cd Examples
# Rebuild all of the peripheral drivers and build all of the Hello_World examples
# Exclude the MAX32572, currently in development
# Exclude the MAX32570, examples are located in another repo
SUBDIRS=$(find . -type d -not -path "./MAX3257*" -name "Hello_World")
for dir in ${SUBDIRS}
do
    echo
    echo "-----------------------------------------------------------------------------------------"
    echo "Build ${dir}"
    echo "-----------------------------------------------------------------------------------------"
    echo 
    make -C ${dir} clean MAXIM_PATH=$MSDK
    make -C ${dir} libclean MAXIM_PATH=$MSDK
    make -C ${dir} -j8 MAXIM_PATH=$MSDK
done

# Find all of the examples, 
# Exclude the MAX32572, currently in development
# Exclude the MAX32570, examples are located in another repo
SUBDIRS=$(find . -mindepth 3 -not \( -path "./MAX3257*" -o -path "./MAX*/UCL*" \) -name "?akefile" -printf '%h\n')
# Exclude some examples
SUBDIRS=${SUBDIRS//"./MAX32672/Display/lvgl-8.0.2/tests"/}
SUBDIRS=${SUBDIRS//"./MAX32572/MAX32572_Demo_FreeRTOS"/}
SUBDIRS=${SUBDIRS//"./MAX32572/MAX32572_Demo_BareMetal"/}
SUBDIRS=${SUBDIRS//"./MAX78000/CNN/mnist-streaming"/}
for dir in ${SUBDIRS}
do
    echo
    echo "-----------------------------------------------------------------------------------------"
    echo "Build ${dir}"
    echo "-----------------------------------------------------------------------------------------"
    echo 
    set -x
    make -C ${dir} clean MAXIM_PATH=$MSDK
    make -C ${dir} libclean MAXIM_PATH=$MSDK
    make -C ${dir} distclean MAXIM_PATH=$MSDK
    make -C ${dir} -j 1 MAXIM_PATH=$MSDK
    set +x
done

echo 
echo "DONE! ---------------------------------------------------------------------------------------"
echo