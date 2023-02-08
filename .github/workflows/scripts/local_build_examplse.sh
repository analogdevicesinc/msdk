#!/usr/bin/env bash

echo "#############################################################################################"
echo "# ./local_build_examples.sh MSDK_path                                                       #"
echo "#############################################################################################"

#--------------------------------------------------------------------------------------------------
# Init
function init
{
    echo "Function: ${FUNCNAME[0]} $@"

    MSDK=$1
    echo MSDK: $MSDK
    if [ ! -d "$MSDK" ]; then
        echo "Err: invalid MSDK path."
        exit 1
    fi

    cd $MSDK
    echo "PWD: "`pwd`
}


#--------------------------------------------------------------------------------------------------
# Build all Hello_World projects
function build_hello_world
{
    echo "Function: ${FUNCNAME[0]}"
    echo

    cd Examples

    # Rebuild all of the peripheral drivers and build all of the Hello_World examples
    # Exclude the MAX32572, currently in development
    # Exclude the MAX32570, examples are located in another repo
    SUBDIRS=$(find . -type d -not -path "./MAX3257*" -name "Hello_World")
    echo SUBDIRS: $SUBDIRS
    echo
    for dir in ${SUBDIRS}
    do
        echo
        echo "---------------------------------------"
        echo ${dir}
        echo "---------------------------------------"
        echo
        make -C ${dir} clean
        make -C ${dir} libclean
        make -C ${dir} -j8
    done

    echo
}


#--------------------------------------------------------------------------------------------------
function build_all_examples
{
    echo "Function: ${FUNCNAME[0]}"
    echo

    cd Examples

    # Find all of the examples, 
    # Exclude the MAX32572, currently in development
    # Exclude the MAX32570, examples are located in another repo
    SUBDIRS=$(find . -mindepth 3 -not \( -path "./MAX3257*" -o -path "./MAX*/UCL*" \) -name "?akefile" -printf '%h\n')

    # Exclude some examples
    SUBDIRS=${SUBDIRS//"./MAX32672/Display/lvgl-8.0.2/tests"/}
    SUBDIRS=${SUBDIRS//"./MAX32572/MAX32572_Demo_FreeRTOS"/}
    SUBDIRS=${SUBDIRS//"./MAX32572/MAX32572_Demo_BareMetal"/}
    SUBDIRS=${SUBDIRS//"./MAX78000/CNN/mnist-streaming"/}

    echo SUBDIRS: $SUBDIRS
    echo

    for dir in ${SUBDIRS}
    do
        echo
        echo "---------------------------------------"
        echo ${dir}
        echo "---------------------------------------"
        echo
        make -C ${dir} clean
        make -C ${dir} libclean
        make -C ${dir} -j8
    done

    echo
}


#--------------------------------------------------------------------------------------------------
function build_max32690_wlp_v1
{
    echo "Function: ${FUNCNAME[0]}"
    echo

    cd Examples

    # Build MAX32690 all BLE projects for board WLP_V1
    SUBDIRS=$(find ./MAX32690 -type d -name "BLE*")
    echo SUBDIRS: ${SUBDIRS}
    echo

    for dir in ${SUBDIRS}
    do
        echo
        echo "---------------------------------------"
        echo ${dir}
        echo "---------------------------------------"
        echo
        make -C ${dir} clean
        make -C ${dir} libclean
        make -C ${dir} -j8 BOARD=WLP_V1
    done

    echo 
}


#--------------------------------------------------------------------------------------------------
# MAIN PROGRAM
start=$(date +%s%n)
echo "Started at ${start}."
init $@

export MAXIM_SBT_DIR=$(pwd)/Tools/SBT
echo MAXIM_SBT_DIR: $MAXIM_SBT_DIR

build_hello_world

build_all_examples

build_max32690_wlp_v1

#--------------------------------------------------------------------------------------------------
end=$(date +%s%n)
echo "Ended at ${end}."

elapsed_sec=$(( ${end} - ${start} ))
echo
echo "Used: ${elapsed_sec} (sec)."
echo "DONE!"