#!/usr/bin/env bash

echo "#############################################################################################"
echo "# ./unlock_plot.sh 1MSDK 2LOCK_FILE_LIST 3PER_RESULT_FILE 4NEED_TO_PLOT                     #"
echo "#############################################################################################"
echo
echo $0 $@
echo

if [ "x$2" == "x" ] || [ "x$3" == "x" ]; then
    echo "ERR: Invalid arguments."
    exit 2
fi

MSDK=$1
CURR_JOB_FILE=$2
all_in_one=$3
NEED_TO_PLOT=$4

# Use python 3.10.9
source ~/anaconda3/etc/profile.d/conda.sh && conda activate py3_10
python3 -c "import sys; print(sys.version)"
echo

echo "-----------------------------------------------------------------------------------"
echo "Unlock the used resource files."
echo

echo "Show lock files in folder Resource_Share."
ls -hal /home/$USER/Workspace/Resource_Share/*.txt
echo

if [ -f "${CURR_JOB_FILE}" ]; then
    echo "Show locked files for this job."
    cat ${CURR_JOB_FILE}
    echo

    while IFS= read -r line; do
        echo "python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py ${line}"
        python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py ${line}
    done <$CURR_JOB_FILE
else
    echo "${CURR_JOB_FILE} not exist."
fi
echo

echo "Show lock files in folder Resource_Share."
ls -hal /home/$USER/Workspace/Resource_Share/*.txt
echo

#----------------------------------------------------------------------------------------------------------------------
echo "Show results."
cat ${all_in_one}
echo

if [ "${NEED_TO_PLOT}" != "True" ]; then
    echo "No need to plot the PER results."
    exit 0
fi

echo "-----------------------------------------------------------------------------------"
echo "Plot the results"
echo

cd ${MSDK}/.github/workflows/scripts
echo PWD: `pwd`
echo

chmod u+x plot_per_results.py
echo "python3 plot_per_results.py ${all_in_one} desc basename"
python3 plot_per_results.py ${all_in_one} desc basename

echo "$0: DONE!"
