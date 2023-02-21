#!/usr/bin/env bash

echo "#############################################################################################"
echo "# ./unlock_plot.sh 1MSDK 2LOCK_FILE_LIST 3PER_RESULT_FILE 4NEED_TO_PLOT 5JOB_CURR_TIME      #"
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
JOB_CURR_TIME=$5

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
res=$?
echo "Exit: $res"
echo

if [ "${NEED_TO_PLOT}" != "True" ] || [[ res -ne 0 ]]; then
    echo "No need to plot the PER results."
    exit 0
fi

echo "-----------------------------------------------------------------------------------"
echo "Plot the results"
echo

cd ${MSDK}/.github/workflows/scripts
echo PWD: `pwd`
REPO_NAME=$(basename `git rev-parse --show-toplevel`)
echo "REPO_NAME: ${REPO_NAME}"
SHA=$(git rev-parse --short HEAD)
echo "SHA: ${SHA}"
DESC="Repo: ${REPO_NAME}, SHA: ${SHA}"
echo "DESC: ${DESC}"
echo

chmod u+x plot_per_results.py
echo python3 plot_per_results.py ${all_in_one} "${DESC}" basename --job_time ${JOB_CURR_TIME}
python3 plot_per_results.py ${all_in_one} "${DESC}" basename --job_time ${JOB_CURR_TIME}

echo "$0: DONE!"
