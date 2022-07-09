#!/bin/bash


if [ $# -lt 2 ]; then
	echo " usage error"
	echo " Syntax: ./create_example_list.sh CHIP EvKit "
	echo " ForExample ./create_example_list.sh MAX32570 EvKit_V1"
	exit 1
fi

scriptDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

chip=$1
destFile=$2
declare -a examples


# Remove existing file
if [ -f $destFile ]
then
	rm $destFile
fi


# Find number of examples
for item in $(ls -d $scriptDIR/../Examples/$chip/*/)
do
	item=${item%%/}
	item=${item##*/}
	
	examples+=($item)
done

# Add Items
numberOfItem=${#examples[@]}
for ((i=0; i < $numberOfItem - 1; i++))
do
	echo ${examples[i]} >> $destFile
done

# add last line without new line
echo -n ${examples[i]} >> $destFile	

