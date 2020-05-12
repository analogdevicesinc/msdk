#!/bin/bash
#
#	Script to build example
#

if [ $# -lt 1 ]; then
	echo "Usage error"
	echo "Usage: build.sh <operation> <float_type>"
	echo "<operation> : build or clean"
	echo "<float_type>: (optional) hard or soft"
	exit 1
fi

operation=$1
fp_type=$2

root_dir=$(pwd)

if [ "${fp_type}" == "hard" ] || [ "${fp_type}" == "Hard" ]; then
	fp_type="hard"
else
	fp_type="soft"
fi

export PATH="/c/MaximSDK/Tools/GNUTools/bin:/c/MaximSDK/Tools/MinGW/msys/1.0/bin:$PATH"
cp ${root_dir}/Tools/SBT/bin/sign_app.exe ./Tools/SBT/bin/sign_app


declare -a examples
examples+=('NFC_PCD_DTE_EMV_Loopback')

cd ./Examples/MAX32570/
	for example in "${examples[@]}"
	do
		cd $example/
			if [ "${operation}" == "clean" ] || [ "${operation}" == "Clean" ]; then
				echo "------- Clean START : $example  -------"
				make ECLIPSE=1 clean
				rm -rf ${root_dir}/Libraries/NFC/*/*.a
				echo " "
				echo "------- Clean END   : $example -------"
			else
				echo "------- Build START: $example -------"
				make ECLIPSE=1 MFLOAT_FLAGS=${fp_type}
				echo " "
				echo "------- Build END  : $example -------"
			fi
		cd ../
	done
cd $root_dir


exit 0
