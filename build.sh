#!/bin/bash
#
#	Script to build / clean and generate documentation
#

if [ $# -lt 1 ]; then
	echo "Usage error"
	echo "Usage: build.sh <operation> <float_type>"
	echo "<operation> : "
	echo "              build: To build NFC example"
	echo "              doc  : To generate doxygen documentation"
	echo "              clean: To clean generated files and create fresh setup"
	echo "<float_type>: (optional) hard or soft should be used for build only"
	echo "<doc_version>:(optional) vX.Y.Z, should be used for doc only"
	exit 1
fi

operation=$1
fp_type=$2
doc_version=$2

root_dir=$(pwd)

example="NFC_PCD_DTE_EMV_Loopback"

export PATH="/c/MaximSDK/Tools/GNUTools/10.3/bin:/c/MaximSDK/Tools/MSYS2/usr/bin:$PATH"

#
#	Decide Operation
#
case "$operation" in
	build)
		echo "------- Build START: $example -------"
		
		if [ "${fp_type}" == "hard" ] || [ "${fp_type}" == "Hard" ]; then
			fp_type="hard"
		else
			fp_type="soft"
		fi

		cp ${root_dir}/Tools/SBT/bin/sign_app.exe ${root_dir}/Tools/SBT/bin/sign_app

		cd ./Examples/MAX32570/$example/
			make MFLOAT_ABI=${fp_type}
			echo " "
		cd $root_dir
		
		echo "------- Build END  : $example -------"
		;;
	
	clean)	
		echo "------- Clean START : $example  -------"
		
		cd ./Examples/MAX32570/$example/
			make clean
			rm -rf ${root_dir}/Libraries/NFC/*/*.a
			
			# remove doxy files
			rm -rf Internal_Only/DoxyOutput
			rm -rf Internal_Only/doxy_warnings.log
			git checkout Internal_Only/doxyfile
			
			echo " "
		cd $root_dir	
		
		# revert out sing_app file
		git checkout ${root_dir}/Tools/SBT/bin/sign_app
			
		
		echo "------- Clean END   : $example -------"
		;;	
		
	doc)
		echo "------- Documentation START : $example  -------"
		
		cd ./Examples/MAX32570/$example/Internal_Only
		 
			gittag=$(doc_version)
			
			#gittag=`git describe --abbrev=0 --tags`
			#gittag=${gittag%%-*}

			#echo "Current tag: $gittag"

			out_dir="DoxyOutput"
			file_name="NFC_PCD_CSP"

			sed -i -e "s/<__PROJECT_VERSION__>/$gittag/"  ./doxyfile
			sed -i -e "s/<__CHM_FILE_NAME__>/$file_name/" ./doxyfile
			
			# define MAXIM_PATH
			export MAXIM_PATH="../../../.."
					
			doxygen ./doxyfile

			doxy_warning_file=$(cat ./doxy_warnings.log)

			if [[ $doxy_warning_file != "" ]]
			then
			   echo "Error!!!: Please check doxy_warning.log file:"
			   cat $doxy_warning_file
			   echo "------------------ END doxy warning file ---------------------"
			   cd $root_dir
			   exit 1
			fi

			# Create packet
			mv ./$out_dir/html/${file_name}.chm ./$out_dir/

			7z a -tzip ./$out_dir/${file_name}_html-$tag.zip ./$out_dir/html

		cd $root_dir
		
		echo "------- Documentation END   : $example -------"
		
		;;	
		
	*)
		echo "Error! Not supported command"
		exit 1
esac


exit 0
