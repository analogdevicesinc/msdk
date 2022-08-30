#!/bin/bash
#
#

if [ $# -lt 1 ]; then
	echo "Usage error"
	exit 1
fi

cmd="$1"

#
#	Decide OS
#
case "$cmd" in
	update)
		cp /c/Users/sadik.ozer/Documents/VBShared/SBT/session_build/bin/windows/64/build_scp_session.exe .
		;;
	
	build)	
		image="$2"
        cp /c/Users/sadik.ozer/Documents/VBShared/SBT/session_build/bin/windows/64/build_scp_session.exe .
		./build_scp_session.exe -c MAX32651 -d script_file=set_scp_gpio_stimulus.txt scp/
		;;	
		
	sign)
		;;	
		
	send)
		image="$2"	
		send_scp.exe -c MAX32651 -s COM33 scp/
		;;
		
	#
	#	Below section is test purpose
	#
	clean)
		echo "Todo clean"
		;;
		
	*)
		echo "Error! Not supported OS"
		exit 1
esac


exit 0
