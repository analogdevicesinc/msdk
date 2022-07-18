#!/bin/sh

# Copyright (C) 2017-2018 Maxim Integrated Products, Inc., All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
# OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Except as contained in this notice, the name of Maxim Integrated 
# Products, Inc. shall not be used except as stated in the Maxim Integrated 
# Products, Inc. Branding Policy.
#
# The mere transfer of this software does not imply any licenses
# of trade secrets, proprietary technology, copyrights, patents,
# trademarks, maskwork rights, or any other form of intellectual
# property whatsoever. Maxim Integrated Products, Inc. retains all 
# ownership rights.

usage() {
    echo
    echo "Runs an SCP script for MAX32xxx SecureROM loader."
    echo "Copyright (c)  Maxim Integrated Products, Inc. 2017-2018"
    echo ""
    echo "**** CAUTION: This tool is not PCI compliant. It is for development purposes only. ****"
    echo ""
    echo " MAX32xxx pre-requisites:"
    echo "   - CRK in OTP must match the signing key use for generating the SCP packets"
    echo ""
    echo " Syntax: sendscp.sh <serialport> <input_dir>"
    echo
    echo "    <serialport> = serial port device (e.g. /dev/ttyS0 (linux) or COM1 (windows))"
    echo "    <input_dir> = directory that contains the SCP packet list"
    echo ""
    echo "Note: "
    echo "See also: build_application.sh to build an SCP script."

}

TOOLDIR=$(readlink -e $(dirname $0))


case $# in
2)	readonly serialport=$1
	readonly input=$2
	;;
*)	usage  >&2
	exit 2
	;;
esac

echo ""

if [ ! -d "$input" ]; then
echo "Error:: the <input_dir> ($input) does not exist."
exit 1
fi

cd -- "$input"  ||  exit

#identifying the OS, as cygwin uses a serial_sender.exe while linux directly uses the python application
system=$(uname -s)
case $system in
*CYGWIN*) readonly serial_sender_bin=serial_sender.exe
	;;
*Linux*)  
	readonly serial_sender_bin=serial_sender.py
	;;
*)	echo >&2 "Unknown system \`$system'!"
	exit 1
	;;
esac

if [ ! -f packet.list ]; then
echo "Error:: the <input_dir> ($input) does not seem to contain a SCP script."
exit 1
fi

echo "Ready to execute $(readlink -e .)"
read -p "Power cycle the MAX32xxx system then press [Enter]" reply
echo "Please wait..."

$TOOLDIR/../bin/serial_sender/$serial_sender_bin -s$serialport -t 2 -v packet.list --serial-baudrate 78008

if [ $? -ne 0 ] ; then
echo "ERROR."
echo "Make sure you have no terminal opened on $serialport."
exit 1
fi

echo "SUCCESS."
