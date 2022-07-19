#!/bin/sh

# Copyright (C) 2012-2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
    echo "Signs a binary executable for MAX32651 SecureROM."
    echo "Copyright (c) 2012-2018 Maxim Integrated Products, Inc."
    echo
    echo "**** CAUTION: This tool is not PCI compliant. It is for development purposes only. ****"
    echo
    echo " MAX32651 pre-requisites:"
    echo "   - For MAX32651 rev A1 and above"
    echo "   - Chip must be in Phase 4"
    echo "   - CRK in OTP must match the signing key invoked here"
    echo
    echo " Syntax: sign_application <execfile> <keyfile>";
    echo
    echo "    <execfile>  = executable binary file to be signed"
    echo "    <keyfile>   = file containing signing key in UCL format"
    echo
    echo "NOTE: This tool creates the output file named <execfile>.sbin"
    echo
    echo "See also: build_application.sh to build an SCP script for"
    echo "          downloading the generated .sbin file."
    echo
}

# Parameters
verbose=0
signonly=yes
soc=


while [ $# -ge 1 ]; do
	case $1 in
	--soc=)	echo >&2 "error: No ROM/chip version specified"
		exit 4
		;;
	--soc=*)
		soc=${1#--soc=}
		;;
	--verbose)
		verbose=yes
		;;
	--)	shift
		break
		;;
	-*)	echo >&2 "error: Unknown option \`$1'"
		exit 5
		;;
	*)	break
		;;
	esac
	shift
done
readonly verbose

case $# in
2)	readonly input=$1
	readonly key=$2
	;;
*)	usage >&2
	exit 2
	;;
esac

TOOLDIR=$(readlink -e $(dirname $0))

INPUTFILE=$(readlink -e -- "$input")  || {
	echo >&2 "error: Input \`$input' does not exist"
	exit 1
}
KEYFILE=$(readlink -e -- "$key")  || {
	echo >&2 "error: Private key \`$key' does not exist"
	exit 3
}

system=$(uname -s)
case $system in
*CYGWIN*) 
	readonly sign_app_bin=sign_app.exe
	;;
*Linux*)  
	readonly sign_app_bin=sign_app
   	;;
*MINGW*)
	readonly sign_app_bin=sign_app.exe
	;;
*)	echo >&2 "Unknown system \`$system'!"
	exit 1
	;;
esac


$TOOLDIR/../../../bin/$sign_app_bin \
ca=$INPUTFILE \
sca=$INPUTFILE.sbin \
algo=ecdsa \
ecdsa_file=$KEYFILE \
verbose=$verbose \
signonly=$signonly \
load_address=0x10000000


if [ $? -ne 0 ] ; then
echo "ERROR."
exit 1
fi

echo "Generated: $input.sbin"
echo "SUCCESS."
