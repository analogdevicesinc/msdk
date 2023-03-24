
#! /usr/bin/env python3

################################################################################
# Copyright (C) 2020 Maxim Integrated Products, Inc., All Rights Reserved.
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
#
###############################################################################

# dtm_sweep.py
#
# Sweep connection parameters.
#
# Ensure that both targets are built with BT_VER := 9
#
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
import logging
import time
import sys
import argparse
from argparse import RawTextHelpFormatter
from time import sleep
import os.path
import json
from termcolor import colored
from json import JSONEncoder
from DBB import DBB
import sys

TRACE_INFO = 2
TRACE_WARNING = 1
TRACE_ERROR = 0

traceLevel = TRACE_INFO

logging.basicConfig(level=logging.INFO)

def printTrace(label, msg, callerLevel, color='white'):
    if callerLevel <= traceLevel:
        print(colored(label + ": ", color), colored(msg, color))


def printWarning(msg):
    printTrace('Warning', msg, TRACE_WARNING, 'yellow')


def printInfo(msg):
    printTrace('Info', msg, TRACE_INFO, 'green')


def printError(msg):
    printTrace('Error', msg, TRACE_ERROR, 'red')

def getMismatches(a, b):
    if len(a) != len(b):
        print('Lengths dont match')
        return []
    return [i for i in range(len(a)) if a[i] != b[i]]


# Setup the command line description text
descText = """
Run Calibration and Initialization Tests
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(
    description=descText, formatter_class=RawTextHelpFormatter)

parser.add_argument('dap_id', help='CMSIS DAP Serial Number')

parser.add_argument('-b','--bin',help='Binary To Program Board with', default='')
parser.add_argument('-urd', '--update-reference-dbb', action='store_true')
parser.add_argument('-ura', '--update-reference-afe', action='store_true')
parser.add_argument('-vd', '--verify-dbb',  action='store_true')
parser.add_argument('-p', '--print',  action='store_true')
parser.add_argument('-f', '--file',  default='dbb_reference.json')


args = parser.parse_args()
print(args)
print("--------------------------------------------------------------------------------------------")
dbbFile = args.file

if not os.path.exists(args.bin):
    print("bin does not exist!")


with ConnectHelper.session_with_chosen_probe(unique_id='040917027f63482900000000000000000000000097969906') as session:

    board = session.board
    target = board.target
    flash = target.memory_map.get_boot_memory()

    # Load firmware into device.

    if args.bin != '' and os.path.exists(args.bin):
        FileProgrammer(session).program(args.bin)

    # Reset, run.
    target.reset_and_halt()
    target.resume()

    time.sleep(1)
    # Read some registers.
    target.halt()


    dbb = DBB(target)
    
    dbbReadout = dbb.getAll()

    
    if args.update_reference_dbb:
        with open(dbbFile, 'w') as write:
                json.dump(dbbReadout, write)

    if args.print:
        print(colored(dbbReadout, 'green'))
    if args.verify_dbb:
        dbbRef = {}
        if (os.path.exists(dbbFile)):
            with open(dbbFile, 'r') as read:
                dbbRef = json.load(read)
            anyMismatches = False
            for region in dbbRef:
                mismatches = getMismatches(dbbRef[region], dbbReadout[region])
                if len(mismatches) != 0:
                    print(f'Mismatches found at region{region} and offsets {mismatches}')
            print('DBB Match', anyMismatches)
        else:
            print(f'{dbbFile} Does Not Exist!')
    
    sys.exit(0)
