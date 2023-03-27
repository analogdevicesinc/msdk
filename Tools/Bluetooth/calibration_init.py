
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

# calibration_init.py
#
# tool to read/verify DBB for calibration purposes
#
# Ensure that both targets are built with BT_VER := 9
#
from pyocd.core.helpers import ConnectHelper
import pyocd.core.options as opts
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

from DBB import DBB
import sys
from BLE_hci import BLE_hci
from BLE_hci import Namespace

TRACE_INFO = 2
TRACE_WARNING = 1
TRACE_ERROR = 0

traceLevel = TRACE_INFO

logging.basicConfig(level=logging.INFO)

# Setup the command line description text
descText = """
Run Calibration and Initialization Tests
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(
    description=descText, formatter_class=RawTextHelpFormatter)

parser.add_argument('dap_id', help='CMSIS DAP Serial Number')
parser.add_argument('hci_id', help='HCI Serial Port')

parser.add_argument(
    '-b', '--bin', help='Binary To Program Board with', default='')
parser.add_argument('-c', '--chip', default='', help='Part number of chip')
parser.add_argument('-urd', '--update-reference-dbb', action='store_true')
parser.add_argument('-ura', '--update-reference-afe', action='store_true')
parser.add_argument('-vd', '--verify-dbb',  action='store_true')
parser.add_argument('-p', '--print',  default='',
                    help='print the structure <ctrl|tx|rx|rffe|all>=<offset-hex>')
parser.add_argument('-f', '--file',  default='dbb_reference.json')




args = parser.parse_args()
print(args)
print("--------------------------------------------------------------------------------------------")
dbbFile = args.file


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
        raise Exception('Lengths dont match')

    return [{f'offset {hex(i)}': {'ref': hex(a[i]), 'read': hex(b[i])}} for i in range(len(a)) if a[i] != b[i]]


def doPrint(dbbReadout, printArg):
    locationInfo = printArg.split('=')
    region = locationInfo[0].lower()

    if region == 'all':
        printInfo(dbbReadout)
        return

    if len(locationInfo) > 1:

        offset = locationInfo[1]
        if 'x' in offset:
            offset = int(offset, 16)
        else:
            offset = int(offset)

    else:
        offset = -1

    if region not in dbbReadout:
        msg = f'Region {region} not in dbb'
        raise Exception(msg)

    if offset >= 0:
        regionLen = len(dbbReadout[region])
        if offset > regionLen - 1:
            msg = f'Invalid offset {offset}, must be less than len of region {regionLen - 1}' 
            raise Exception(msg)
        
        regionReadout = dbbReadout[region][offset]
        printInfo(f'Region {region} offset {offset}: {regionReadout}')
    else:
        printInfo(f'Region {region}: {dbbReadout[region]}')


def verifyDbb(dbbReadout):
    dbbRef = {}
    if (os.path.exists(dbbFile)):
        with open(dbbFile, 'r') as read:
            dbbRef = json.load(read)

        anyMismatches = False
        failureFilePath = dbbFile.split('.')
        failureFilePath = f'{failureFilePath[0]}_failure.json'
        failureFile = open(failureFilePath, 'w')
        allMismatches = {}

        for region in dbbRef:
            mismatches = getMismatches(dbbRef[region], dbbReadout[region])
            if len(mismatches) != 0:
                printWarning(
                    f'Mismatches found at region {region} and offsets {mismatches}')

                allMismatches[region] = mismatches

        if anyMismatches:
            failureFile.close()
            os.remove(failureFilePath)
        else:
            json.dump(allMismatches, failureFile)
            failureFile.close()

        print('DBB Match', anyMismatches)
        return True
    else:
        print(f'{dbbFile} Does Not Exist!')
        return False

def hciSetup(hciId):
    hci = BLE_hci(Namespace(serialPort=hciId,  monPort='', baud=115200, id=0))
    hci.resetFunc(None)
    hci.txPowerFunc(Namespace(power=0, handle="0"))
    hci.txTestVSFunc(Namespace(channel=0, phy=1,
                     packetLength=0, numPackets=0, payload=3))




session = ConnectHelper.session_with_chosen_probe(blocking=False,unique_id=args.dap_id)


if args.chip != '':
    session.options['target_override'] = 'max32690'




with session:

    printInfo(session.options['target_override'])

    board = session.board
    
    target = board.target
    flash = target.memory_map.get_boot_memory()

    # Load firmware into device.

    if args.bin != '' and os.path.exists(args.bin):
        FileProgrammer(session).program(args.bin)

    # Reset, run.
    target.reset_and_halt()
    target.resume()

    sleep(2)

    # reset the hci
    hciSetup(args.hci_id)
    target.halt()

    time.sleep(1)
    # Read some registers.

    dbb = DBB(target,args.chip)

    
    dbbReadout = dbb.getAll()
    if args.print:
        doPrint(dbbReadout, args.print)

    if args.update_reference_dbb:
        with open(dbbFile, 'w') as write:
            json.dump(dbbReadout, write)

    if args.verify_dbb:
        verifyDbb(dbbReadout)
        

    target.reset()
    target.resume()

    sys.exit(0)
