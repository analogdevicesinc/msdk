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

## dtm_sweep.py
 #
 # Sweep connection parameters.
 #
 # Ensure that both targets are built with BT_VER := 9
 #

import sys
import argparse
from argparse import RawTextHelpFormatter
from time import sleep
import itertools
from mini_RCDAT_USB import mini_RCDAT_USB
from BLE_hci import BLE_hci
from BLE_hci import Namespace
import socket
import time
import mxc_radio

from json import JSONEncoder

if socket.gethostname() == "wall-e":
    rf_switch = True
else:
    rf_switch = False
TRACE_INFO = 2
TRACE_WARNING =  1
TRACE_ERROR = 0

traceLevel = TRACE_INFO

def printTrace(label, msg,callerLevel, color='white'):
    if  callerLevel <= traceLevel:
        print(colored(label + ": ", color), colored(msg, color))

def printWarning(msg):
    printTrace('Warning', msg, TRACE_WARNING, 'yellow')

def printInfo(msg):
    printTrace('Info', msg, TRACE_INFO, 'green')

def printError(msg):
    printTrace('Error', msg, TRACE_ERROR, 'red')

# Setup the command line description text
descText = """
Run Calibration and Initialization Tests
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
parser.add_argument('serialPort',help='Serial port for slave device')
parser.add_argument('results',help='CSV files to store the results')
parser.add_argument('-d', '--dbb', default=5,help='Read and dump DBB registers')

args = parser.parse_args()
print(args)

print("--------------------------------------------------------------------------------------------")

print("Serial Port   :", args.serialPort)
print("results       :", args.results)


# Open the results file, write the parameters
results = open(args.results, "a")





class RegisterEncoder(JSONEncoder):
    def default(self, o):
            return o.__dict__
class DbbCtrlRegs:
    pass
class DbbRxRegs:
    pass
class DbbTxRegs:
    pass
class DbbRffeRegs:
    pass
class DBB:
    def __init__(self,ctrlReg=None, rxReg=None, txReg=None, rffeReg=None):
        self.ctrlReg = ctrlReg
        self.rxReg = rxReg
        self.txReg = txReg
        self.rffeReg = rffeReg


def listEq(list1,list2):

    if len(list1) != len(list2):
        return False
    length = len(list1)
    
    for i in range(length):
        if list1[i] != list2[i]:
            return False

    return True

def readDbbCtrl(hciInterface):
    """
    Read and return the DBB Ctrl Reg
    """

    reg = MXC_BASE_BTLE + 0x1000


    MXC_BASE_BTLE_DBB_CTRL =  "0x%08X" % (reg)
    
    
    #anything bigger than 128 bytes seems to cause a problem with the fromhex function
    ctrlRegSize = "0x%02X" %(32 * 4)
    regBase = reg
    ctrlReg = []
    
    #size of the ctrl reg is 304 bytes so just go  through more than that
    while reg <  regBase + 128 * 3:

        evtBytes = hciInterface.readRegFunc(Namespace(addr=MXC_BASE_BTLE_DBB_CTRL,length=ctrlRegSize))
        ctrlReg.extend(evtBytes)
        reg += 128
    
    print(ctrlReg)
    print('length', len(ctrlReg))
    
    

    
def readDbbRx(hciInterface):
    MXC_BASE_BTLE_DBB_RX = MXC_BASE_BTLE + 0x3000
    
def readDbbTx(hciInterface):
    MXC_BASE_BTLE_DBB_TX = MXC_BASE_BTLE + 0x2000
def readDbbRffe(hciInterface):
    MXC_BASE_BTLE_DBB_EXT_RFFE  = MXC_BASE_BTLE + 0x8000
    

def readDBB(hciInterface):
    """
    Function to read DBB register of device 
    Return DBB registers as a DBB class
    """
    dbb = mxcDBB()

    dbb.ctrlReg = readDbbCtrl(hciInterface)
    dbb.txReg = readDbbRx(hciInterface)
    dbb.rxReg = readDbbTx(hciInterface)
    dbb.rffeReg = readDbbRffe(hciInterface)

    return dbb

def verifyDBB(dbb):
    
    #get expected dbb values
    
    
    #make sure the all values match
    dbbMatches = True


    return dbbMatches
    

def main():
    # Create the BLE_hci objects
    hciInterface  = BLE_hci(Namespace(serialPort=args.serialPort,  monPort="", baud=115200, id=1))
    # hciInterface.resetFunc(None)
    # hciInterface.txTestFunc(Namespace(channel=0, phy=1, packetLength=0, payload=3))

    dbb = mxc_radio.DBB(hciInterface=hciInterface)
    ctrlReg = dbb.readCtrlReg()

    # print(ctrlReg)
    

    

    
    sys.exit(0)

if __name__ == "__main__":
    main()
