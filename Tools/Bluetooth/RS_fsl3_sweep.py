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

## RS_fsl3_sweep.py
 #
 # Sweep through tests executed by RS_fsl3.py
 #

import sys
import argparse
from argparse import RawTextHelpFormatter
from pprint import pprint
from time import sleep
from BLE_hci import BLE_hci
from BLE_hci import Namespace
from RS_fsl3 import RS_fsl3

# Setup the command line description text
descText = """
RS_fsl3.py test sweep.
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
parser.add_argument('ipAddress',help='Instrument IP address')
parser.add_argument('serialPort',help='DUT HCI serial port')
parser.add_argument("--tx_pwr", default=4, help="TX POWE (0, 4)")

args = parser.parse_args()
pprint(args)

# Create the BLE_hci objects
hciDUT = BLE_hci(Namespace(serialPort=args.serialPort, monPort="", baud=115200))
hciDUT.resetFunc(None)

# Wait for calibration
sleep(1)

# Create the RS_FSL3 spectrum analyzer object
sa = RS_fsl3(Namespace(ipAddress=args.ipAddress))

# Setup the DUT to TX at max power
hciDUT.txPowerFunc(Namespace(power=int(args.tx_pwr), handle=None))

channels = [0,1,2,10,19,30,36,37,38,39]

for channel in channels:
    # Start transmitting
    hciDUT.txTestFunc(Namespace(channel=channel, packetLength=255, payload=0, phy=1))
    sleep(1)

    if(channel > 36):
        # Start the OB2 test
        retval = sa.testRB2()

        if(retval != True):
            print("RB2 test failed")
            sys.exit(1)
    else:
        # Start the OBW test
        retval = sa.testOBW(ch=channel)

        if(retval != True):
            print("OBW test failed")
            sys.exit(1)

    if(channel == 0):
        # Start the OB1 test
        retval = sa.testRB1()

        if(retval != True):
            print("RB1 test failed")
            sys.exit(1)

print("Test passed!")
sys.exit(0)
