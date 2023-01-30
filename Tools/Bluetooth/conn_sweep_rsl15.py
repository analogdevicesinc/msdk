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
from termcolor import colored

verbose=True

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
Connection sweep.

This tool uses a Mini Circuits RCDAT to control attenuation between two devices
running DTM software. A connection is created and PER data is gathered based on a 
combination of parameters.
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
parser.add_argument('slaveSerial',help='Serial port for slave device')
parser.add_argument('masterSerial',help='Serial port for master device')
parser.add_argument('results',help='CSV files to store the results')
parser.add_argument('-d', '--delay', default=5,help='Number of seconds to wait before ending the test')
parser.add_argument('-l', '--limit', default=0,help='PER limit for return value')
parser.add_argument('-p', '--phys', default="1",help='PHYs to test with, comma separated list with 1-4.')
parser.add_argument('-t', '--txpows', default="0",help='TX powers to test with, comma separated list.')
parser.add_argument('-a', '--attens', help='Attenuation settings to use, comma separated list.')
parser.add_argument('-da', '--disable-atten', action='store_true',help='Disbale Attenuator For Testing Purposes')

args = parser.parse_args()
print(args)

packetLengths    = [250]
phys             = args.phys.strip().split(",")
txPowers         = args.txpows.strip().split(",")

if(args.attens == None):
    attens = list(range(20,90,10))

    # Add the max attenuation 
    attens.append(90)
else:
    attens = args.attens.strip().split(",")

if args.disable_atten is not None:
    disableAttenuator = True
else:
    disableAttenuator = False

print("slaveSerial   :",args.slaveSerial)
print("masterSerial  :",args.masterSerial)
print("results       :",args.results)
print("delay         :",args.delay)
print("packetLengths :",packetLengths)
print("phys          :",phys)
print("attens        :",attens)
print("txPowers      :",txPowers)
print("PER limit     :",args.limit)

# Open the results file, write the parameters
results = open(args.results, "a")
results.write("# slaveSerial   : "+str(args.slaveSerial)+"\n")
results.write("# masterSerial  : "+str(args.masterSerial)+"\n")
results.write("# results       : "+str(args.results)+"\n")
results.write("# delay         : "+str(args.delay)+"\n")
results.write("# packetLengths : "+str(packetLengths)+"\n")
results.write("# phys          : "+str(phys)+"\n")
results.write("# attens        : "+str(attens)+"\n")
results.write("# PER limit     : "+str(args.limit)+"\n")

# Write the header line
results.write("packetLen,phy,atten,txPower,perMaster,perSlave\n")

# Create the BLE_hci objects
hciSlave = BLE_hci(Namespace(serialPort=args.slaveSerial, monPort="", baud=115200))
hciMaster = BLE_hci(Namespace(serialPort=args.masterSerial, monPort="", baud=115200))

perMax = 0

for packetLen,phy,txPower in itertools.product(packetLengths,phys,txPowers):

    # Reset the devices
    hciSlave.resetFunc(None)
    hciMaster.resetFunc(None)
    sleep(0.1)

    # # Reset the attenuation
    if not disableAttenuator:
        mini_RCDAT = mini_RCDAT_USB(Namespace(atten=30))
        sleep(0.1)


    printInfo('Setting Data Length')
    hciSlave.dataLenFunc(None)
    hciMaster.dataLenFunc(None)


    

    printInfo('Setting PHY')
    # Set the PHY
    hciMaster.phyFunc(Namespace(phy=str(phy)))



    # Set the TX Power
    printInfo('Setting TX Power')
    hciSlave.txPowerFunc(Namespace(power=txPower, handle="0"))
    hciMaster.txPowerFunc(Namespace(power=txPower, handle="0"))


    for atten in attens:
        print(packetLen," ",phy," ",atten," ",txPower)

        # Set the attenuation
        if not disableAttenuator:
            mini_RCDAT = mini_RCDAT_USB(Namespace(atten=atten))
            sleep(0.1)
        


        #start the test
        hciSlave.rxTestFunc(Namespace(channel=0, phy=1))
        hciMaster.txTestFunc(Namespace(channel=0, phy=1, payload=0,packetLength=0))
        
        sleep(int(args.delay))
        
        packtesReceived = hciSlave.endTestFunc(Namespace(noPrint=True))
        stats = hciMaster.endTestVSFunc(Namespace(noPrint=True))
        
        packetsTransmitted = 0

        if stats is not None:
            packetsTransmitted = stats['txData']    
        
        if packetsTransmitted != 0:
            perSlave = round(100 * (1 - packtesReceived / packetsTransmitted), 2)
            
        else:
            printWarning('Connection stats returned invalid data. (Packets Transmitted = 0) PER rate being set to 100')
            perSlave = 100
        

        #PER cannot be defined for the master with this scheme currently so setting to zero
        perMaster = 0    
    
        if(perSlave > perMax):
            perMax = perSlave
            

        # Save the results to file
        results.write(str(packetLen)+","+str(phy)+",-"+str(atten)+","+str(txPower)+","+str(perMaster)+","+str(perSlave)+"\n")

# Reset the devices
hciSlave.resetFunc(None)
hciMaster.resetFunc(None)
sleep(0.1)

results.write("\n")
results.close()

print("perMax: ",perMax)

if(float(args.limit) != 0.0):
    if(perMax > float(args.limit)):
        print("PER too high!")
        sys.exit(1)

sys.exit(0)
