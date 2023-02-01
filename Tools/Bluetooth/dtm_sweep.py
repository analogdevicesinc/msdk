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

if socket.gethostname() == "wall-e":
    rf_switch = True
else:
    rf_switch = False

# Setup the command line description text
descText = """
DTM sweep.

This tool uses a Mini Circuits RCDAT to control attenuation between two devices
running DTM software. The Packet error rate (PER) of the master will be collected by setting the slave device in tx test mode and the master in rx test mode.
A vendor specific command is sent which sets the total number of packets which shoule be transmitted. 
The total number of packets transmitted will be compared to the number of packtes received and the PER will be  
calculated as numPacketsReceived/numPacketsTransmitted * 100

IMPORTANT:
The tx test command is vendor specific and is only guarenteed to work on MAX32 BLE devices running the latest stack. 
The command is also supported by Nordic SoCs
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
parser.add_argument('slaveSerial',help='Serial port for slave device')
parser.add_argument('masterSerial',help='Serial port for master device')
parser.add_argument('results',help='CSV files to store the results')
parser.add_argument('-d', '--delay', default=5,help='Number of seconds to wait before ending the test')
parser.add_argument('-l', '--limit', default=0,help='PER limit for return value')
parser.add_argument('-p', '--phys', default="1",help='PHYs to test with, comma separated list with 1-4.')
parser.add_argument('-c', '--channel', default="0", help="Test channel, 0-39")
parser.add_argument('-t', '--txpows', default="0",help='TX powers to test with, comma separated list.')
parser.add_argument('-a', '--attens', help='Attenuation settings to use, comma separated list.')
parser.add_argument('-s', '--step', default=10, help='Attenuation sweep step size in dBm.')
parser.add_argument('-e', '--pktlen', default="250", help="packet length, comma separated list.")

parser.add_argument('-n', '--numpkt', default='5000',help='Number of packets in test.')

parser.add_argument('--mtp', default="", help="master TRACE serial port")
parser.add_argument('--stp', default="", help="slave TRACE serial port")
 
args = parser.parse_args()
print(args)

print("--------------------------------------------------------------------------------------------")
packetLengths    = args.pktlen.strip().split(",")
numPackets       = args.numpkt.strip().split(",")
phys             = args.phys.strip().split(",")
txPowers         = args.txpows.strip().split(",")
chan             = args.channel.strip().split(",")

if args.attens is None:
    if int(args.step) == 0:
        attens = [20, 70]
    else:
        attens = list(range(20, 90, int(args.step)))

    # Add the max attenuation
    attens.append(90)
else:
    attens = args.attens.strip().split(",")

print("slaveSerial   :", args.slaveSerial)
print("masterSerial  :", args.masterSerial)
print("results       :", args.results)
print("delay         :", args.delay)
print("packetLengths :", packetLengths)
print("numPackets    :", numPackets)
print("phys          :", phys)
print("attens        :", attens)
print("txPowers      :", txPowers)
print("Channel       :", chan)
print("PER limit     :", args.limit)

# Open the results file, write the parameters
results = open(args.results, "a")
if 0:
    results.write("# slaveSerial   : "+str(args.slaveSerial)+"\n")
    results.write("# masterSerial  : "+str(args.masterSerial)+"\n")
    results.write("# results       : "+str(args.results)+"\n")
    results.write("# delay         : "+str(args.delay)+"\n")
    results.write("# packetLengths : "+str(packetLengths)+"\n")
    results.write("# numPackets    : "+str(numPackets)+"\n")
    results.write("# phys          : "+str(phys)+"\n")
    results.write("# attens        : "+str(attens)+"\n")
    results.write("# txPower       : "+str(txPower)+"\n")
    results.write("# Channel       : "+str(chan)+"\n")
    results.write("# PER limit     : "+str(args.limit)+"\n")

    # Write the header line
    results.write("packetLen,numPkt,phy,atten,txPower,channel,perMaster,perSlave\n")

# Create the BLE_hci objects
hciSlave  = BLE_hci(Namespace(serialPort=args.slaveSerial,  monPort=args.stp, baud=115200, id=2))
hciMaster = BLE_hci(Namespace(serialPort=args.masterSerial, monPort=args.mtp, baud=115200, id=1))

perMax = 0

for packetLen, numPkt, phy, txPower, chan in itertools.product(packetLengths, numPackets, phys, txPowers, chan):
    per_100 = 0
    for atten in attens:
        RETRY = 2
        while per_100 < RETRY:
            start_secs = time.time()
            print(f'\n---------------------------------------------------------------------------------------')
            print(f'packetLen: {packetLen}, numPackets: {numPkt}, phy: {phy}, atten: {atten}, txPower: {txPower}, Channel: {chan}\n')

            print("Set the requested attenuation.")
            if rf_switch:
                mini_RCDAT = mini_RCDAT_USB(Namespace(atten=atten))
            sleep(0.1)

            print("\nReset the devices.")
            hciSlave.resetFunc(None)
            hciMaster.resetFunc(None)
            sleep(0.1)

            print("\nSet the PHY.")
            hciMaster.phyFunc(Namespace(phy=phy), timeout=1)

            print("\nSet the txPower.")
            hciSlave.txPowerFunc(Namespace(power=txPower, handle="0")) 
            hciMaster.txPowerFunc(Namespace(power=txPower, handle="0"))


            print('--------------')
            print("\nSet slave to RX.")
            print(chan)
            hciSlave.rxTestFunc(Namespace(channel=chan, phy=phy))
            print("\nSet master to TX, start test.")
            hciMaster.txTestVSFunc(Namespace(channel=chan, phy=phy, packetLength=packetLen, numPackets=numPkt,payload=0))            
            print(f"\nWait {args.delay} secs for the DTM Test to complete.")
            sleep(int(args.delay))

            print("\nEnd test.")
            hciMaster.endTestFunc(None)
            perSlave = hciSlave.endTestFunc(None) / int(numPkt) * 100

            print('--------------')
            print("\nReset the devices.")
            hciSlave.resetFunc(None)
            hciMaster.resetFunc(None)
            sleep(0.1)
            print(chan)
            print("\nSet master to RX.")
            hciMaster.rxTestFunc(Namespace(channel=chan, phy=phy))
            print("\nSet slave to TX, start test.")
            hciSlave.txTestVSFunc(Namespace(channel=chan, phy=phy, packetLength=packetLen, numPackets=numPkt,payload=0))            

            print(f"\nWait {args.delay} secs for the DTM Test to complete.")
            sleep(int(args.delay))

            print("\nEnd test.")
            hciSlave.endTestFunc(None)
            perMaster = hciMaster.endTestFunc(None) / int(numPkt) * 100

            print("\nCollect results.")
            print("perMaster  : ", perMaster)
            print("perSlave   : ", perSlave)

            if perMaster is None or perSlave is None:
                per_100 += 1
                print(f'Retry: {per_100}')
                continue

            # Record max per
            if perMaster > perMax:
                perMax = perMaster
            if perSlave > perMax:
                perMax = perSlave
            print("perMax     : ", perMax)

            break

        if per_100 >= RETRY:
            print(f'Tried {per_100} times, give up.')
            perMaster = 100
            perSlave = 100
            perMax = 100

        # Save the results to file
        results.write(str(packetLen)+","+str(numPkt)+","+str(phy)+",-"+str(atten)+","+str(txPower)+","+str(chan)+","+str(perMaster)+","+str(perSlave)+"\n")
        end_secs = time.time()
        print(f'\nUsed {(end_secs - start_secs):.0f} seconds.')

print('--------------------------------------------------------------------------------------------')
print("Reset the devices.")
hciSlave.resetFunc(None)
hciMaster.resetFunc(None)
sleep(0.1)

results.write("\n")
results.close()

print("perMax: ", perMax)

if float(args.limit) != 0.0:
    if perMax > float(args.limit):
        print("PER too high!")
        sys.exit(1)

sys.exit(0)
