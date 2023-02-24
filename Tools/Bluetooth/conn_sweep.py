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

## conn_sweep.py
 #
 # Sweep connection parameters.
 #
 # Ensure that both targets are built with BT_VER := 9
 #


from datetime import datetime as dt
import sys
import argparse
from argparse import RawTextHelpFormatter
from time import sleep
import itertools
from mini_RCDAT_USB import mini_RCDAT_USB
from BLE_hci import BLE_hci
from BLE_hci import Namespace
import os
from pprint import pprint
import socket
from subprocess import call, Popen, PIPE, CalledProcessError, STDOUT
import time

total_retry_times = 0

if socket.gethostname() == "wall-e":
    rf_switch = True
else:
    rf_switch = False


def run_script_reset_board(sh_file):
    """call a prepared script file to reset a board"""
    sh_file = os.path.realpath(sh_file)
    print(f"Run script file {sh_file}.")
    p = Popen([f'{sh_file}'], stdout=PIPE, stderr=PIPE, shell=True)

    for line in iter(p.stdout.readline, b''):
        print(f'{dt.now()} - {line.strip().decode("utf-8")}')
    
    p.stdout.close()
    p.wait()
    result = p.returncode
    print(f'Exit: {result}')
    return result


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
parser.add_argument('-s', '--step', default=10, help='Attenuation sweep step size in dBm.')
parser.add_argument('-e', '--pktlen', default="250", help="packet length, comma separated list.")
parser.add_argument('--mtp', default="", help="master TRACE serial port")
parser.add_argument('--stp', default="", help="slave TRACE serial port")
parser.add_argument('--loss', default=0, help="Calibrated path loss, -15.7 dBm (-16.4+0.7)")
parser.add_argument('--brd1_reset', default="", help="script file to reset board1")
parser.add_argument('--brd2_reset', default="", help="script file to reset board2")
parser.add_argument('--retry_limit', default=3, help="limit of retry times after fail")
parser.add_argument('--short', action='store_true', help="shorter test")
 
args = parser.parse_args()

print("--------------------------------------------------------------------------------------------")
pprint(vars(args))

packetLengths    = args.pktlen.strip().split(",")
phys             = args.phys.strip().split(",")
txPowers         = args.txpows.strip().split(",")

if args.attens is None:
    if int(args.step) == 0 or int(args.step) == -1:
        attens = [20, 70]
    else:
        attens = list(range(20, 90, int(args.step)))

    # Add the max attenuation
    if int(args.step) != -1:
        attens.append(90)
else:
    temp = args.attens.replace(" ", "")
    attens = temp.split(",")
    attens = [float(x) for x in attens]

print("slaveSerial   :", args.slaveSerial)
print("masterSerial  :", args.masterSerial)
print("slave TRACE   :", args.stp)
print("master TRACE  :", args.mtp)
print("results       :", args.results)
print("delay         :", args.delay)
print("packetLengths :", packetLengths)
print("phys          :", phys)
print("attens        :", attens)
print("txPowers      :", txPowers)
print("PER limit     :", args.limit)

# Open the results file, write the parameters
results = open(args.results, "a")
if 0:
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
hciSlave  = BLE_hci(Namespace(serialPort=args.slaveSerial,  monPort=args.stp, baud=115200, id=2))
hciMaster = BLE_hci(Namespace(serialPort=args.masterSerial, monPort=args.mtp, baud=115200, id=1))

ABORTED = False
perMax = 0
RETRY = int(args.retry_limit)
need_to_setup = True  # only do it at the beginning or after flash

for packetLen, phy, txPower in itertools.product(packetLengths, phys, txPowers):
    if args.short:
        for atten in attens:
            per_100 = 0
            while per_100 < RETRY:
                if need_to_setup:
                    need_to_setup = False

                    start_secs = time.time()

                    print("\nReset the devices at the beginning of the test or after flash the board again.")
                    hciSlave.resetFunc(None)
                    hciMaster.resetFunc(None)
                    sleep(10)

                    print("\nSet addresses.")
                    txAddr = "00:12:34:88:77:33"
                    rxAddr = "11:12:34:88:77:33"
                    hciSlave.addrFunc(Namespace(addr=txAddr))
                    hciMaster.addrFunc(Namespace(addr=rxAddr))
                    sleep(1)

                    print("\n----------------------------------")                
                    print("pre-test setup")
                    print("----------------------------------")
            
                    print("\nReset the attenuation to 30.")
                    if rf_switch:
                        set_val = 30 + float(args.loss)
                        mini_RCDAT = mini_RCDAT_USB(Namespace(atten=set_val))
                    sleep(0.1)

                    print("\nStart advertising.")
                    hciSlave.advFunc(Namespace(interval="60", stats="False", connect="True", maintain=False, listen="False"))

                    print("\nStart connection.")
                    hciMaster.initFunc(Namespace(interval="6", timeout="64", addr=txAddr, stats="False", maintain=False, listen="False"))

                    print("\nSlave and master listenFunc")
                    hciSlave.listenFunc(Namespace(time=1, stats="False"))
                    hciMaster.listenFunc(Namespace(time=1, stats="False"))

                    print("\nSlave and master dataLenFunc")
                    hciSlave.dataLenFunc(None)
                    hciMaster.dataLenFunc(None)

                    print("\nSlave listenFunc")
                    hciSlave.listenFunc(Namespace(time=1, stats="False"))

                    print("\nMaster set PHY and listenFunc.")
                    hciMaster.phyFunc(Namespace(phy=str(phy)), timeout=1)
                    hciMaster.listenFunc(Namespace(time=2, stats="False"))

                    print("\nSlave and master set the txPower.")
                    hciSlave.txPowerFunc(Namespace(power=txPower, handle="0")) 
                    hciMaster.txPowerFunc(Namespace(power=txPower, handle="0"))

                    print("\nSlave listenFunc")
                    hciSlave.listenFunc(Namespace(time=1, stats="False"))

                    print("\nSlave and master sinkAclFunc")
                    hciSlave.sinkAclFunc(None)
                    hciMaster.sinkAclFunc(None)

                    print("\nslave listenFunc, 1 sec")
                    hciSlave.listenFunc(Namespace(time=1, stats="False"))

                    print("\nSlave and master sendAclFunc, slave listenFunc")
                    hciSlave.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(0)))
                    hciMaster.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(0)))
                    hciSlave.listenFunc(Namespace(time=1, stats="False"))

                    print("\nSlave and master sendAclFunc, slave listenFunc")
                    hciSlave.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(1)))
                    hciMaster.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(1)))
                    hciSlave.listenFunc(Namespace(time=1, stats="False"))
                
                start_secs = time.time()

                print('\n---------------------------')
                print(f'packetLen: {packetLen}, phy: {phy}, atten: {atten}, txPower: {txPower}')
                print('---------------------------')

                print(f"\nSet the requested attenuation: {atten}.")
                if rf_switch:
                    set_val = atten + float(args.loss)
                    mini_RCDAT = mini_RCDAT_USB(Namespace(atten=set_val))
                
                print("\nSleep 1 second")
                sleep(1)

                print("\nReset the packet stats.")
                hciSlave.cmdFunc(Namespace(cmd="0102FF00"), timeout=10.0)
                hciMaster.cmdFunc(Namespace(cmd="0102FF00"), timeout=10.0)

                print("\nSlave listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))
                print(f'used {(time.time() - start_secs):.0f} secs.')

                print("\nMaster listenFunc")
                hciMaster.listenFunc(Namespace(time=1, stats="False"))

                print(f"\nsleep args.delay {args.delay} secs")
                sleep(int(args.delay))

                print("\nRead any pending events. slave and master listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))
                hciMaster.listenFunc(Namespace(time=1, stats="False"))

                print("\nMaster collects results.")
                perMaster = hciMaster.connStatsFunc(None)

                print("\nSlave collects results.")
                perSlave = hciSlave.connStatsFunc(None)

                print("perMaster  : ", perMaster)
                print("perSlave   : ", perSlave)

                reset_master = False
                if perMaster is None:
                    print("perMaster is None. Reset the master.")
                    reset_master = True
                elif perMaster >= 99.99:
                    print("perMaster invalid. Reset the master.")
                    reset_master = True
                
                reset_slave = False
                if perSlave is None:
                    print("perSlave is None. Flash the slave.")
                    reset_slave = True
                elif perSlave >= 99.99:
                    print("perSlave invalid. Flash the slave.")
                    reset_slave = True
                
                if reset_slave or reset_master:
                    run_script_reset_board(args.brd1_reset)
                    run_script_reset_board(args.brd2_reset)

                    per_100 += 1
                    total_retry_times += 1
                    sleep(10)

                    need_to_setup = True

                    continue
                
                # Record max per
                if perMaster > perMax:
                    perMax = perMaster
                if perSlave > perMax:
                    perMax = perSlave
                print("perMax     : ", perMax)

                break  # no retry

            if per_100 >= RETRY:
                print(f'Tried {per_100} times, give up.')
                perMaster = 100
                perSlave = 100
                perMax = 100
                
                ABORTED = True
                break

            # Save the results to file
            results.write(str(packetLen)+","+str(phy)+",-"+str(atten)+","+str(txPower)+","+str(perMaster)+","+str(perSlave)+"\n")
            end_secs = time.time()
            print(f'\nTotally used {(end_secs - start_secs):.0f} seconds for this point.')

    else:  # original method
        for atten in attens:
            per_100 = 0
            RETRY = int(args.retry_limit)
            while per_100 < RETRY:
                start_secs = time.time()
                print(f'\n---------------------------------------------------------------------------------------')
                print(f'packetLen: {packetLen}, phy: {phy}, atten: {atten}, txPower: {txPower}\n')

                print("\nReset the devices.")
                hciSlave.resetFunc(None)
                hciMaster.resetFunc(None)
                sleep(0.1)

                print("\nReset the attenuation to 30.")
                if rf_switch:
                    set_val = 30 + float(args.loss)
                    mini_RCDAT = mini_RCDAT_USB(Namespace(atten=set_val))
                sleep(0.1)

                print("\nSet addresses.")
                txAddr = "00:12:34:88:77:33"
                rxAddr = "11:12:34:88:77:33"
                hciSlave.addrFunc(Namespace(addr=txAddr))
                hciMaster.addrFunc(Namespace(addr=rxAddr))

                print("\nStart advertising.")
                hciSlave.advFunc(Namespace(interval="60", stats="False", connect="True", maintain=False, listen="False"))

                print("\nStart connection.")
                hciMaster.initFunc(Namespace(interval="6", timeout="64", addr=txAddr, stats="False", maintain=False, listen="False"))
                
                print("\nSlave and master listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))
                hciMaster.listenFunc(Namespace(time=1, stats="False"))

                print("\nSlave and master dataLenFunc")
                hciSlave.dataLenFunc(None)
                hciMaster.dataLenFunc(None)

                print("\nSlave listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))

                print("\nMaster set PHY and listenFunc.")
                hciMaster.phyFunc(Namespace(phy=str(phy)), timeout=1)
                hciMaster.listenFunc(Namespace(time=2, stats="False"))

                print("\nSlave and master set the txPower.")
                hciSlave.txPowerFunc(Namespace(power=txPower, handle="0")) 
                hciMaster.txPowerFunc(Namespace(power=txPower, handle="0"))

                print("\nSlave listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))

                print("\nSlave and master sinkAclFunc")
                hciSlave.sinkAclFunc(None)
                hciMaster.sinkAclFunc(None)
                print("\nslave listenFunc, 1 sec")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))

                print("\nSlave and master sendAclFunc, slave listenFunc")
                hciSlave.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(0)))
                hciMaster.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(0)))
                hciSlave.listenFunc(Namespace(time=1, stats="False"))

                print("\nSlave and master sendAclFunc, slave listenFunc")
                hciSlave.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(1)))
                hciMaster.sendAclFunc(Namespace(packetLen=str(packetLen), numPackets=str(1)))
                hciSlave.listenFunc(Namespace(time=1, stats="False"))

                print('--------------')
                print(f'packetLen: {packetLen}, phy: {phy}, atten: {atten}, txPower: {txPower}\n')
        
                print(f"Set the requested attenuation: {atten}.")
                if rf_switch:
                    set_val = atten + float(args.loss)
                    mini_RCDAT = mini_RCDAT_USB(Namespace(atten=set_val))
                sleep(0.1)

                print("\nReset the packet stats.")
                hciSlave.cmdFunc(Namespace(cmd="0102FF00"), timeout=10.0)
                hciMaster.cmdFunc(Namespace(cmd="0102FF00"), timeout=10.0)

                print("\nSlave listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))

                print("\nMaster listenFunc")
                hciMaster.listenFunc(Namespace(time=1, stats="False"))

                print(f"\nWait {args.delay} secs for the TX to complete.")
                sleep(int(args.delay))

                print("\nRead any pending events. slave and master listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))
                hciMaster.listenFunc(Namespace(time=1, stats="False"))

                print("\nMaster collects results.")
                perMaster = hciMaster.connStatsFunc(None)
                print("\nSlave collects results.")
                perSlave = hciSlave.connStatsFunc(None)

                print("perMaster  : ", perMaster)
                print("perSlave   : ", perSlave)

                reset_master = False
                if perMaster is None:
                    print("perMaster is None. Reset the master.")
                    reset_master = True
                elif perMaster >= 99.99:
                    print("perMaster invalid. Reset the master.")
                    reset_master = True
                
                reset_slave = False
                if perSlave is None:
                    print("perSlave is None. Reset the slave.")
                    reset_slave = True
                elif perSlave >= 99.99:
                    print("perSlave invalid. Reset the slave.")
                    reset_slave = True
                
                if reset_slave or reset_master:
                    run_script_reset_board(args.brd1_reset)
                    run_script_reset_board(args.brd2_reset)

                    per_100 += 1
                    total_retry_times += 1
                    sleep(10)
                    continue

                # Record max per
                if perMaster > perMax:
                    perMax = perMaster
                if perSlave > perMax:
                    perMax = perSlave
                print("perMax     : ", perMax)

                break # no retry

            if per_100 >= RETRY:
                print(f'Tried {per_100} times, give up.')
                perMaster = 100
                perSlave = 100
                perMax = 100
                ABORTED = True

                break # no need to test other atten points

            # Save the results to file
            results.write(str(packetLen)+","+str(phy)+",-"+str(atten)+","+str(txPower)+","+str(perMaster)+","+str(perSlave)+"\n")
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

print(f'total_retry_times: {total_retry_times}')

if float(args.limit) != 0.0:
    if perMax > float(args.limit):
        print("PER too high!")
        sys.exit(1)

if ABORTED:
    sys.exit(2)

sys.exit(0)
