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
import json
from mini_RCDAT_USB import mini_RCDAT_USB
from BLE_hci import BLE_hci
from BLE_hci import Namespace
import os
from pprint import pprint
import socket
from subprocess import call, Popen, PIPE, CalledProcessError, STDOUT
import time

total_retry_times = 0
RESET_CNT = 1

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
parser.add_argument('--chip', default="", help="DUT chip")
parser.add_argument('--min_pwrs', default="90,90,90,90", help="abs min power")
 
args = parser.parse_args()

print("--------------------------------------------------------------------------------------------")
pprint(vars(args))

# PER mask
CONFIG_FILE=os.path.expanduser("~/Workspace/ci_config/RF-PHY-closed.json")
obj = json.load(open(CONFIG_FILE))
use_per_mask = obj['tests']["simple_per.yml"]["use_per_mask"]
per_mask_margin = int(obj['tests']['per_mask']['per_mask_margin'])
per_corr_dtm_to_cm = int(obj['tests']['per_mask']['per_corr_dtm_to_cm'])

phy_str = [
    "",
    "1M",
    "2M",
    "S8",
    "S2"
]

per_mask = {
    "1M": [
        [-20,                       per_mask_margin],
        [-90,                       per_mask_margin],
        [-93+per_corr_dtm_to_cm,    5+per_mask_margin],
        [-96+per_corr_dtm_to_cm,    30+per_mask_margin],
        [-99+per_corr_dtm_to_cm,    100],
        [-114,                      100] 
    ],

    "2M": [
        [-20,                       per_mask_margin],
        [-87,                       per_mask_margin],
        [-90+per_corr_dtm_to_cm,    5+per_mask_margin],
        [-93+per_corr_dtm_to_cm,    30+per_mask_margin],
        [-96+per_corr_dtm_to_cm,    100],
        [-111,                      100] 
    ],

    "S2": [
        [-20,                       per_mask_margin],
        [-95,                       per_mask_margin],
        [-98+per_corr_dtm_to_cm,    5+per_mask_margin],
        [-101+per_corr_dtm_to_cm,   30+per_mask_margin],
        [-104+per_corr_dtm_to_cm,   100],
        [-119,                      100]
    ],

    "S8": [
        [-20,                       per_mask_margin],
        [-98,                       per_mask_margin],
        [-101+per_corr_dtm_to_cm,   5+per_mask_margin],
        [-104+per_corr_dtm_to_cm,   30+per_mask_margin],
        [-107+per_corr_dtm_to_cm,   100],
        [-122,                      100]
    ]
}

print(f'      use_per_mask: {use_per_mask}')
print(f'   per_mask_margin: {per_mask_margin}')
print(f'per_corr_dtm_to_cm: {per_corr_dtm_to_cm}')
print('per_mask:')
pprint(per_mask)

# default minimum power
min_pwrs = [90, 90, 90, 90]  # for PHY 1, 2, 3, 4

temp = args.min_pwrs.replace(" ", "")
pwrs = temp.split(",")
if len(pwrs) == 4:
    min_pwrs = [int(x) for x in pwrs]
    print(f'abs min pwr for each PHY: {min_pwrs}')

packetLengths    = args.pktlen.strip().split(",")
phys             = args.phys.strip().split(",")
txPowers         = args.txpows.strip().split(",")

print("slaveSerial   :", args.slaveSerial)
print("masterSerial  :", args.masterSerial)
print("slave TRACE   :", args.stp)
print("master TRACE  :", args.mtp)
print("results       :", args.results)
print("delay         :", args.delay)
print("packetLengths :", packetLengths)
print("phys          :", phys)
print("txPowers      :", txPowers)
print("PER limit     :", args.limit)

# Open the results file, write the parameters
results = open(args.results, "a")

print("\nReset the attenuation to 30.")
if rf_switch:
    set_val = 30 + float(args.loss)
    mini_RCDAT = mini_RCDAT_USB(Namespace(atten=set_val))
sleep(1)

# Create the BLE_hci objects
hciSlave  = BLE_hci(Namespace(serialPort=args.slaveSerial,  monPort=args.stp, baud=115200, id=2))
hciMaster = BLE_hci(Namespace(serialPort=args.masterSerial, monPort=args.mtp, baud=115200, id=1))

ABORTED = False
perMax = 0
RETRY = int(args.retry_limit)
need_to_setup = True  # only do it at the beginning or after flash

testing = 1
for packetLen, phy, txPower in itertools.product(packetLengths, phys, txPowers):

    if args.attens is None:
        if int(args.step) == 0 or int(args.step) == -1:
            attens = [20, 70]
        else:
            attens = list(range(20, min_pwrs[int(phy)-1], int(args.step)))

        # Add the max attenuation
        if int(args.step) != -1:
            attens.append(min_pwrs[int(phy)-1])
    else:
        temp = args.attens.replace(" ", "")
        attens = temp.split(",")
        attens = [float(x) for x in attens]
    
    # check if use PER mask
    if use_per_mask == "1" and False:
        attens = list()
        for item in per_mask[phy_str[int(phy)]]:
            attens.append(item[0]*(-1))

    print(f'attens: {attens}')

    if args.short:
        for atten in attens:
            per_100 = 0
            while per_100 < RETRY:
                if need_to_setup:
                    need_to_setup = False

                    start_secs = time.time()

                    print(f"{dt.now()} ----- sleep extra 2 secs\n")
                    sleep(2)
                    print(f"\n{dt.now()} ----- end of the sleep")

                    print("\n\n\nReset the devices at the beginning of the test or after flash the board again.")
                    
                    print("\nslave reset")
                    hciSlave.resetFunc(None)
                    print("\nmaster reset")
                    hciMaster.resetFunc(None)
                    sleep(0.2)

                    print("\nSet addresses.")
                    txAddr = "00:12:34:88:77:33"
                    rxAddr = "11:12:34:88:77:33"
                    print(f"\nslave set txAddr: {txAddr}")
                    hciSlave.addrFunc(Namespace(addr=txAddr))
                    print(f"\nmaster set rxAddr: {rxAddr}")
                    hciMaster.addrFunc(Namespace(addr=rxAddr))
                    sleep(0.2)

                    print("\n----------------------------------")                
                    print("pre-test setup")
                    print("----------------------------------")

                    print("\nslave start advertising.")
                    hciSlave.advFunc(Namespace(interval="60", stats="False", connect="True", maintain=False, listen="False"))

                    print("\nmaster start connection.")
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

                print('\n-----------------------------------------------------------------------------------------')
                print(f'{args.chip} - packetLen: {packetLen}, phy: {phy}, atten: {atten}, txPower: {txPower}, testing point: {testing}')
                print('-------------------------------------------------------------------------------------------')

                print(f"\nSet the requested attenuation: {atten}.")
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

                print(f"\nsleep args.delay {args.delay} secs")
                sleep(int(args.delay))

                print("\nRead any pending events. slave and master listenFunc")
                hciSlave.listenFunc(Namespace(time=1, stats="False"))
                hciMaster.listenFunc(Namespace(time=1, stats="False"))

                print("\nMaster collects results.")
                perMaster = hciMaster.connStatsFunc(None)

                print("\nSlave collects results.")
                perSlave = hciSlave.connStatsFunc(None)                

                reset_master = False
                if perMaster is None:
                    print("perMaster is None. Reset the master.")
                    reset_master = True
                elif perMaster >= 99.99:
                    print(f"perMaster {perMaster}% invalid. Reset the master.")
                    reset_master = True
                else:
                    print(f"\n\nperMaster  : {perMaster:.2f} %")
                
                reset_slave = False
                if perSlave is None:
                    print("perSlave is None. Flash the slave.")
                    reset_slave = True
                elif perSlave >= 99.99:
                    print(f"perSlave {perSlave}% invalid. Flash the slave.")
                    reset_slave = True
                else:
                    print(f"perSlave   : {perSlave:.2f} %")
                
                if reset_slave or reset_master:
                    run_script_reset_board(args.brd1_reset)
                    run_script_reset_board(args.brd2_reset)

                    per_100 += 1
                    total_retry_times += 1

                    print("\nReset the attenuation to 30.")
                    if rf_switch:
                        set_val = 30 + float(args.loss)
                        mini_RCDAT = mini_RCDAT_USB(Namespace(atten=set_val))

                    sleep(10)

                    need_to_setup = True
                    type_string = 1

                    continue
                
                # Record max per
                if perMaster > perMax:
                    perMax = perMaster
                if perSlave > perMax:
                    perMax = perSlave
                print(f"\nperMax     : {perMax:.2f} %")

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
            print(f'\nTotally used time for this point (secs): {(end_secs - start_secs):.0f}')

            #if testing >= RESET_CNT:
            #    testing = RESET_CNT
            #
            #    hciMaster.cmdFunc(Namespace(cmd="01060403000013"))  # close connection
            #    need_to_setup = True
            #else:
            #    testing += 1
            testing += 1
            
    else:  # original method
        for atten in attens:
            per_100 = 0
            RETRY = int(args.retry_limit)
            while per_100 < RETRY:
                start_secs = time.time()
                print(f'\n---------------------------------------------------------------------------------------')
                print(f'{args.chip} - packetLen: {packetLen}, phy: {phy}, atten: {atten}, txPower: {txPower}\n')

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
