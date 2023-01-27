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

## RS_fsl3.py
 #
 # Remote control a Rohde&Schwarz,FSL-3
 #

import sys
import argparse
from argparse import RawTextHelpFormatter
from time import sleep
import pyvisa

OBW_LIMIT=5000000

# Namespace class used to create function arguments similar to argparse
class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class RS_fsl3:

    # Create the resource manager object
    rm = pyvisa.ResourceManager()
    sa = ""

    def __init__(self, args):

        try:
            # Connect to the instrument
            queryString = "TCPIP::"+args.ipAddress+"::inst0::INSTR"
            print(f'query: {queryString}')
            self.sa = self.rm.open_resource(queryString)
        except ConnectionRefusedError as err:
            print("Error connecting to instrument at IP:",args.ipAddress)
            sys.exit(1)

        print('query: *IDN?')
        print(self.sa.query('*IDN?'))

        # Leave the display on when remote controlling
        self.sa.write(':SYST:DISP:UPD ON')

        # Reset the device
        self.sa.write('*RST')

    def testOBW(self, ch=39):
        self.sa.write('*RST')
        # Test occupied bandwidth
        self.sa.write(':CALC:MARK:FUNC:POW:SEL OBW')
        self.sa.write(':SENS:POW:BWID 99PCT')
        self.sa.write(':SENS:POW:ACH:PRES OBW')

        # Map the channel to the frequency
        freq = 2402000000 + (ch * 2000000)
        print("CH  :",ch)
        print("freq:",freq)

        self.sa.write(':FREQ:CENT ',str(freq))
        self.sa.write(':FREQ:SPAN 20000000')
        self.sa.write(':BAND:AUTO ON')
        self.sa.write(':BAND:RES 30000')
        self.sa.write(':BAND:VID 100000')
        self.sa.write(':TRIG:SOUR IMM')
        self.sa.write(':DISP:TRAC:MODE MAXH')
        self.sa.write(':AVER:COUN 0')

        # Set the marker
        self.sa.write(':CALC:MARK1:MAX')
        self.sa.write(':CALC:MARK1:X ',str(freq))

        # Set the reference level
        self.sa.write(':DISP:TRAC:Y:RLEV 10dBm')

        while not (self.sa.query('*OPC?')): pass
        self.sa.write('INIT')

        sleep(10)

        obwString = self.sa.query(':CALC:MARK:FUNC:POW:RES? OBW')
        print("OBW:",obwString)
        obwFloat = float(obwString)

        txPowerString = self.sa.query(':CALC:MARK1:Y?')
        txPowerFloat = float(txPowerString)
        print("TX Power :",txPowerString)

        if(obwFloat > OBW_LIMIT):
            print("OBW too large >",OBW_LIMIT)
            return False

        return True

    def testRB2(self):
        self.sa.write('*RST')
        # Test RB2
        self.sa.write(':FREQ:CENT 2480000000') 
        self.sa.write(':FREQ:SPAN 20000000')
        self.sa.write(':BAND:AUTO ON')
        self.sa.write(':BAND:RES 1000000')
        self.sa.write(':TRIG:SOUR IMM')
        self.sa.write(':DISP:TRAC:MODE MAXH')
        self.sa.write(':AVER:COUN 0')

        # Set the reference level
        self.sa.write(':DISP:TRAC:Y:RLEV 10dBm')

        # Set the markers
        self.sa.write(':CALC:MARK1:MAX')
        self.sa.write(':CALC:MARK1:X 2483500000')

        self.sa.write(':CALC:MARK2:MAX')
        self.sa.write(':CALC:MARK2:X 2480000000')

        while not (self.sa.query('*OPC?')): pass
        self.sa.write('INIT')

        sleep(10)

        txPowerString = self.sa.query(':CALC:MARK2:Y?')
        txPowerFloat = float(txPowerString)
        print("TX Power :",txPowerString)

        rb2PowerString = self.sa.query(':CALC:MARK1:Y?')
        rb2PowerFloat = float(rb2PowerString)
        print("RB2 Power:",rb2PowerString)

        if(rb2PowerFloat > -20):
            print("rb2Power too high")
            return False

        return True

    def testRB1(self):
        self.sa.write('*RST')
        # Test RB1
        self.sa.write(':FREQ:CENT 2395000000') 
        self.sa.write(':FREQ:SPAN 20000000')
        self.sa.write(':BAND:AUTO ON')
        self.sa.write(':BAND:RES 1000000')
        self.sa.write(':BAND:VID 3000000')
        self.sa.write(':TRIG:SOUR IMM')
        self.sa.write(':DISP:TRAC:MODE MAXH')
        self.sa.write(':AVER:COUN 0')

        # Set the reference level
        self.sa.write(':DISP:TRAC:Y:RLEV 10dBm')

        # Set the markers
        self.sa.write(':CALC:MARK1:MAX')
        self.sa.write(':CALC:MARK1:X 2390000000')

        self.sa.write(':CALC:MARK2:MAX')
        self.sa.write(':CALC:MARK2:X 2402000000')

        while not (self.sa.query('*OPC?')): pass
        self.sa.write('INIT')

        sleep(10)

        txPowerString = self.sa.query(':CALC:MARK2:Y?')
        txPowerFloat = float(txPowerString)
        print("TX Power :",txPowerString)

        rb1PowerString = self.sa.query(':CALC:MARK1:Y?')
        rb1PowerFloat = float(rb1PowerString)
        print("RB1 Power:",rb1PowerString)

        if(rb1PowerFloat > -20):
            print("rb1Power too high")
            return False

        return True

        
if __name__ == '__main__':

    # Setup the command line description text
    descText = """
    Rohde&Schwarz,FSL-3 automation tools.

    This tool will setup a signal analyzer over IP to take measurements verifying
    FCC Restricted band 2     : < -20 dBm at  2.4835 GHz
    FCC Restricted band 1     : < -20 dBm at  2.3900 GHz
    Occupied bandwidth        : 99% power less than """+str(OBW_LIMIT/1000000)+""" MHz
    """

    # Parse the command line arguments
    parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
    parser.add_argument('ipAddress',help='Instrument IP address')
    parser.add_argument('cmd',help='Command to run: rb2, rb1, obw<channel>')

    args = parser.parse_args()

    print("IP :",args.ipAddress)
    print("cmd:",args.cmd)

    # make sure we only have one command
    commands = 0
    channel = 0
    if("obw" in args.cmd):
        commands = commands+1
        try:
            channel = int(args.cmd.split("obw")[1])
        except ValueError as err:
            print("Error parsing channel")
            print("obw<channel> is the expected command, e.g. obw39")
            sys.exit(1)

    if("rb1" in args.cmd):
        commands = commands+1
    if("rb2" in args.cmd):
        commands = commands+1

    if((commands != 1) or (len(args.cmd) > 5) or (len(args.cmd) < 3)):
        print("cmd not formatted properly:",args.cmd)
        parser.print_help()
        sys.exit(1)

    # Create the object
    rs_fsl3 = RS_fsl3(args)

    if ("obw" in args.cmd):
        # Run the OBW test
        if (rs_fsl3.testOBW(channel) == False):
            sys.exit(1)

    if ("rb2" in args.cmd):
        if (rs_fsl3.testRB2() == False):
            sys.exit(1)

    if ("rb1" in args.cmd):
        if (rs_fsl3.testRB1() == False):
            sys.exit(1)

    sys.exit(0)
