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

## BLE_hci.py
 #
 # HCI Tool used to control a Bluetooth Controller through a serial port.
 #

import serial
import sys
import signal
import codecs
import argparse
from argparse import RawTextHelpFormatter
from time import sleep
import datetime
import threading
from termcolor import colored

# Setup the default serial port settings
defaultBaud=115200
defaultSP="/dev/ttyUSB0"
defaultMonSP=""

# Setup the default Bluetooth settings
defaultAdvInterval="0x60"
defaultScanInterval="0x100"

defaultConnInterval="0x6" # 7.5 ms
defaultSupTimeout="0x64" # 1 s

defaultDevAddr="00:11:22:33:44:55"
defaultInitAddr=defaultDevAddr

# Magic value for the exit function to properly return
exitFuncMagic=999

## Convert integer to hex.
 #
 # Converts integer to hex with a given number of bits for sign extension.
################################################################################
def tohex(val, nbits):
  return hex((val + (1 << nbits)) % (1 << nbits))

## Parse BD address.
 #
 # Reverses a Bluetooth address to bytes, LSB first.
################################################################################
def parseBdAddr(addr):
    # Reorder the address
    addr = addr.split(":")
    if(len(addr) != 6):
        print("Address is wrong length, needs to be 6 bytes separated by ':'")
        return ""
    addrBytes = addr[5]+addr[4]+addr[3]+addr[2]+addr[1]+addr[0]
    return addrBytes

## Parse register address.
 #
 # Reverses a hex number to bytes, LSB first.
################################################################################
def parseAddr(addr, numNibbles=8):
    # Make sure it's a hex number starting with 0x
    if(addr[:2] != "0x"):
        print("Address must be a hex number starting with 0x")

    # Remove the 0x
    addr = addr[2:]

    # Make sure this is a 32 bit address
    if(len(addr) != numNibbles):
        print("Address must be 32 bit hex number")

    # Split the address into bytes
    chunks, chunk_size = len(addr), 2
    addrBytes = [ addr[i:i+chunk_size] for i in range(0, chunks, chunk_size) ]

    # Reverse the bytes to LSB first

    addrString = ""
    for i in range (int(numNibbles/2)-1, -1, -1):
        addrString = addrString + addrBytes[i]

    return addrString

## Parse bytes string.
 #
 # Parses a string of 4 hex bytes, LSB first. Returns 32 bit int.
################################################################################
def parseBytes32(byteString):
    return int(byteString[3]+byteString[2]+byteString[1]+byteString[0], 16)


# Namespace class used to create function arguments similar to argparse
class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class BLE_hci:

    port = serial.Serial()
    serialPort = ""

    def __init__(self, args):
        
        try:
            if "id" in vars(args).keys():
                self.id = args.id
            else:
                self.id = "-"
                
            # Open serial port
            serialPort = args.serialPort
            self.port = serial.Serial(
                port=str(serialPort),
                baudrate=args.baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                rtscts=False,
                dsrdtr=False,
                timeout=1.0
            )
            self.port.isOpen()

            if args.monPort == "":
                self.mon_port = None
            else:
                mon_port = serial.Serial()
                self.mon_port = serial.Serial(
                    port=str(args.monPort),
                    baudrate=args.baud,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    rtscts=False,
                    dsrdtr=False,
                    timeout=1.0
                )
                self.mon_port.isOpen()
                
        except serial.SerialException as err:
            print(err)
            sys.exit(1)

        except OverflowError as err:
            print("baud rate exception, "+str(args.baud)+" is too large")
            print(err)
            sys.exit(1)

        if self.mon_port != None:
            monTraceMsgThread = threading.Thread(target=self.monTraceMsg, daemon=True)
            monTraceMsgThread.start()

    def closeListenDiscon(self):
        # Close the listener thread if active
        self.listenDisconThread 
        self.listenDisconStop

        if(self.listenDisconThread.is_alive()):
            self.listenDisconStop = True
            self.listenDisconThread.join()

    ## Exit function.
     #
     # Sends an exit code that is handled below.
    ################################################################################
    def exitFunc(self, args):

        # Close the serial port
        if self.port.is_open:
            self.port.flush()
            self.port.close()

        if self.mon_port is not None:
            if self.mon_port.is_open:
                self.mon_port.flush()
                self.mon_port.close()

        print("")

        # Close the listener thread if active
        self.closeListenDiscon()

        try:
            if(args.returnVal != None):
                sys.exit(int(args.returnVal))
        except AttributeError:
            sys.exit(exitFuncMagic)

        sys.exit(exitFuncMagic)

    ## Wait for an HCI event.
     #
     # Waits for an HCI event, optionally prints the received event. 
     # Will timeout on the serial port if nothing arrives.
    ################################################################################
    def wait_event(self, print_evt = True, timeout=1.0):

        # Set the serial port timeout
        self.port.timeout=timeout

        # Receive the event
        evt=self.port.read(size=1)
        if(len(evt) == 0):
            # TODO: Read flush
            self.port.flush()
            return ""

        evt=int(codecs.encode(evt, 'hex_codec'),16)
        status_string = '%02X'%evt

        # ACL data
        if(evt == 2):
            # Receive the rest of the header
            hdr = self.port.read(size=4)
            packet_len = hdr[2] + (hdr[3] << 8)
            hdr=int(codecs.encode(hdr, 'hex_codec'),16)
            status_string+= '%08X'%hdr

        # HCI Event
        elif(evt == 4):
            # Receive the rest of the header
            hdr = self.port.read(size=2)
            packet_len = hdr[1]
            hdr=int(codecs.encode(hdr, 'hex_codec'),16)
            status_string+= '%04X'%hdr

        else:
            print("Error: unknown evt = "+str(evt))
            return

        payload = self.port.read(size=packet_len)

        # Print the packet
        if print_evt and len(payload) > 0:
            for i in range(0, len(payload)):
                status_string += '%02X'%payload[i]

            if self.id == "-":
                print(str(datetime.datetime.now()) + "  <", status_string)
            else:
                print(str(datetime.datetime.now()) + f" {self.id}<", status_string)

        return status_string

    ## Wait for HCI events.
     #
     # Waits to receive HCI events, prints the timestamp every 30 seconds.
    ################################################################################
    def wait_events(self, seconds = 2, print_evt = True):
        # Read events from the device for a few seconds
        start_time = datetime.datetime.now()
        delta = datetime.datetime.now() - start_time
        while((delta.seconds < seconds) or (seconds == 0)):
            self.wait_event(print_evt = print_evt, timeout=0.1)
            delta = datetime.datetime.now() - start_time
            if((delta.seconds > 30) and ((delta.seconds % 30) == 0)) :
                print(str(datetime.datetime.now()) + " |")

    ## Send HCI command.
     #
     # Send a HCI command to the serial port. Will add a small delay and wait for
     # and print an HCI event by default.
    ################################################################################
    def send_command(self, packet, resp = True, delay = 0.01, print_cmd = True, timeout=3):
        # Send the command and data
        if(print_cmd):
            if self.id == "-":
                print(str(datetime.datetime.now()) + "  >", packet)
            else:
                print(str(datetime.datetime.now()) + f" {self.id}>", packet)

        self.port.write(bytearray.fromhex(packet))
        sleep(delay)

        if(resp):
            return self.wait_event(timeout=timeout)


    ## Parse connection stats event.
     #
     # Parses a connection stats event and prints the results.
    ################################################################################
    def parseConnStatsEvt(self, evt):
        """
        Example:
            2023-01-04 12:41:48.018410 2> 01FDFF00
            2023-01-04 12:41:48.029006 2< 040E2001FDFF00880000000100000000000000880000000000000000000A0016000700

            rxDataOk   : 136
            rxDataCRC  : 1
            rxDataTO   : 0
            txData     : 136
            errTrans   : 0
            PER        : 0.73 %

            2023-01-04 16:04:27.486935 1> 01FDFF00
            2023-01-04 16:04:27.497405 1< 040E2001FDFF0000000000000000000000000000000000000000000000000000000000
            rxDataOk   : 0
            rxDataCRC  : 0
            rxDataTO   : 0
            txData     : 0
            errTrans   : 0
            perMaster  :  100.0

        :param evt:
        :return: per
        """
        try:
            # Offset into the event where the stats start, each stat is 32 bits, or
            # 8 hex nibbles
            i = 14
            rxDataOk  = int(evt[6+i:8+i]+evt[4+i:6+i]+evt[2+i:4+i]+evt[0+i:2+i],16)
            i += 8
            rxDataCRC = int(evt[6+i:8+i]+evt[4+i:6+i]+evt[2+i:4+i]+evt[0+i:2+i],16)
            i += 8
            rxDataTO  = int(evt[6+i:8+i]+evt[4+i:6+i]+evt[2+i:4+i]+evt[0+i:2+i],16)
            i += 8
            txData    = int(evt[6+i:8+i]+evt[4+i:6+i]+evt[2+i:4+i]+evt[0+i:2+i],16)
            i += 8
            errTrans  = int(evt[6+i:8+i]+evt[4+i:6+i]+evt[2+i:4+i]+evt[0+i:2+i],16)
        except ValueError as err:
            print(err)
            return None


        print(self.serialPort)
        print("rxDataOk   : "+str(rxDataOk))
        print("rxDataCRC  : "+str(rxDataCRC))
        print("rxDataTO   : "+str(rxDataTO))
        print("txData     : "+str(txData))
        print("errTrans   : "+str(errTrans))

        per = 100.0
        if (rxDataCRC+rxDataTO+rxDataOk) != 0:
            per = round(float((rxDataCRC+rxDataTO)/(rxDataCRC+rxDataTO+rxDataOk))*100, 2)
            print("PER        : "+str(per)+" %")

        return per


    ## Monitor the UART0
     #
     # Listen for the trace message from the board UART0
    ################################################################################
    def monTraceMsg(self):
        first = True
        while True:
            if self.mon_port is not None:
                msg = self.mon_port.readline().decode("utf-8")
                msg = msg.replace("\r\n", "")
                if msg != "":
                    if first:
                        print(f'\n{str(datetime.datetime.now())} {self.id}  {msg}')
                        first = False
                    else:
                        print(f'{str(datetime.datetime.now())} {self.id}  {msg}')


    ## Get connection stats.
     #
     # Send the command to get the connection stats, parse the return value, return the PER.
    ################################################################################
    def connStatsFunc(self, args):

        per = None
        retries = 5
        while((per == None) and (retries > 0)):
            # Send the command to get the connection stats, save the event
            statEvt = self.send_command("01FDFF00")

            if(retries != 5):
                # Delay to clear pending events
                self.wait_events(1)
                

            # Parse the connection stats event
            per = self.parseConnStatsEvt(statEvt)

            retries = retries - 1

        if retries == 0 and per is None:
            print(colored('Warning: Failed to get connection stats', 'yellow'))
        
        return per


    ## Listen for disconnection events.
     #
     #  Listen for events and print them to the terminal. Send command if we get a disconnect event.
    ################################################################################
    def listenDiscon(self):

        # TODO: Add a way to cancel this
        if(self.listenDisconCommand != ""):
            self.send_command(listenDisconCommand)

        while(True):
            evt = self.wait_event(timeout=0.1)
            if(self.listenDisconStop):
                sys.exit(1)
            if("040504" in evt):
                if(self.listenDisconCommand != ""):
                    self.send_command(listenDisconCommand)
    # Thread used to listen for disconnection events
    listenDisconThread = threading.Thread(target=listenDiscon)
    listenDisconStop = False
    listenDisconCommand = ""

    ## Set BD address command.
     #
     #  Sets the public BD address for the device.
    ################################################################################
    def addrFunc(self, args):
        # Reorder the address
        addrBytes = parseBdAddr(args.addr)

        # Send the vendor specific set address command
        self.send_command("01F0FF06"+addrBytes)

    ## Start advertising function.
     #
     # Sends HCI commands to start advertising.
    ################################################################################
    def advFunc(self, args):
        # Bogus address to use for commands, allows any peer to connect
        peer_addr = "000000000000"

        # Setup the event masks
        self.send_command("01010C08FFFFFFFFFFFFFFFF")
        self.send_command("01630C08FFFFFFFFFFFFFFFF")
        self.send_command("01010C08FFFFFFFFFFFFFFFF")
        self.send_command("01012008FFFFFFFFFFFFFFFF")

        # Reset the connection stats
        if(args.stats):
            self.send_command("0102FF00")

        # Set default PHY to enable all PHYs
        self.send_command("01312003"+"00"+"07"+"07")

        advType = "03" # Non connectable undirected advertising (ADV_NONCONN_IND)
        if(args.connect == "True"):
            advType = "00" # Connectable and scannable undirected advertising (ADV_IND)

        # Convert advertising interval string to int
        advIntervalInt = int(args.interval, 16)

        # Convert int to 2 hex bytes, LSB first
        advInterval = "%0.2X"%(advIntervalInt & 0xFF)
        advInterval+= "%0.2X"%((advIntervalInt & 0xFF00) >> 8)

        # LeSetAdvParam(Advertising_Interval_Min=advInterval,
        # Advertising_Interval_Max=advInterval,
        # Advertising_Type=advType,
        # Own_Address_Type=0 (public),
        # Peer_Address_Type=0 (public),
        # Peer_Address=peer_addr,
        # Advertising_Channel_Map=7 (all 3 advertising channels),
        # Advertising_Filter_Policy=0 (don't do any filtering))
        self.send_command("0106200F"+advInterval+advInterval+advType+"0000"+peer_addr+"0700")

        # Start advertising
        connCommand = "010A200101"

        # Start a thread to listen for disconnection events and restart advertising
        if(args.maintain == True):
            global listenDisconCommand
            global listenDisconStop
            global listenDisconThread

            listenDisconThread = threading.Thread(target=listenDiscon)
            listenDisconCommand = connCommand
            listenDisconStop = False

            # Start the thread and wait for it to startup
            listenDisconThread.start()
            sleep(1)

            return
        else:
            self.send_command(connCommand)

        if(args.listen == "False"):
            return

        # Listen for events indef
        if(args.stats):
            per = 100.0
            listenTime = int(args.listen)
            while (listenTime > 0):
                self.wait_events(10)

                # Send the command to get the connection stats, save the event
                statEvt = self.send_command("01FDFF00")

                # Parse the connection stats event
                per = self.parseConnStatsEvt(statEvt)

                listenTime = listenTime - 10

            return per

        # Listen for events for a few seconds
        if(args.listen != "True"):
            self.wait_events(int(args.listen))
            return
        else:
            while True:
                self.wait_events(0)


    ## Start scanning function.
     #
     # Sends HCI commands to start scanning.
    ################################################################################
    def scanFunc(self, args):

        # Setup the event masks
        self.send_command("01010C08FFFFFFFFFFFFFFFF")
        self.send_command("01630C08FFFFFFFFFFFFFFFF")
        self.send_command("01010C08FFFFFFFFFFFFFFFF")
        self.send_command("01012008FFFFFFFFFFFFFFFF")

        # Set scan parameters
        # Active scanning
        # window and interval 0xA0
        # Own address type is 0, public
        # Not doing any filtering
        self.send_command("010B200701A000A0000000")

        # Start scanning, don't filter duplicates
        self.send_command("010C20020100")

        while True:
            self.wait_event()

    ## Start initiating function.
     #
     # Sends HCI commands to start initiating and create a connection.
    ################################################################################
    def initFunc(self, args):
        # Reorder the address
        addrBytes = parseBdAddr(args.addr)

        # Setup the event masks
        self.send_command("01010C08FFFFFFFFFFFFFFFF")
        self.send_command("01630C08FFFFFFFFFFFFFFFF")
        self.send_command("01010C08FFFFFFFFFFFFFFFF")
        self.send_command("01012008FFFFFFFFFFFFFFFF")

        # Reset the connection stats
        if(args.stats):
            self.send_command("0102FF00")

        # Set default PHY to enable all PHYs
        self.send_command("01312003"+"00"+"07"+"07")

        # Convert connection interval string to int
        connIntervalInt = int(args.interval, 16)

        # Convert int to 2 hex bytes, LSB first
        connInterval = "%0.2X"%(connIntervalInt & 0xFF)
        connInterval+= "%0.2X"%((connIntervalInt & 0xFF00) >> 8)

        # Convert supervision timeout string to int
        supTimeoutInt = int(args.timeout, 16)

        # Convert int to 2 hex bytes, LSB first
        supTimeout = "%0.2X"%(supTimeoutInt & 0xFF)
        supTimeout+= "%0.2X"%((supTimeoutInt & 0xFF00) >> 8)

        # Create the connection, using a public address for peer and local
        ownAddrType="00"
        connLatency="0000"
        connCommand="010D2019A000A00000"+"00"+addrBytes+ownAddrType+connInterval+connInterval+connLatency+supTimeout+"0F100F10"

        # Start a thread to listen for disconnection events and restart the connection
        if(args.maintain == True):
            global listenDisconCommand
            global listenDisconStop
            global listenDisconThread

            listenDisconThread = threading.Thread(target=listenDiscon)
            listenDisconCommand = connCommand
            listenDisconStop = False

            # Start the thread and wait for it to startup
            listenDisconThread.start()
            sleep(1)

            return
        else:
            self.send_command(connCommand)

        if(args.listen == "False"):
            return

        # Listen for events indef
        if(args.stats):
            per = 100.0
            listenTime = int(args.listen)
            while (listenTime > 0):
                self.wait_events(10)

                # Send the command to get the connection stats, save the event
                statEvt = self.send_command("01FDFF00")

                # Parse the connection stats event
                per = self.parseConnStatsEvt(statEvt)

                listenTime = listenTime - 10

            return per

        # Listen for events for a few seconds
        if(args.listen != "True"):
            self.wait_events(int(args.listen))
            return
        else:
            while True:
                self.wait_events(0)

    ## Set the data length.
     #
     #  Sets the TX data length in octets and time for handle 0.
    ################################################################################
    def dataLenFunc(self, args):
        # Send the set data length command with max length
        self.send_command("01222006"+"0000"+"FB00"+"9042")

    ## Set the length of empty packets.
     #
     #  Sets the length of empty packets.
    ################################################################################
    def sendAclFunc(self, args):
        
        packetLen = "%0.2X"%(int(args.packetLen) & 0xFF)
        packetLen+= "%0.2X"%((int(args.packetLen) & 0xFF00) >> 8)

        if(args.numPackets == "0"):
            self.send_command("01E5FF02"+packetLen)
            return

        numPackets = "%0.2X"%(int(args.numPackets) & 0xFF)

        # Send the vendor specific command to send ACL packets, handle 0
        self.send_command("01E4FF05"+"0000"+packetLen+numPackets)

        ## Set the length of empty packets.


     #
     #  Sets the length of empty packets.
    ################################################################################
    def sinkAclFunc(self, args):

        # Send the vendor specific command to sink ACL packets
        self.send_command("01E3FF0101")

    ## PHY switch function.
     #
     # Sends HCI command to switch PHYs. Assumes that we can't do asymmetric PHY settings.
     # Assumes we're using connection handle 0000
    ################################################################################
    def phyFunc(self, args, timeout=3):
        # Convert PHY options to bits
        phy="01"
        phyOptions="0000"
        if(args.phy == "4"):
            phy="04"
            phyOptions="0100"
        elif(args.phy == "3"):
            phy="04"
            phyOptions="0200"
        elif(args.phy == "2"):
            phy="02"
        elif(args.phy != "1"):
            print("Invalid PHY selection, using 1M")

        self.send_command("01322007"+"0000"+"00"+phy+phy+phyOptions)
        self.wait_events(timeout)

    ## Rest function.
     #
     # Sends HCI reset command.
    ################################################################################
    def resetFunc(self, args):

        # Close the listener thread if active
        self.closeListenDiscon()

        # Send the HCI command for HCI Reset
        self.send_command("01030C00")

    ## Listen for events.
     #
     # Listen for HCI events.
    ################################################################################
    def listenFunc(self, args):
        waitSeconds = int(args.time)

        per = 100.0

        if(args.stats):

            startTime = datetime.datetime.now()
            while True:

                # Wait for at least 10 seconds
                waitPrintSeconds = waitSeconds
                if(waitSeconds == 0):
                    waitPrintSeconds = 10
                self.wait_events(waitPrintSeconds)

                # Send the command to get the connection stats, save the event
                statEvt = self.send_command("01FDFF00")

                # Parse the connection stats event
                per = self.parseConnStatsEvt(statEvt)

                timeNow = datetime.datetime.now()

                if((waitSeconds != 0) and ((timeNow - startTime).total_seconds() > waitSeconds)):
                    return per

        else:
            self.wait_events(waitSeconds)

        return per


    ## txTest function.
     #
     # Sends HCI command for the transmitter test.
    ################################################################################
    def txTestFunc(self, args):
        channel="%0.2X"%int(args.channel)
        packetLength="%0.2X"%int(args.packetLength)
        payload="%0.2X"%int(args.payload)
        phy="%0.2X"%int(args.phy)
        self.send_command("01342004"+channel+packetLength+payload+phy)

    ## txTestVS function.
     #
     # Sends a vendor specific HCI command for the transmitter test.
    ################################################################################
    def txTestVSFunc(self, args):
        channel="%0.2X"%int(args.channel)
        packetLength="%0.2X"%int(args.packetLength)
        payload="%0.2X"%int(args.payload)
        phy="%0.2X"%int(args.phy)

        numPackets = "%0.2X"%(int(args.numPackets) & 0xFF)
        numPackets+= "%0.2X"%((int(args.numPackets) & 0xFF00) >> 8)

        self.send_command("0103FF06"+channel+packetLength+payload+phy+numPackets)

    ## rxTest function.
     #
     # Sends HCI command for the receiver test.
    ################################################################################
    def rxTestFunc(self, args):
        channel="%0.2X"%int(args.channel)
        phy="%0.2X"%int(args.phy)
        modulationIndex="00"
        self.send_command("01332003"+channel+phy+modulationIndex)
    
   
    def endTestVSFunc(self, args) -> dict | None:
        """
        Vendor specific command to end test\n
        Returns a dictionary of entire test report\n
        Keys:\n
            rxDataTO -  Timeouts on receive\n
            rxDataCRC - RX data with CRC errors\n
            rxDataOk - Total RX data received with no issues\n
            txData   - Total number of TX packets sent\n
        """
        
        evtString = self.send_command("0104FF00")

        if evtString:                      
            stats = {}

            #flip every two character strings to create a big endian string to cast            
            stats['rxDataTO']   = int(evtString[-2:] + evtString[-4 :-2],16)
            stats['rxDataCRC']  = int(evtString[-6 :-4] + evtString[-8:-6],16) 
            stats['rxDataOk']   = int(evtString[-10 :-8] + evtString[-12:-10] ,16)
            stats['txData']     = int(evtString[-14 :-12] + evtString[-16:-14],16)

            # if args is None or args.noPrint is False:
            for item in stats:
                print(item, stats[item])
                
            return stats
        else:

            return None


    ## End Test function.
     #
     # Sends HCI command for the end test command.
    ################################################################################
    def endTestFunc(self, args):
        # Send the end test command, store the event
        evtString = self.send_command("011F2000")

        # Parse the event and print the number of received packets
        evtData = int(evtString, 16)
        rxPackets = int((evtData & 0xFF00)>>8)+int((evtData & 0xFF)<<8)
        if (args is None) or (vars(args).get('noPrint') is not True):
            print("Received PKTS  : "+str(rxPackets))

        return rxPackets

    ## txPower function.
     #
     # Sends HCI command to set the TX power.
    ################################################################################
    def txPowerFunc(self, args):

        # Get an 8 bit signed integer
        power="%0.2X"%int(tohex(int(args.power), nbits=8), base=16)

        # Set the advertising TX power level
        self.send_command("01F5FF01"+power)

        if(args.handle):
            # Convert int handle to 2 hex bytes, LSB first
            handle = "%0.2X"%(int(args.handle) & 0xFF)
            handle+= "%0.2X"%((int(args.handle) & 0xFF00) >> 8)

            # Set the connection TX power level
            self.send_command("01F6FF03"+handle+power)  

    ## Disconnect function.
     #
     # Sends HCI command to disconnect from a connection.
    ################################################################################
    def disconFunc(self, args):
        # Send the disconnect command, handle 0, reason 0x16 Local Host Term
        self.send_command("01060403000016")

    ## Set channel map function.
     #
     # Sends vendor specific HCI commands to set the channel map.
    ################################################################################
    def setChMapFunc(self, args):

        chMask = 0xFFFFFFFFFF
        if(args.mask == None):
            if(args.chan == None):
                # Use all of the channels
                chMask = 0xFFFFFFFFFF
            elif(args.chan == "0"):
                # Use channels 0 and 1
                chMask = 0x0000000003
            else:
                chMask = 0x0000000001
                chMask = chMask | (1 << int(args.chan))

        else:
            chMask = int(args.mask, 16)

        # Mask off the advertising channels
        chMask = chMask & ~(0xE000000000)

        # Convert to a string
        chMaskString = "0x%0.16X"%(chMask)
        print(chMaskString)
        maskString = parseAddr(chMaskString, numNibbles=16)


        # Convert int handle to 2 hex bytes, LSB first
        handle = "%0.2X"%(int(args.handle) & 0xFF)
        handle+= "%0.2X"%((int(args.handle) & 0xFF00) >> 8)

        print(maskString)
        self.send_command("01F8FF0A"+handle+maskString)

    ## Command function.
     #
     # Sends HCI commands.
    ################################################################################
    def cmdFunc(self,args, timeout=None):
        if timeout is None:
            self.send_command(args.cmd)
        else:
            self.send_command(args.cmd, timeout=timeout)

    ## Read register function.
     #
     # Sends HCI command to read a register.
    ################################################################################
    def readRegFunc(self, args):

        # Get the addr string
        addr=args.addr
        
        # Reverse the bytes to LSB first
        addrBytes = parseAddr(addr)

        # Get the read length
        readLen = args.length
        if(readLen[:2] != "0x"):
            print("Length must be a hex number starting with 0x")
            return
        readLen = readLen[2:]
        readLenString = "%0.2X"%int(readLen, 16)

        # Calculate the total length, 1 for the read len, 4 for the address length
        totalLen = "%0.2X"%(1+4)

        # Send the command and save the event
        evtString = self.send_command("0101FF"+totalLen+readLenString+addrBytes)

        # Get the data
        evtString = evtString[14:]

        # Split the data into bytes
        chunks, chunk_size = len(evtString), 2
        evtBytes = [ evtString[i:i+chunk_size] for i in range(0, chunks, chunk_size) ]

        # Print the data
        startingAddr = int(args.addr, 16)

        # Pad the length so we can print 32-bit numbers
        readLen = int(readLen,16)
        readLenPad = readLen
        if(readLen % 4):
            readLenPad += (4-readLen%4)

        for i in range (0, readLenPad):
            addr = startingAddr+i

            # Print the address every 4 bytes
            if(i%4 == 0):
                print("0x%0.8X"%addr, end=": 0x")

            # Print from MSB to LSB in the 32 bit value
            lineAddr = int(i/4)*4 + (4-(i%4)) - 1

            # Print spaces if we're padding the length
            if(lineAddr >= readLen):
                print("__", end="")
            else:
                print(evtBytes[lineAddr], end="")

            # Print a new line at the end of the 32 bit value
            if(i%4 == 3):
                print()

        return evtBytes

    ## Write register function.
     #
     # Sends HCI command to write a register.
    ################################################################################
    def writeRegFunc(self, args):

        # Get the data string
        data=args.value

        # Make sure input value is a hex number starting with 0x
        if(data[:2] != "0x"):
            print("Input value must be a hex number starting with 0x")

        data = data[2:]

        # Get the total length, minus 2 for the 0x
        writeLen=len(data)

        # Make sure the writeLen is an even number
        if(writeLen % 2 != 0):
            print("Input value must be on a byte boundary, even number of digits")
            return

        # Convert nibbles to number of bytes
        writeLen = int(writeLen/2)

        if((writeLen != 4) and (writeLen != 2) and (writeLen != 1)):
            print("Input value must be either 8, 16, or 32 bits")
            return

        # Calculate the length, convert to string
        totalLen="%0.2X"%(writeLen+5)
        writeLen="%0.2X"%(writeLen)

        # Get the addr string
        addr=args.addr
        
        # Make sure it's a hex number starting with 0x
        if(addr[:2] != "0x"):
            print("Address must be a hex number starting with 0x")

        addr = addr[2:]

        if(len(addr) != 8):
            print("Address must be 32 bit hex number")

        # Split the address into bytes
        chunks, chunk_size = len(addr), len(addr)//4
        addrBytes = [ addr[i:i+chunk_size] for i in range(0, chunks, chunk_size) ]

        # Reverse the bytes to LSB first
        addrBytes = addrBytes[3]+addrBytes[2]+addrBytes[1]+addrBytes[0]

        self.send_command("0100FF"+totalLen+writeLen+addrBytes+data)

## Help function.
 #
 # Prints the help text.
################################################################################
def helpFunc(args):
    terminal.print_help()

## Signal handler.
 #
 # Catches OS signals and closes the serial port.
################################################################################
def signal_handler(signal, frame):
    print()
    sys.exit(0)
        
if __name__ == '__main__':

    # Setup the signal handler to catch the ctrl-C
    signal.signal(signal.SIGINT, signal_handler)

    # Setup the command line description text
    descText = """
    Bluetooth Low Energy HCI tool.

    This tool is used in tandem with the BLE controller examples. This tools sends
    HCI commands through the serial port to the target device. It will receive and print
    the HCI events received from the target device.

    Serial port is configured as 8N1, no flow control, default baud rate of """+str(defaultBaud)+""".
    """

    # Parse the command line arguments
    parser = argparse.ArgumentParser(description=descText, formatter_class=RawTextHelpFormatter)
    parser.add_argument('serial_port', nargs='?', default="",
                        help='Serial port path or COM#, default: '+defaultSP)
    parser.add_argument('baud', nargs='?', default=defaultBaud,
                        help='Serial port baud rate, default: '+str(defaultBaud))
    parser.add_argument('--monPort', nargs='?', default=defaultMonSP,
                        help='Monitor Trace Msg Serial Port path or COM#, default: ' + defaultMonSP)
    parser.add_argument('--serialPort', nargs='?', default=defaultSP,
                        help='Serial port path or COM#, default: '+defaultSP)
    parser.add_argument('--baud', nargs='?', default=defaultBaud,
                        help='Serial port baud rate, default: '+str(defaultBaud))
    parser.add_argument('-c', '--command', default="", help='Commands to run')

    args = parser.parse_args()
    if args.serial_port != "":
        args.serialPort = args.serial_port
    monSP = args.monPort  # monitor trace msg serial port

    print("Bluetooth Low Energy HCI tool")
    print("Serial port: " + args.serialPort)
    print("Monitor Trace Msg Serial Port: "+monSP)
    print("8N1 "+str(args.baud))
    if(args.command != ""):
        print("running commands: "+args.command)
    print("")

    # Create the BLE_hci object
    ble_hci = BLE_hci(args)

    # Start the terminal argparse
    terminal = argparse.ArgumentParser(prog="", add_help=True)
    subparsers = terminal.add_subparsers()

    addr_parser = subparsers.add_parser('addr', help="Set the device address")
    addr_parser.add_argument('addr',
        help="Set the device address, ex: 00:11:22:33:44:55 ")
    addr_parser.set_defaults(func=ble_hci.addrFunc)

    adv_parser = subparsers.add_parser('adv', help="Send the advertising commands", formatter_class=RawTextHelpFormatter)
    adv_parser.add_argument('-i', '--interval', default=str(defaultAdvInterval), 
        help="Advertising interval in units of 0.625 ms, 16-bit hex number 0x0020 - 0x4000, default: "+str(defaultAdvInterval))
    adv_parser.add_argument('-c', '--connect', default="True", help="Advertise as a connectable device, default: True")
    adv_parser.add_argument('-l', '--listen', default="False", help="Listen for events \n\t\"True\" for indefinitely, ctrl-c to exit \n\t\"False\" to return \n\tnumber of seconds")
    adv_parser.add_argument('-s', '--stats', action='store_true', help="Periodically print the connection stats if listening")
    adv_parser.add_argument('-m', '--maintain', action='store_true', help="Setup an event listener to restart advertising if we disconnect")
    adv_parser.set_defaults(func=ble_hci.advFunc)

    scan_parser = subparsers.add_parser('scan', help="Send the scanning commands and print scan reports. ctrl-c to exit")
    scan_parser.add_argument('-i', '--interval', default=str(defaultAdvInterval), 
        help="Advertising interval in units of 0.625 ms, 16-bit hex number 0x0020 - 0x4000, default: "+str(defaultAdvInterval))
    scan_parser.set_defaults(func=ble_hci.scanFunc)

    init_parser = subparsers.add_parser('init', help="Send the initiating commands to open a connection", formatter_class=RawTextHelpFormatter)
    init_parser.add_argument('addr', help="Address of peer to connect with, ex: 00:11:22:33:44:55 ")
    init_parser.add_argument('-i', '--interval', default=str(defaultConnInterval), 
        help="Connection interval in units of 1.25 ms, 16-bit hex number 0x0006 - 0x0C80, default: "+str(defaultConnInterval))
    init_parser.add_argument('-t', '--timeout', default=str(defaultSupTimeout), 
        help="Supervision timeout in units of 10 ms, 16-bit hex number 0x000A - 0x0C80, default: "+str(defaultSupTimeout))
    init_parser.add_argument('-l', '--listen', default="False", help="Listen for events \n\t\"True\" for indefinitely, ctrl-c to exit \n\t\"False\" to return \n\tnumber of seconds")
    init_parser.add_argument('-s', '--stats', action='store_true', help="Periodically print the connection stats if listening")
    init_parser.add_argument('-m', '--maintain', action='store_true', help="Setup an event listener to restart the connection if we disconnect")
    init_parser.set_defaults(func=ble_hci.initFunc)

    dataLen_parser = subparsers.add_parser('dataLen', help="Set the max data length", formatter_class=RawTextHelpFormatter)
    dataLen_parser.set_defaults(func=ble_hci.dataLenFunc)

    sendAcl_parser = subparsers.add_parser('sendAcl', help="Send ACL packets", formatter_class=RawTextHelpFormatter)
    sendAcl_parser.add_argument('packetLen', help="Number of bytes per ACL packet, 16-bit decimal ex: 128, 0 to disable")
    sendAcl_parser.add_argument('numPackets', help="Number of packets to send, 8-bit decimal ex: 255, 0 to enable auto-generate ")
    sendAcl_parser.set_defaults(func=ble_hci.sendAclFunc)

    sinkAcl_parser = subparsers.add_parser('sinkAcl', help="Sink ACL packets, do not send events to host", formatter_class=RawTextHelpFormatter)
    sinkAcl_parser.set_defaults(func=ble_hci.sinkAclFunc)

    connStats_parser = subparsers.add_parser('connStats', help="Get the connection stats", formatter_class=RawTextHelpFormatter)
    connStats_parser.set_defaults(func=ble_hci.connStatsFunc)

    phy_parser = subparsers.add_parser('phy', help="Update the PHY in the active connection", formatter_class=RawTextHelpFormatter)
    phy_parser.add_argument('phy', help=
    """
    Desired PHY
    1: 1M
    2: 2M
    3: S8 
    4: S2
    default: 1M
    """)
    phy_parser.set_defaults(func=ble_hci.phyFunc)

    reset_parser = subparsers.add_parser('reset', help="Sends a HCI reset command")
    reset_parser.set_defaults(func=ble_hci.resetFunc)

    listen_parser = subparsers.add_parser('listen', help="Listen for HCI events, print to screen")
    listen_parser.add_argument('-t', '--time', default="0", help="Time to listen in seconds, default: 0(indef)")
    listen_parser.add_argument('-s', '--stats', action='store_true', help="Periodically print the connection stats if listening")
    listen_parser.set_defaults(func=ble_hci.listenFunc)

    txTest_parser = subparsers.add_parser('txTest', aliases=['tx'], help="Execute the transmitter test", formatter_class=RawTextHelpFormatter)
    txTest_parser.add_argument('-c', '--channel', default="0", help="TX test channel, 0-39, default: 0")
    txTest_parser.add_argument('--phy', default="1", help=
    """TX test PHY
        1: 1M
        2: 2M
        3: S8 
        4: S2
        default: 1M
    """)
    txTest_parser.add_argument('-p','--payload', default="0", help=
    """TX test Payload
        0: PRBS9
        1: 11110000
        2: 10101010
        3: PRBS15
        4: 11111111
        5: 00000000
        6: 00001111
        7: 01010101
        default: PRBS9
    """)
    txTest_parser.add_argument('-pl', '--packetLength', default="0", help=
    """"TX packet length, number of bytes per packet, 0-255
        default: 0
    """)
    txTest_parser.set_defaults(func=ble_hci.txTestFunc)

    txTestVS_parser = subparsers.add_parser('txTestVS', aliases=['tx'], help="Execute the transmitter test", formatter_class=RawTextHelpFormatter)
    txTestVS_parser.add_argument('-c', '--channel', default="0", help="TX test channel, 0-39, default: 0")
    txTestVS_parser.add_argument('--phy', default="1", help=
    """TX test PHY
        1: 1M
        2: 2M
        3: S8
        4: S2
        default: 1M
    """)
    txTestVS_parser.add_argument('-p','--payload', default="0", help=
    """TX test Payload
        0: PRBS9
        1: 11110000
        2: 10101010
        3: PRBS15
        4: 11111111
        5: 00000000
        6: 00001111
        7: 01010101
        default: PRBS9
    """)
    txTestVS_parser.add_argument('-pl', '--packetLength', default="0", help=
    """"TX packet length, number of bytes per packet, 0-255
        default: 0
    """)
    txTestVS_parser.add_argument('-np', '--numPackets', default="0", help=
    """"Number of packets to TX, 2 bytes hex, 0 equals inf.
        default: 0
    """)
    txTestVS_parser.set_defaults(func=ble_hci.txTestVSFunc)

    rxTest_parser = subparsers.add_parser('rxTest', aliases=['rx'], help="Execute the receiver test")
    rxTest_parser.add_argument('-c', '--channel', default="0", help="RX test channel, 0-39, default: 0")
    rxTest_parser.add_argument('--phy', default="1", help=
    """RX test PHY
        1: 1M
        2: 2M
        3: S8 
        4: S2
        default: 1M
    """)
    rxTest_parser.set_defaults(func=ble_hci.rxTestFunc)

    endTest_parser = subparsers.add_parser('endTest', aliases=['end'], 
        help="End the TX/RX test, print the number of correctly received packets")
    endTest_parser.set_defaults(func=ble_hci.endTestFunc)

    endTestVS_parser = subparsers.add_parser('endTestVS', aliases=['end'], 
        help="End the TX/RX test, print the full test report, and return the report as a dictionary")
    endTestVS_parser.set_defaults(func=ble_hci.endTestVSFunc)

    txPower_parser = subparsers.add_parser('txPower', aliases=['txp'], help="Set the TX power", formatter_class=RawTextHelpFormatter)
    txPower_parser.add_argument('power', help="""Integer power setting in units of dBm""")
    txPower_parser.add_argument('--handle', help="Connection handle, integer")
    txPower_parser.set_defaults(func=ble_hci.txPowerFunc)

    discon_parser = subparsers.add_parser('discon', aliases=['dc'], 
        help="Send the command to disconnect")
    discon_parser.set_defaults(func=ble_hci.disconFunc)

    setChMap_parser = subparsers.add_parser('setChMap', formatter_class=RawTextHelpFormatter, 
        help="""Set the connection channel map to a given channel.""")
    setChMap_parser.add_argument('chan', help="""Channel to use in channel map
    Will set the channel map to the given channel, plus one additional channel.""", nargs="?")
    setChMap_parser.add_argument('-m','--mask', help="""40 bit hex number to use a channel map
    0xFFFFFFFFFF will use all channels, 0x000000000F will use channels 0-3""")
    setChMap_parser.add_argument('--handle', help="Connection handle, integer", default="0")
    setChMap_parser.set_defaults(func=ble_hci.setChMapFunc)

    cmd_parser = subparsers.add_parser('cmd', formatter_class=RawTextHelpFormatter, 
        help="Send raw HCI commands")
    cmd_parser.add_argument('cmd', help="String of hex bytes LSB first\nex: \"01030C00\" to send HCI Reset command")
    cmd_parser.add_argument('-l', '--listen', action='store_true', help="Listen for events indefinitely, ctrl-c to exit")
    cmd_parser.set_defaults(func=ble_hci.cmdFunc)

    readReg_parser = subparsers.add_parser('readReg', formatter_class=RawTextHelpFormatter, 
        help="Read register, device performs a memcpy from address and returns the value")
    readReg_parser.add_argument('addr', help="Address to read, 32-bit hex value\nex: \"0x20000000\"")
    readReg_parser.add_argument('length', help="Number of bytes to read, hex value\nex: \"0x2\"")
    readReg_parser.set_defaults(func=ble_hci.readRegFunc)

    readWrite_parser = subparsers.add_parser('writeReg', formatter_class=RawTextHelpFormatter, 
        help="Write register, device performs a memcpy to memory address")
    readWrite_parser.add_argument('addr', help="Address to write, 32-bit hex value\nex: \"0x20000000\"")
    readWrite_parser.add_argument('value', help="Data to write, 8,16, or 32 bit hex value,\nex: \"0x12\"")
    readWrite_parser.set_defaults(func=ble_hci.writeRegFunc)

    # Exit function defined above
    exit_parser = subparsers.add_parser('exit', aliases=['quit'], help="Exit the program")
    exit_parser.set_defaults(func=ble_hci.exitFunc)

    help_parser = subparsers.add_parser('help', aliases=['h'], help="Show help message")
    help_parser.set_defaults(func=helpFunc)

    # Parse the command input and execute the appropriate function
    if(args.command != ""):
        commands=args.command.split(";")
        for i in range (0,len(commands)):
            # Remove leading and trailing white space
            command = commands[i].strip()

            # Split the command into its arguments
            command = command.split()

            # Run the commands
            try:
                args = terminal.parse_args(command)
                args.func(args)
            except AttributeError:
                continue
                
            # Catch SystemExit, allows user to ctrl-c to quit the current command
            except SystemExit as err:
                if("{0}".format(err) != "0"):
                    # Catch the magic exit value, return 0
                    if("{0}".format(err) == str(exitFuncMagic)):
                        sys.exit(0)

                    # Return error
                    sys.exit(int("{0}".format(err)))

                # Continue if we get a different code


    # Start the terminal
    while True:
        # Get the terminal input
        astr = input('>>> ')
        try:
            # Parse the input and execute the appropriate function
            args = terminal.parse_args(astr.split())
            try:
                args.func(args)
            except AttributeError:
                continue

        # Catch SystemExit, allows user to ctrl-c to quit the current command
        except SystemExit as err:
            if("{0}".format(err) != "0"):
                # Catch the magic exit value, return 0
                if("{0}".format(err) == str(exitFuncMagic)):
                    sys.exit(0)

                # Return error
                sys.exit(int("{0}".format(err)))

            # Continue if we get a different code
