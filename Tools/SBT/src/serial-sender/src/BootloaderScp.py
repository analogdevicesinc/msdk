#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# src/BootloaderScp.py
#
# ----------------------------------------------------------------------------
# Copyright © 2009-2018, Maxim Integrated Products
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Maxim Integrated Products nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------------------------------------------------------------------------
#
# Created on: Dec 16, 2009
# Author: Grégory Romé <gregory.rome@maxim-ic.com>
# Author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
#

""" Bootloader SCP Classes

:Summary: Bootloader SCP Classes
:Author: Grégory Romé - gregory.rome@maxim-ic.com
:Author: Benjamin VINOT - benjamin.vinot@maximintegrated.com
:Organization: Maxim Integrated Products
:Copyright: Copyright © 2009-2018, Maxim Integrated Products
:License: BSD License - http://www.opensource.org/licenses/bsd-license.php
"""

from struct import unpack
from binascii import hexlify
from utils import *
import serial

__author__ = "Benjamin VINOT <benjamin.vinot@maximintegrated.com>"


class ScpCmdHdr:
    """
    SCP command header
    """

    FMT = '>cccBHBB'
    SIZE = 8

    """SYNC: """
    sync = None
    ctl = None
    dl = None
    id = None
    cks = None
    data_bin = None
    extra = None

    def __init__(self, data=None):
        """
        :Parameter data: data for initialization
        """
        if data is not None:
            self.extra = self.parseData(data)
            if self.extra is not None:
                print >> sys.stderr, "waring: extra data in header"

    def parseData(self, data):
        """
        Fill the SCP command header with `data`

        :Parameter data: data to parse
        """
        self.data_bin = data

        r = unpack(self.FMT, data)
        extra = None
        if len(r) == 9:
            (sync0, sync1, sync2, self.ctl, self.dl, self.id, self.cks, extra) = r
        else:
            (sync0, sync1, sync2, self.ctl, self.dl, self.id, self.cks) = r
        self.sync = (sync0, sync1, sync2)

        return extra

    def __eq__(self, other):
        """
        Implement == operator
        """
        return ((self.data_bin == other.data_bin))

    def __ne__(self, other):
        """
        Implement != operator
        """
        return ((self.data_bin != other.data_bin))

    def __str__(self):
        """
        Implement `str()` method
        """
        ret = 'ScpCmdHdr:\n'
        ret += '  - SYNC: ' + str(self.sync) + '\n'
        ret += '  - CTL: ' + str(self.ctl) + '\n'
        ret += '  - DL: ' + str(self.dl) + '\n'
        ret += '  - ID: ' + str(self.id) + '\n'
        ret += '  - CKS: ' + str(self.cks)
        return ret


class ScpCmd:
    """
    SCP command class
    """

    DATA_FMT = 's'
    CHK_FMT = 'I'

    hdr = None
    data = None
    chk = None
    len = 0
    data_bin = None
    data_len = 0

    def __init__(self, buffer=None):
        """
        :Parameter buffer: a data buffer implementing the method `read(size)`
        """

        if buffer is not None:
            self.parseData(buffer)

    def setHdr(self, hdr):
        """
        Set the SCP command header
        """

        self.hdr = hdr

    def parseData(self, buffer):
        """
        Fill the SCP command with data from `buffer`

        :Parameter buffer: buffer implementing the method `read(size)`
        """

        # Read the header
        data = buffer.read(8)
        if len(data) != 8:
            print >> sys.stderr, "error: expected hdr size != real one"
            print >> sys.stderr, "       real size = " + str(len(data))
            print >> sys.stderr, "       expected size = 8"
            raise Exception()

        self.hdr = ScpCmdHdr()
        self.len = 8
        self.hdr.parseData(data)

        if self.hdr.dl != 0:
            data = buffer.read(self.hdr.dl)
            if len(data) != (self.hdr.dl):
                print >> sys.stderr, "error: expected data size != real one"
                print >> sys.stderr, "       real size = " + str(len(data))
                print >> sys.stderr, "       expected size = " + str((self.hdr.dl))
                raise Exception()

            self.data_bin = data
            self.data_len = self.hdr.dl

            #self.data = unpack(str(self.hdr.dl) + self.DATA_FMT, data)
            self.data = hexlify(data)

            data = buffer.read(4)
            if len(data) != (4):
                print >> sys.stderr, "error: expected chk size != real one"
                print >> sys.stderr, "       real size = " + str(len(data))
                print >> sys.stderr, "       expected size = " + str(4)
                raise Exception()

            #r = unpack(self.CHK_FMT, data)
            #(self.chk,) = r
            self.chk = hexlify(data)

        else:
            self.data = None
            self.chk = None


    def __eq__(self, other):
        """
        Implement == operator
        """
        return ((self.data_bin == other.data_bin) and
                (self.hdr.data_bin == other.hdr.data_bin))

    def __ne__(self, other):
        """
        Implement != operator
        """
        return ((self.data_bin != other.data_bin) or
                (self.hdr.data_bin != other.hdr.data_bin))

    def __str__(self):
        """
        Implement `str()` method
        """

        ret = 'ScpCmd:\n'
        ret += str(self.hdr) + '\n'
        ret += 'Data:\n' + str(self.data) + '\n'
        ret += 'Checksum: ' + str(self.chk)
        return ret


class BootloaderScp(serial.Serial):
    """
    BootloaderScp extends `Serial` by adding functions to read SCP commands.
    """

    def __init__(self, port, options):
        """
        :Parameter port: serial port to use (/dev/tty* or COM*)
        """

        if options.serial_parity == 'none':
            parity = serial.PARITY_NONE
        elif options.serial_parity == 'odd':
            parity = serial.PARITY_ODD
        elif options.serial_parity == 'even':
            parity = serial.PARITY_EVEN
        elif options.serial_parity == 'mark':
            parity = serial.PARITY_MARK
        elif options.serial_parity == 'space':
            parity = serial.PARITY_SPACE
        elif options.serial_parity == 'names':
            parity = serial.PARITY_NAMES
        else:
            print("Unknown parity mode " + options.serial_parity + " Using NONE")
            parity = serial.PARITY_NONE

        if options.serial_stopbits == 1:
            stopbits = serial.STOPBITS_ONE
        elif options.serial_stopbits == 1.5:
            stopbits = serial.STOPBITS_ONE_POINT_FIVE
        elif options.serial_stopbits == 2:
            stopbits = serial.STOPBITS_TWO
        else:
            print("Unknown number of STOP bit  " + options.serial_stopbits + " Using 1 STOP bit")
            stopbits = serial.STOPBITS_ONE

        if options.serial_bytesize == 5:
            bytesize = serial.FIVEBITS
        elif options.serial_bytesize == 6:
            bytesize = serial.SIXBITS
        elif options.serial_bytesize == 7:
            bytesize = serial.SEVENBITS
        elif options.serial_bytesize == 8:
            bytesize = serial.EIGHTBITS
        else:
            print("Unknown byte size " + options.serial_bytesize + " Using 8 bits per byte")
            bytesize = serial.EIGHTBITS

        serial.Serial.__init__(self, port=port,
                               baudrate=options.serial_baudrate,
                               timeout=options.timeout,
                               bytesize=bytesize,
                               stopbits=stopbits,
                               rtscts=options.serial_rtscts,
                               dsrdtr=options.serial_dsrdtr,
                               parity=parity,
                               xonxoff=options.serial_xonxoff)

        if sys.platform == 'linux':
            import fcntl
            fcntl.flock(self.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB )
        self.dtr = True

    def writePacket(self, packet):

        print ScpCmd(packet)
        self.timeout = 1

        return self.write(packet.read())


    def readPacket(self, quiet):
        """
        Read a full packet
        """

        scp_cmd = ScpCmd()

        hdr = self.readHeader(quiet)
        if hdr is None:
            print >> sys.stderr, "error: no header"
            raise Exception()

        scp_cmd.setHdr(hdr)
        scp_cmd.len = 8

        if hdr.dl != 0:
            data = self.read(hdr.dl)

            if len(data) != hdr.dl:
                print_err("Error: expected data size != real one")
                print_err("\t\t real size = " + str(len(data)))
                print_err("\t\t expected size = " + str(hdr.dl))
                raise Exception()

            scp_cmd.len += hdr.dl
            scp_cmd.data = hexlify(data)
            scp_cmd.data_bin = data

            data = self.read(4)
            if len(data) != 4 and scp_cmd.hdr.ctl != 12:
                print_err("Error: expected chk size != real one")
                print_err("\t\t real size = " + str(len(data)))
                print_err("\t\t expected size = 4")
                raise Exception()

            scp_cmd.len += 4
            scp_cmd.chk = hexlify(data)

        return scp_cmd

    def readHeader(self, quiet):
        """
        Read the packet header
        """

        scp_hdr = ScpCmdHdr()

        data = self.read(scp_hdr.SIZE)
        if len(data) == 0:
            if not quiet:
                print_err("Error: timeout, no packet received")
            raise Exception()

        if len(data) != scp_hdr.SIZE:
            if not quiet:
                print_err("Error: expected hdr size != real one")
                print_err("\t\t real size = " + str(len(data)))
                print_err("\t\t expected size = " + str(scp_hdr.SIZE))
            raise Exception()

        scp_hdr.parseData(data)

        return scp_hdr

    def readDump(self, filename):
        print filename
        if filename is not None:
                f = open(filename, 'w')

        while True:
            data = self.read(1)
            if data == chr(4):
                if filename is not None:
                    f.close()
                break
            if filename is not None:
                f.write(data)

            sys.stdout.write(data)
