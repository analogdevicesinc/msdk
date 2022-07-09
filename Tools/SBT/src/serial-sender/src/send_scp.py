#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#
# src/serial_sender.py
#
# ----------------------------------------------------------------------------
# Copyright © 2009-2018, Maxim Integrated Products
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 	 * Redistributions in binary form must reproduce the above copyright
# 	   notice, this list of conditions and the following disclaimer in the
# 	   documentation and/or other materials provided with the distribution.
# 	 * Neither the name of the Maxim Integrated Products nor the
# 	   names of its contributors may be used to endorse or promote products
# 	   derived from this software without specific prior written permission.
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
# ----------------------------------------------------------------------------
#
# Author: Grégory Romé <gregory.rome@maxim-ic.com>
# Author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>

u""" Usage: serial_sender [options] FILENAME

Options:
   | --version				  show program's version number and exit
   | -h, --help				 show this help message and exit
   | -s SERIAL, --serial=SERIAL define the serial port to use
   | -v, --verbose			  enable verbose mode
   | -l, --license			  display license
   | --list-serial			  display available serial ports
   | -b, --bl-emulation	      emulate the bootloader

serial_send sends signed packets to the bootloader on the serial link. This
tool can be used as test tool for bootloader and validation tool. FILENAME
contains both sended and received packets. Those last ones are used for
verification.

:Summary: Bootloader SCP commands sender
:Version: 2.3.0

:Author: Grégory Romé - gregory.rome@maxim-ic.com
:Author: Benjamin VINOT - benjamin.vinot@maximintegrated.com
:Organization: Maxim Integrated Products
:Copyright: Copyright © 2009-2018, Maxim Integrated Products
:License: BSD License - http://www.opensource.org/licenses/bsd-license.php
"""

# ---- IMPORTS


import re
import time
import progressbar
import serial

from optparse import OptionParser, OptionGroup
from serial.tools import list_ports
from scan import scan
from ScpPacket import *
from utils import *
import colorama


# ---- CONSTANTS

AUTHOR = u"Grégory Romé, Benjamin VINOT"

VERSION = "2.3.0"
__version__ = "2.3.0"
PROG = "serial_sender"

COPYRIGHT = u"Copyright © 2009-2018, Maxim Integrated Products"

LICENSE = u"""Copyright © 2009-2018, Maxim Integrated Products
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the Maxim Integrated Products nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."""

EPILOG = """serial_send sends signed packets to the bootloader on the serial
link. This tool can be used as test tool for bootloader and validation tool.
FILENAME contains both sended and received packets. Those last ones are used for
verification"""

# ---- GLOBALS

_ERROR_MSG = [['  ' for i in range(40)] for j in range(20)]


def get_extra_script():

	if 'MAXIM_SBT_DIR' not in os.environ or 'MAXIM_SBT_DEVICE' not in os.environ :
		print_err("Environment Variable 'MAXIM_SBT_DIR' and/or  'MAXIM_SBT_DEVICE' not correctly set.")
		raise Error

	if options.verbose >= DEBUG:
		print "Env MAXIM_SBT_DIR :" + os.environ['MAXIM_SBT_DIR']
		print "Env MAXIM_SBT_DEVICE :" + os.environ['MAXIM_SBT_DEVICE']
		print "Param 'Extra_Script' :" + options.extra_script

	packets = os.path.join(os.environ['MAXIM_SBT_DIR'], "devices", os.environ['MAXIM_SBT_DEVICE'], "scp_packets", options.extra_script, "packet.list")

	if options.verbose >= DEBUG:
		print "Packets Path" + packets

	if os.path.isfile(packets):
		return packets
	else:
		print_err("Requested Extra Script packets does not exist " + str(packets) )
		raise FileError()


def parse_scpcmd_file(filename, options):
	"""
	:param filename:
	:param options:
	:return:
	"""

	if options.verbose >= VERBOSE:
		print 'Open file: ' + filename

	cmds_file = open(filename, 'r')
	file_dir = os.path.dirname(filename)

	packet_list = []

	# Get number of packets to send
	for line in cmds_file:
		file_name = line.strip()
		s_m = re.search('(\w+[_-]*\w+)\.(\d+)\.(\w+[_-]*\w+)\.((\w+[_-]*)*)\.\w+', file_name)
		if s_m is not None:
			id = s_m.group(2)
			cmd = s_m.group(4)
			way_str = s_m.group(3)
		else:
			print_err("error: wrong filename: " + file_name)
			raise FileError()

		if way_str == 'bl':
			way = False ^ options.bl_emulation
		elif way_str == 'host':
			way = True ^ options.bl_emulation
		else:
			print_err("error: wrong filename: " + file_name)
			raise FileError()

		if cmd == "connection_request" or cmd == "connection_reply":
			packet = ConnectionPacket(file_dir, file_name, options, cmd, id, way)
		elif cmd == "hello_reply":
			packet = HelloReplyPacket(file_dir, file_name, options, cmd, id, way)
		elif cmd == "erase_mem" or cmd == "del_mem":
			packet = ErasePacket(file_dir, file_name, options, cmd, id, way)
		elif cmd == "dump":
			packet = DumpPacket(file_dir, file_name, options, cmd, id, way)
		elif cmd == "write_crk_response":
			packet = WriteCRKPacket(file_dir, file_name, options, cmd, id, way)
		else:
			packet = ScpPacket(file_dir, file_name, options, cmd, id, way)

		packet_list.append(packet)

	cmds_file.close()

	return packet_list


def process_packet(packet_list, options, bl_scp):

	if options.verbose >= VERBOSE:
		print 'Start SCP session (use -v for details)'

	# Get the connection packets
	con_req = packet_list[0]
	con_reply = packet_list[1]
	con_ack = packet_list[2]

	if options.auto_reset:
		if options.verbose >= VERBOSE:
			print 'Reset Board throw UART'
		bl_scp.setRTS(True)
		time.sleep(0.5)
		bl_scp.setRTS(False)

	if options.verbose >= VERBOSE:
		print 'Trying to Connect.'

	bbar = progressbar.ProgressBar(widgets=[progressbar.AnimatedMarker()], maxval=options.first_retry_nb - 1).start()
	for i in bbar((i for i in range(options.first_retry_nb))):
		try:
			con_req.process(bl_scp)
			con_reply.process(bl_scp)
			break
		except KeyboardInterrupt:
			print "Keyboard Interruption : Closing connection !"
			raise Exception()
		except Exception as inst:
			pass

	if options.mpc:
				mpc_status(2, len(packet_list))

	con_ack.process(bl_scp)

	if options.verbose >= VERBOSE:
		print '\nConnected !'

	if options.mpc:
				mpc_status(3, len(packet_list))
	current = 0
	if options.verbose <= VERBOSE and not options.mpc:
		bar = progressbar.ProgressBar(maxval=(len(packet_list) - 5),
									  widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()]).start()

	for packet in packet_list[3:-2]:
		try:
			packet.process(bl_scp)
			current += 1
			if options.verbose <= VERBOSE and not options.mpc:
				bar.update(current)
			if options.mpc:
				mpc_status(current + 3, len(packet_list))
		except Exception as insts:
			raise

	if options.verbose <= VERBOSE and not options.mpc:
		bar.finish()

	if options.verbose >= VERBOSE:
		print '\nDisconnecting...'

	decon_req = packet_list[-2]
	decon_reply = packet_list[-1]
	try:
		decon_req.process(bl_scp)
		if options.mpc:
				mpc_status(current + 4, len(packet_list))
		decon_reply.process(bl_scp)
		if options.mpc:
				mpc_status(current + 5, len(packet_list))
		else:
			print 'Disconnected !'
	except Exception as insts:
		raise


# ---- MAIN

if __name__ == "__main__":
	colorama.init()
	return_code = 0
	m = re.search('Rev(ision)*\s*:\s*(\d+)', __version__)
	if m is not None:
		revision = m.group(2)
	else:
		revision = "unknown"

	usage = "usage: " + PROG + " [options] FILENAME"
	version = "%prog " + VERSION + '-rev' + revision + "\n" + COPYRIGHT

	parser = OptionParser(prog=PROG, usage=usage, version=version, epilog=EPILOG)
	parser.add_option("-s", "--serial", dest="serial", type="string", help="define the serial port to use")
	parser.add_option("-x", "--extra", dest="extra_script", type="string", help="Send ready to use packet provided by Maxim")
	parser.add_option("-v", action="count", dest="verbose", help="enable verbose mode")


	parser.add_option("-l", "--license", action="store_true", dest="license", default=False, help="display license")
	parser.add_option("--list-serial", action="store_true", dest="list_serial", default=False, help="display available serial ports")

	group = OptionGroup(parser, "Timming Options")
	group.add_option("-t", "--timeout", dest="timeout", type="int", default=10, help="specifies the protocol timeout (s). \
By default the timeout is 10s")

	group.add_option("-e", "--erase-timeout", dest="erase_timeout", type="int", default=5, help="specifies the protocol erase mem command timeout (s). \
By default the timeout is 5s")

	parser.add_option_group(group)

	parser.add_option("-f", "--first-retry", dest="first_retry_nb", type="int",
					  default=200, help="specifies the number of retry for first packet. \
By default the number is 200")

	group = OptionGroup(parser, "Extra Options")

	group.add_option("--auto-reset", action="store_true", dest="auto_reset", default=False, help="Perform a reset throw UART RTS before SCP session")
	group.add_option("-b", "--bl-emulation", action="store_true", dest="bl_emulation", default=False, help="emulate the bootloader")
	group.add_option("-m", "--mpc", action="store_true", dest="mpc", default=False, help="Activate mpc standard output")

	group.add_option("-d", "--dump-file", dest="dump_filename", help="write dump to FILE", metavar="FILE")

	group.add_option("-c", "--chip", dest="chip_name", help="Force CHIP selection for error identification", metavar="CHIP")

	parser.add_option_group(group)

	group = OptionGroup(parser, "Serial Options")

	group.add_option("--serial-baudrate", dest="serial_baudrate", type="int",
					  default=115200, help="specifies the serial baudrate. \
						By default the baudrate is 115200")

	parser.add_option("--serial-rtscts", action="store_true", dest="serial_rtscts",
					  default=False, help="Enable serial HW flow control")

	parser.add_option("--serial-dsrdtr", action="store_true", dest="serial_dsrdtr",
					  default=False, help="Enable serial HW flow control")

	parser.add_option("--serial-xonxoff", action="store_true", dest="serial_xonxoff",
					  default=False, help="Enable serial HW flow control")

	group.add_option("--serial-bytesize", dest="serial_bytesize", type="int",
					  default=8, help="specifies the serial bytesize. \
						By default the bytesize is 8")

	group.add_option("--serial-stopbits", dest="serial_stopbits", type="int",
					  default=1, help="specifies the serial number of stop bits. \
						By default the number of stop bits is 1")
	group.add_option("--serial-parity", dest="serial_parity",default="none",
					 help="Serial parity( none, odd, even, mark, space) default None")

	parser.add_option_group(group)

	(options, args) = parser.parse_args()
	if options.license:
		print LICENSE
		sys.exit(0)

	if options.list_serial:
		print "Available serial ports:"
		for port_name in scan():
			print '  - ' + port_name
		sys.exit(0)

	try:

		bl_scp = None

		if options.extra_script is not None:
			filename = get_extra_script()
		else:

			if args.__len__() != 1:
				parser.error("argument(s) missing")
				sys.exit(-1)

			filename = args[0]

		if os.path.isdir(filename):
			filename = os.path.join(filename, "packet.list")

		if options.serial is None:
			if os.name == 'nt':
				serial = 'COM1'
			else:
				serial = '/dev/ttyS0'
		else:
			serial = options.serial

		port_name = serial

		if options.verbose >= VERBOSE:
			print 'Open serial port: ' + port_name + ' (timeout: ' + str(options.timeout) + 's)'

		packets_list = parse_scpcmd_file(filename, options)

		while True:
			try:
				if sum(1 for item in iter(list_ports.grep(serial))) == 0:
					if not options.mpc:
						print 'Waiting for device ' + serial + ' to appears'
				while sum(1 for item in iter(list_ports.grep(serial))) == 0:
					pass
				time.sleep(0.8)
				bl_scp = BootloaderScp(port_name, options)
				process_packet(packets_list, options, bl_scp)

				break
			except FlashCRKWriteError:
				bl_scp.close()
				bl_scp = BootloaderScp(port_name, options)
				packets_list[0].process(bl_scp)
				bl_scp.close()

			except Exception:
				raise
		if not options.mpc:
			print_ok("SCP session OK")
	except ValueError:
		if not options.mpc:
			print_err("Connection Failed")
		return_code = -2
	except Error:
		print "An error Happened, Quiting..."
	except Exception as inst:
		if not options.mpc:
			print inst
			print_err("error: SCP session FAILED")
		return_code = -1

	finally:
		if bl_scp is not None:
			bl_scp.close()

	sys.exit(return_code)


