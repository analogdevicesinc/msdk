###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 # (now owned by Analog Devices, Inc.)
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
 ##############################################################################
 #
 # Copyright 2023 Analog Devices, Inc.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ##############################################################################

import serial
import time
import os
import sys
import string

ser = serial.Serial()

def ByteToHex( ch ):
	try:
		ch = ord(ch)
	except:
		ch = 0

	return ch

def init(comPort, baudRate):
	#ser = serial.Serial('COM6', 115200, timeout=0)
	global ser
	ser.baudrate = baudRate
	ser.port = str(comPort)
	ser.timeout=3
	try:
		ser.open()
	except:
		None
	if ser.is_open:
		return 0
	else:
		return 1

magic_str = "*RDY*"
start_str = ""


PRTCL_STATE_FREE  = 0
PRTCL_STATE_1	  = 1,
PRTCL_STATE_2	  = 2,
PRTCL_STATE_3	  = 3,
PRTCL_STATE_4	  = 4,
PRTCL_STATE_FOUND = 5,

sync_word_state = PRTCL_STATE_FREE

def find_sync( ):
	global start_str
	global sync_word_state
	isFind = 0

	if True:
		for x in range(1, 1000):
			byt = ser.read(1)
			if sync_word_state == PRTCL_STATE_FREE:
				if (b'*' == byt):
					sync_word_state = PRTCL_STATE_1

			elif sync_word_state == PRTCL_STATE_1:
				if (b'S' == byt):
					sync_word_state = PRTCL_STATE_2
				elif (b'*' == byt):
					sync_word_state = PRTCL_STATE_1
				else:
					sync_word_state = PRTCL_STATE_FREE

			elif sync_word_state == PRTCL_STATE_2:
				if (b'T' == byt):
					sync_word_state = PRTCL_STATE_3
				elif (b'*' == byt):
					sync_word_state = PRTCL_STATE_1
				else:
					sync_word_state = PRTCL_STATE_FREE

			elif sync_word_state == PRTCL_STATE_3:
				if (b'R' == byt):
					sync_word_state = PRTCL_STATE_4
				elif (b'*' == byt):
					sync_word_state = PRTCL_STATE_1
				else:
					sync_word_state = PRTCL_STATE_FREE

			elif sync_word_state == PRTCL_STATE_4:
				if (b'*' == byt):
					sync_word_state = PRTCL_STATE_FREE
					isFind = 1
					break
				else:
					sync_word_state = PRTCL_STATE_FREE
	else:
		for x in range(1, 1000):
			byt = ser.read(1)

			if ((ByteToHex(byt) < 32) or  (ByteToHex(byt) > 127) ):
				byt = '-'

			try:
				ch = byt.decode("ascii")
			except :
				ch = str(byt)

			if (len(start_str) == 5):
				start_str = start_str[1:] + ch
			else:
				start_str = start_str[0:] + ch

			if start_str == magic_str:
				isFind = 1
				break
	return isFind

def read( expectedLen ):
	data=bytearray()
	data = ser.read(expectedLen)
	return data
