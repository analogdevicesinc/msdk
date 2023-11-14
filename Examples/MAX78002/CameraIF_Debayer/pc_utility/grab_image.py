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
from datetime import datetime
import os
import sys
import string
import comManager
import imgConverter
import cv2

if len(sys.argv) == 3:
	comport  = sys.argv[1]
	baudRate = sys.argv[2]
else:
	comport  = sys.argv[1]
	# Setup the default baudrate.
	baudRate = 921600  # 115200

READ_TIMEOUT = 15

def ByteToHex( ch ):
	try:
		ch = ord(ch)
	except:
		ch = 0
	return ch

def print_sep_line( ch ):
	line = ch * 80
	print(line, flush=True)

#----------------------------------------------

print("Image Reader Started")
print_sep_line('-')

retVal = comManager.init(comport, baudRate)
if retVal != 0:
	print ("comport open failed. Please check %s status\n" % comport);
	sys.exit()

while True:
	if ( comManager.find_sync() == 1 ):
		print ("\n\n***Sync word found***", flush=True)
		print ("Reading image bytes, please wait...", flush=True)

		# width
		arr = comManager.read(2)
		w = arr[0]*256 + arr[1]
		# height
		arr = comManager.read(2)
		h = arr[0]*256 + arr[1]

		pixelformat_len = comManager.read(1)
		pixelformat = comManager.read(pixelformat_len[0])

		arr = comManager.read(4)
		imageLen = arr[0]*256*256*256 + arr[1]*256*256 + arr[2]*256 + arr[3]

		print ("image Len: %d" % imageLen, flush=True);
		print('Image format is ' + pixelformat.decode('ASCII'))
		if ( imageLen > 0 ):
			image = bytearray()
			startTime = time.time();
			timeout = 0
			while (imageLen != len(image)):
				image += comManager.read(imageLen-len(image))
				print ("  Total Read Len:%d bytes" % len(image), flush=True)
				# check timeout
				if ( time.time() > (startTime + READ_TIMEOUT)):
					print("Test Failed: Timeout\n", flush=True);
					timeout = 1
					break

			if (timeout == 1):
				continue

			print ("All image data read", flush=True);
			imgConverter.convert(image, "Image.png", w, h, pixelformat.decode('ASCII'))
			#hex_string = "".join("%02x" % b for b in image)
			#print (hex_string)
			#image_file = open("Image.txt", "w")
			#image_file.write(hex_string)
			#image_file.close()

			image = cv2.imread("image.png")
			cv2.imshow(" ", image)
			cv2.waitKey(1)
			#exit(0)