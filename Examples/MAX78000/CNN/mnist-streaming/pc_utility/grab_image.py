import serial
import time
from datetime import datetime
import os
import sys
import string
import comManager
import imgConverter

if len(sys.argv) == 3:
	comport  = sys.argv[1]
	baudRate = sys.argv[2]
else:
	comport  = sys.argv[1]
	# Setup the default baudrate.
	baudRate = 115200

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
