import time
import os
import sys
import string
import sys
import zlib
import struct
import png
#import the library opencv
import cv2

def swap32(i):
    return struct.unpack("<I", struct.pack(">I", i))[0]

def _clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))

def yuv422_to_rgb(bytesequence):
	img = []
	for i in range(len(bytesequence) // 4):
		offset = i * 4
		byte1 = bytesequence[offset + 0]
		byte2 = bytesequence[offset + 1]
		byte3 = bytesequence[offset + 2]
		byte4 = bytesequence[offset + 3]
		
		Y  = byte1
		U  = byte2
		Y1 = byte3
		V  = byte4
 
		#Y  = byte1
		#U  = byte2
		#Y1 = byte3
		#V  = byte4
 
		#B1 = 1.164 * (y1-16) + 2.018 * (u - 128)
		#G1 = 1.164 * (y1-16) - 0.813 * (v - 128) - 0.391 * (u - 128)
		#R1 = 1.164 * (y1-16) + 1.596*(v - 128)
		#pix[j*2, i] = int(R), int(G), int(B)
 
		#B2 = 1.164 * (y2-16) + 2.018 * (u - 128)
		#G2 = 1.164 * (y2-16) - 0.813 * (v - 128) - 0.391 * (u - 128)
		#2 = 1.164 * (y2-16) + 1.596*(v - 128)
		#pix[j*2+1, i] = int(R), int(G), int(B)

		#R1 = y1 + 1.4075 * (v - 128)
		#G1 = y1 - 0.3455 * (u - 128) - (0.7169 * (v - 128))
		#B1 = y1 + 1.7790 * (u - 128)

		#R2 = y2 + 1.4075 * (v - 128)
		#G2 = y2 - 0.3455 * (u - 128) - (0.7169 * (v - 128))
		#B2 = y2 + 1.7790 * (u - 128)

		R1 = Y + 1.4075 * (V - 128)
		G1 = Y - 0.3455 * (U - 128) - (0.7169 * (V - 128))
		B1 = Y + 1.7790 * (U - 128)

		R2 = Y1 + 1.4075 * (V - 128)
		G2 = Y1 - 0.3455 * (U - 128) - (0.7169 * (V - 128))
		B2 = Y1 + 1.7790 * (U - 128)

		img.append(_clamp(int(R1), 0, 255))
		img.append(_clamp(int(G1), 0, 255))
		img.append(_clamp(int(B1), 0, 255))

		img.append(_clamp(int(R2), 0, 255))
		img.append(_clamp(int(G2), 0, 255))
		img.append(_clamp(int(B2), 0, 255))
	return img

def yuv422_to_blackAndWhite(bytesequence):
	img = []
	for i in range(len(bytesequence) // 4):
		offset = i * 4
		byte1 = bytesequence[offset + 0]
		byte2 = bytesequence[offset + 1]
		byte3 = bytesequence[offset + 2]
		byte4 = bytesequence[offset + 3]
		
		Y  = byte1
		U  = byte2
		Y1 = byte3
		V  = byte4

		r = Y
		g = Y
		b = Y

		img.append(r)
		img.append(g)
		img.append(b)

		r = Y1
		g = Y1
		b = Y1

		img.append(r)
		img.append(g)
		img.append(b)

	return img
	
def blackAndWhite_to_rgb(bytesequence):
	img = []
	for i in range(len(bytesequence)):
		byte1 = bytesequence[i]

		r = byte1
		g = byte1
		b = byte1

		img.append(r)
		img.append(g)
		img.append(b)

	return img

def rgb565_to_rgb(bytesequence):
    img = []
    for i in range(len(bytesequence) // 4):
        offset = i * 4
        byte1 = bytesequence[offset + 0]
        byte2 = bytesequence[offset + 1]
        byte3 = bytesequence[offset + 2]
        byte4 = bytesequence[offset + 3]

        value = byte1 * 0x100 + byte2

        r = (value & 0xf800) >> 11
        g = (value & 0x07e0) >> 5
        b = (value & 0x001f) >> 0

        img.append(r)
        img.append(g)
        img.append(b)

        value = byte3 * 0x100 + byte4

        r = (value & 0xf800) >> 11
        g = (value & 0x07e0) >> 5
        b = (value & 0x001f) >> 0

        img.append(r)
        img.append(g)
        img.append(b)

    return img

def rgb555_to_rgb(bytesequence):
    img = []
    for i in range(len(bytesequence) // 4):
        offset = i * 4
        byte1 = bytesequence[offset + 0]
        byte2 = bytesequence[offset + 1]
        byte3 = bytesequence[offset + 2]
        byte4 = bytesequence[offset + 3]

        value = byte1 * 0x100 + byte2

        r = (value & 0x7C00) >> 10
        g = (value & 0x03e0) >> 5
        b = (value & 0x001f) >> 0

        img.append(r)
        img.append(g)
        img.append(b)

        value = byte3 * 0x100 + byte4

        r = (value & 0x7C00) >> 11
        g = (value & 0x03e0) >> 5
        b = (value & 0x001f) >> 0

        img.append(r)
        img.append(g)
        img.append(b)

    return img
	
def show_image(file):
	#destroy the window
	cv2.destroyAllWindows()
	a = cv2.imread(file)
	# %%%%%%%%%%%%%%%%%%%%%
	#conversion numpy array into rgb image to show
	#c = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)
	cv2.imshow('CameraImage', a)
	#wait for 1 second
	k = cv2.waitKey(1)
	
def convert(bytesequence, outputfile, xres, yres, pixelformat):
	image = []
	

	if (pixelformat == "YUV422"):
		imagepixels = yuv422_to_rgb(bytesequence)
	elif (pixelformat == "RGB555"):
		imagepixels = rgb555_to_rgb(bytesequence)
	elif (pixelformat == "RGB565"):
		imagepixels = rgb565_to_rgb(bytesequence)
	elif (pixelformat == "GRAYSCALE"): #Black and white yuv422
		imagepixels = blackAndWhite_to_rgb(bytesequence)

	offset = 0
	for i in range(yres):
		line = []
		offset = (xres * 3) * i
		for j in range(xres * 3):
			line.append(imagepixels[j + offset])
		image.append(line)

	print("Output image to file xres {}, yres {}".format(xres,yres), flush=True)
	f = open(outputfile, 'wb')      # binary mode is important
	w = png.Writer(xres, yres, greyscale=False)
	w.write(f, image)
	f.close()
	show_image(outputfile)
	