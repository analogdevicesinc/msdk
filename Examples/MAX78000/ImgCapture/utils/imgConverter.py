################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

import struct
from PIL import Image

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

def rgb888_to_rgb(bytesequence):
	img = []
	for i in range(len(bytesequence) // 3):
		offset = i * 3
		byte1 = bytesequence[offset + 0]
		byte2 = bytesequence[offset + 1]
		byte3 = bytesequence[offset + 2]

		r = byte1
		g = byte2
		b = byte3

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
		byte1 = bytesequence[offset + 2]
		byte2 = bytesequence[offset + 3]
		byte3 = bytesequence[offset + 0]
		byte4 = bytesequence[offset + 1]

		pixel1 = byte1 * 0x100 + byte2
		r1 = (pixel1 >> 11) & 0x1f
		g1 = (pixel1 >> 5) & 0x3f
		b1 = (pixel1 >> 0) & 0x1f
		r1 = (r1 * 255) / 31
		g1 = (g1 * 255) / 63
		b1 = (b1 * 255) / 31

		pixel2 = byte3 * 0x100 + byte4
		r2 = (pixel2 >> 11) & 0x1f
		g2 = (pixel2 >> 5) & 0x3f
		b2 = (pixel2 >> 0) & 0x1f
		r2 = (r2 * 255) / 31
		g2 = (g2 * 255) / 63
		b2 = (b2 * 255) / 31

		img.append(int(r2))
		img.append(int(g2))
		img.append(int(b2))

		img.append(int(r1))
		img.append(int(g1))
		img.append(int(b1))

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

#
# generate_img
#
def generate_img(output, color, resolution):
	# img = Image.open(backdrop)
	img = Image.new("RGB", resolution, color)
	img.save(output, quality=100)
	return img

def convert(bytesequence, outputfile, xres, yres, pixelformat):
	image = []

	if (pixelformat == "YUV422"):
		imagepixels = yuv422_to_rgb(bytesequence)
	elif (pixelformat == "RGB555"):
		imagepixels = rgb555_to_rgb(bytesequence)
	elif (pixelformat == "RGB565"):
		imagepixels = rgb565_to_rgb(bytesequence)
	elif (pixelformat == "RGB888"):
		imagepixels = rgb888_to_rgb(bytesequence)
	elif (pixelformat == "GRAYSCALE"): #Black and white yuv422
		imagepixels = blackAndWhite_to_rgb(bytesequence)
	elif (pixelformat == "BAYER"): #Black and white raw
		imagepixels = blackAndWhite_to_rgb(bytesequence)
        
	offset = 0
	for i in range(yres):
		line = []
		offset = (xres * 3) * i
		for j in range(xres * 3):
			line.append(imagepixels[j + offset])
		image.append(line)

	# print("Output image to file xres {}, yres {}".format(xres,yres), flush=True)

	g_pil_image = generate_img(outputfile, (0, 0, 0), (xres, yres))
	x = 0
	y = 0
	for i in range(int(len(imagepixels) / 3)):
		color_r = imagepixels[i * 3 + 0]
		color_g = imagepixels[i * 3 + 1]
		color_b = imagepixels[i * 3 + 2]
		g_pil_image.putpixel( (x, y), (color_r, color_g, color_b, 255))
		x = x + 1
		if x > (xres - 1):
			x = 0
			y = y + 1
			if y > (yres - 1):
				break
	g_pil_image.save(outputfile)


