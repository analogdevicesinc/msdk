from PIL import Image, ImageOps
import numpy as np
import os
import sys
import cv2

np.set_printoptions(threshold=sys.maxsize)
im = (Image.open('./scissor7.png'))
size = (64,64)

img = ImageOps.fit(im, size, Image.ANTIALIAS)
np1_img = np.array(img)
new_img = Image.fromarray(np1_img)
new_img.save("new_img.png")

src = cv2.imread('./new_img.png', cv2.IMREAD_UNCHANGED)

#Get red channel from img
red = src[:,:,2]
red = red - 128
red_f = red.flatten()

#Get green channel from img
green = src[:,:,1]
green = green - 128
green_f = green.flatten()

#Get blue channel from img
blue = src[:,:,0]
blue = blue - 128
blue_f = blue.flatten()

arr_red = []
arr_blue = []
arr_green = []

for i in range(len(red_f)):
	if(i==0):
		result_red = red_f[i]
		result_blue = blue_f[i]
		result_green = green_f[i]
	else:
		d=i%4
		if(d==0):
			result_red |= red_f[i]<<0
			result_blue |= blue_f[i]<<0
			result_green |= green_f[i]<<0
		elif (d==1):
			result_red |= red_f[i]<<8
			result_blue |= blue_f[i]<<8
			result_green |= green_f[i]<<8
		elif (d==2):
			result_red |= red_f[i]<<16
			result_blue |= blue_f[i]<<16
			result_green |= green_f[i]<<16
		elif (d==3):
			result_red |= red_f[i]<<24
			result_blue |= blue_f[i]<<24
			result_green |= green_f[i]<<24

			arr_red.append((result_red))
			arr_blue.append((result_blue))
			arr_green.append((result_green))

			result_red = 0
			result_blue = 0
			result_green = 0

#convert list to numpy array
out_arr_red = np.asarray(arr_red, dtype=np.uint32)
out_arr_blue = np.asarray(arr_blue, dtype=np.uint32)
out_arr_green = np.asarray(arr_green, dtype=np.uint32)

#Write out red channel data to the header file
with open('sampledata.h', 'w') as outfile:
	outfile.write('#define INPUT_0 { \\')
	outfile.write('\n')

	for i in range(len(out_arr_red)):
		if i==0:
			outfile.write('\t0x{0:08x},\t'.format((out_arr_red[i])))

		else :
			d = i%8
			if(d!=0):
				outfile.write('0x{0:08x},\t'.format((out_arr_red[i])))
			else:
				outfile.write('\\')
				outfile.write('\n\t')
				outfile.write('0x{0:08x},\t'.format((out_arr_red[i])))

	outfile.write('\\')
	outfile.write('\n')
	outfile.write('}')
	outfile.write('\n')

#Write out green channel data to the header file
with open('sampledata.h', 'a') as outfile:
	outfile.write('#define INPUT_1 { \\')
	outfile.write('\n')

	for i in range(len(out_arr_green)):
		if i==0:
			outfile.write('\t0x{0:08x},\t'.format((out_arr_green[i])))

		else :
			d = i%8
			if(d!=0):
				outfile.write('0x{0:08x},\t'.format((out_arr_green[i])))
			else:
				outfile.write('\\')
				outfile.write('\n\t')
				outfile.write('0x{0:08x},\t'.format((out_arr_green[i])))

	outfile.write('\\')
	outfile.write('\n')
	outfile.write('}')
	outfile.write('\n')

#Write out blue channel data to the header file
with open('sampledata.h', 'a') as outfile:
	outfile.write('#define INPUT_2 { \\')
	outfile.write('\n')

	for i in range(len(out_arr_blue)):
		if i==0:
			outfile.write('\t0x{0:08x},\t'.format((out_arr_blue[i])))

		else :
			d = i%8
			if(d!=0):
				outfile.write('0x{0:08x},\t'.format((out_arr_blue[i])))
			else:
				outfile.write('\\')
				outfile.write('\n\t')
				outfile.write('0x{0:08x},\t'.format((out_arr_blue[i])))

	outfile.write('\\')
	outfile.write('\n')
	outfile.write('}')
	outfile.write('\n')

sys.stdout.close()

