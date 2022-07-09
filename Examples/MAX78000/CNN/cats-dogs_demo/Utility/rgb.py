from PIL import Image
import numpy as np
import sys
import cv2

desc = """
This file is deprecated!  Use rgb128.py instead.
"""
print(desc)

np.set_printoptions(threshold=sys.maxsize)
im = (Image.open('dog.jpg'))
size = (64*2,64*2)
img = im.resize(size)
#img = ImageOps.fit(im, size, Image.ANTIALIAS)
np1_img = np.array(img)
new_img = Image.fromarray(np1_img)
new_img.save("new_img.png")

src = cv2.imread('new_img.png', cv2.IMREAD_UNCHANGED)

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

arr_result = []

# 0x00bbggrr
for i in range(len(red_f)):
	result = red_f[i] | blue_f[i]<<8 | green_f[i]<<16
	arr_result.append((result))
	
#convert list to numpy array
out_arr_result = np.asarray(arr_result, dtype=np.uint32)

#Write out data to the header file
with open('sampledata.h', 'w') as outfile:
	outfile.write('#define SAMPLE_INPUT_0 { \\')
	outfile.write('\n')

	for i in range(len(out_arr_result)):
		if i==0:
			outfile.write('\t0x{0:08x},\t'.format((out_arr_result[i])))

		else :
			d = i%8
			if(d!=0):
				outfile.write('0x{0:08x},\t'.format((out_arr_result[i])))
			else:
				outfile.write('\\')
				outfile.write('\n\t')
				outfile.write('0x{0:08x},\t'.format((out_arr_result[i])))

	outfile.write('\\')			
	outfile.write('\n')
	outfile.write('}')
	outfile.write('\n')

sys.stdout.close()

