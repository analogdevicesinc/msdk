#!/usr/bin/env python
# coding: utf-8
import sys
import crc8
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import serial
from utils import *

if len(sys.argv) == 4:
    image_path = sys.argv[1]
    ser = serial.Serial(sys.argv[2])
    ser.baudrate = sys.argv[3]
else:
    image_path = 'image1.png'
    ser = serial.Serial('COM52')
    ser.baudrate = 115200 # Setup the default baudrate
    
pattern_enable = False # If True, it overwrites the image with a fix pattern (for debugging)
ser.timeout = 0.001
output_filename = 'CNNout_sampledata.txt'

# read the original image and display it
im = Image.open(image_path)
img = np.asarray(im)
print(img.min(), img.max())

# resize the image
newsize = (352, 352)
im_resize = im.resize((newsize), Image.LANCZOS)
img_resize = np.asarray(im_resize)[:, :, :3]

# set to True for testing pattern
if pattern_enable:
    img_resize1=np.ndarray(shape=(352,352,3), dtype=np.uint8)
    for i in range(352):
        for j in range(352):
            for k in range (3):
                if k== 0:
                    img_resize1[i, j, k] = 133
                if k ==1:
                    img_resize1[i, j, k] = j >> 1
                if k == 2:
                    img_resize1[i, j, k] = i >> 1
else:
    img_resize1 = img_resize

# fold the image
folded_img = fold_image(img_resize1, 4)
folded_img = np.transpose(folded_img, (2, 0, 1))

# send the folded image to device and save the received result to output_filename
print(folded_img.min(), folded_img.max())

send_image_receive_output(ser, folded_img, output_filename)  # send image and receive CNN output

# read from output_filename and create the predicted map
colors = read_output_from_txtfile(output_filename)

f, ax = plt.subplots(1, 3, figsize=(12, 4))
ax[0].imshow(img_resize1)
ax[1].imshow(colors)
ax[2].imshow(img_resize1)
ax[2].imshow(colors, alpha=0.2)
plt.show() 

