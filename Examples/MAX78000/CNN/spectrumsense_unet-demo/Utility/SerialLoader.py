###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2025 Analog Devices, Inc.
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
#!/usr/bin/env python
# coding: utf-8
import sys
import crc8
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches as mpatches
from PIL import Image
import serial
import argparse
from utils import *

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--com', type=str, default='COM52', help='COM Port for MAX78000')
parser.add_argument('-b', '--baud', type=int, default=115200, help='UART Baud Rate')
parser.add_argument('-i', '--input', type=str, default='LTE_frame_12.png', help='Input Image file')
parser.add_argument('-o', '--output', type=str, default='CNNout.txt', help='Output file')
args = parser.parse_args()

image_path = args.input
ser = serial.Serial(args.com)
ser.baudrate = args.baud
output_filename = args.output
pattern_enable = False # If True, it overwrites the image with a fix pattern (for debugging)
ser.timeout = 0.001

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

f, ax = plt.subplots(1, 2, figsize=(12, 4))
ax[0].imshow(img_resize1)
ax[1].imshow(colors)

#Noise is black, LTE is Green, 5G is blue
handles = [mpatches.Patch(color='black', label='Noise/Other'),
           mpatches.Patch(color='#00FF00', label='4G/LTE'),
           mpatches.Patch(color='#0000FF', label='5G/NR')]
f.legend(handles=handles, loc=7)
plt.show()
