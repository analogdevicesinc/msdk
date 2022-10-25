from PIL import Image
import numpy as np

img = Image.open("kodim07.png")
rgb = np.asarray(img)
bayer = np.zeros(rgb.shape, dtype=np.uint8)
print(rgb.shape)
print(bayer.shape)

for x in range(rgb.shape[0]):
    for y in range(rgb.shape[1]):
        if x & 0b1: # Odd row, G R G R ...
            if not y & 0b1:
                # bayer[x][y][0] = rgb[x][y][1]
                bayer[x][y][1] = rgb[x][y][1]
                # bayer[x][y][2] = rgb[x][y][1]
            else: 
                bayer[x][y][0] = rgb[x][y][0]
                # bayer[x][y][1] = rgb[x][y][0]
                # bayer[x][y][2] = rgb[x][y][0]
        else: # Even row, B G B G ...
            if not y & 0b1:
                # bayer[x][y][0] = rgb[x][y][2]
                # bayer[x][y][1] = rgb[x][y][2]
                bayer[x][y][2] = rgb[x][y][2]
            else: 
                # bayer[x][y][0] = rgb[x][y][1]
                bayer[x][y][1] = rgb[x][y][1]
                # bayer[x][y][2] = rgb[x][y][1]

# print(bayer)

img_bayer = Image.fromarray(bayer, mode="RGB")
img_bayer.save("kodim07-bayer.png")
