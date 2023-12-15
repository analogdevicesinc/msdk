# bmp2c
This utility converts a 24 bit color bitmap, or a jpeg image to a RGB565 format and generates C code array that can be used for the ILI9341 320x240 TFT LCD controller.


```bash
Usage:
 $python bmp2c.py image_24bit.bmp [-r][-s][-f][-m]
 $python bmp2c.py image.jpg [-r][-s][-f][-m]
  -r: rotate 90 degrees
  -s: rescale to fit the TFT size (320x240)
  -f: flip vertically
  -m: mirror horizontally
```

Examples:

1. Convert JPG image (use option  `-f` by default)

```python
 $python bmp2c.py image_pattern.jpg -f
```

2. Convert BMP image (use option  `-f` by default)

```python
 $python bmp2c.py img_1.bmp -f
```

3. Convert BMP image and rescale (use option  `-f` by default)

```python
 $python bmp2c.py logo_white-bg_darkgrey.bmp -f -s
```

