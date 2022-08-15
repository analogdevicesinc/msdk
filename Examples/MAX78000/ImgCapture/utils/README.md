## Description

This folder contains Python scripts designed for use with the "ImgCapture" firmware.  These include a full-featured console application and image conversion utilites.

## Requirements

Python 3.9+

Then, install Python packages using the `requirements.txt` file in this folder.

Ex:

```shell
pip install -r requirements.txt
```

## Usage

### Console (console.py)

1. Ensure the latest ImgCapture-Serial is flashed to the host MCU and running.

2. Verify that the LED on the host MCU is flashing.  This is the signal that the firmware is searching for the host.

3. Connect the Python console to the host MCU firmware using `console.py`.  Ex:

    ```shell
    $ python console.py <COM port>
    ```

    "<COM port>" should match the port presented by the host MCU.  On the MAX78000FTHR, that will be a single port presented by the integrated CMSIS-DAP debugger.  On the MAX78000EVKIT, that will match the port presented by CN1 (USB/PWR).

    Alternatively, use `python console.py -h` to see a list of console start-up options.

4. Verify a successful connection, which should look similar to the following:

    ```shell
    $ python console.py COM8
    Started ImgCapture console and opened COM8
    MCU: *SYNC*
    MCU: Established communications with host!
    MCU: Registered 7 total commands:
    MCU: -----
    MCU: 'help' : Print this help string
    MCU: 'reset' : Issue a soft reset to the host MCU.
    MCU: 'capture' : Perform a standard blocking capture of a single image
    MCU: 'imgres' <width> <height> : Set the image resolution of the camera to <width> x <height>
    MCU: 'stream' : Performs a line-by-line streaming DMA capture of a single image, capable of higher resolutions
    MCU: 'set-reg' <register> <value> : Write a value to a camera register.
    MCU: 'get-reg' <register> : Prints the value in a camera register.
    MCU: -----
    MCU: Initializing DMA
    MCU: Initializing camera
    MCU: Camera I2C slave address: 3c
    MCU: Camera ID detected: 7fa2
    MCU: Ready!
    $
    ```

    If SD card functionality has been enabled in firmware, an extended command set will be loaded, and your startup sequence should look similar to the following:

    ```shell
    $ python console.py COM8
    Started ImgCapture console and opened COM8
    MCU: *SYNC*
    MCU: Established communications with host!
    MCU: Registered 18 total commands:
    MCU: -----
    MCU: 'help' : Print this help string
    MCU: 'reset' : Issue a soft reset to the host MCU.
    MCU: 'capture' : Perform a standard blocking capture of a single image
    MCU: 'imgres' <width> <height> : Set the image resolution of the camera to <width> x <height>
    MCU: 'stream' : Performs a line-by-line streaming DMA capture of a single image, capable of higher resolutions
    MCU: 'set-reg' <register> <value> : Write a value to a camera register.
    MCU: 'get-reg' <register> : Prints the value in a camera register.
    MCU: 'mount' : Mount the SD card, enabling the commands below.  This will format the SD card if the MCU detects it's blank.
    MCU: 'unmount' : Unmount the SD card.
    MCU: 'cwd' : Print the current working directory (cwd).
    MCU: 'cd' <dir> : Change the current working directory to <dir>.
    MCU: 'ls' : List the contents of the current working directory.
    MCU: 'mkdir' <dir> : Create a directory
    MCU: 'rm' <item> : Remove a file or (empty) directory.
    MCU: 'touch' <filename> : Create an empty file.
    MCU: 'write' <filename> <string> : Write a string to a file.
    MCU: 'cat' <filename> : Print the contents of a file.
    MCU: 'snap' <filename> : Snap an image (using 'stream') and save it to the SD card.  <filename> is optional.  If none is specified, images will be saved to /raw.
    MCU: -----
    MCU: Mounting SD card...
    MCU: SD card mounted.
    MCU: Volume label: MAXIM-SD
    MCU: Initializing DMA      
    MCU: Initializing camera   
    MCU: Camera I2C slave address: 3c
    MCU: Camera ID detected: 7fa2
    MCU: Ready!
    $
    ```

5. Enter any of the registered commands to excercise the example.  Any `capture`-d or `stream`-ed images will be saved to an "Image.png" file next to the script.

### Batch Conversion (batchconvert.py)

This script is useful for converting a large set of images that have been saved to an SD card.  It will attempt to convert all of the images in the specified input directory to .png files in its output directory.

Run `-h` for help: 

```shell
$ python batchconvert.py -h
```

Usage example:

```shell
$ python batchconvert.py D:/raw
Sucessfully converted D:\raw\img0 -> converted\img0.png
Sucessfully converted D:\raw\img1 -> converted\img1.png
Sucessfully converted D:\raw\img10 -> converted\img10.png
Sucessfully converted D:\raw\img11 -> converted\img11.png
Sucessfully converted D:\raw\img12 -> converted\img12.png
Sucessfully converted D:\raw\img13 -> converted\img13.png
Sucessfully converted D:\raw\img14 -> converted\img14.png
Sucessfully converted D:\raw\img15 -> converted\img15.png
Sucessfully converted D:\raw\img16 -> converted\img16.png
Sucessfully converted D:\raw\img2 -> converted\img2.png
Sucessfully converted D:\raw\img3 -> converted\img3.png
Sucessfully converted D:\raw\img4 -> converted\img4.png
Sucessfully converted D:\raw\img5 -> converted\img5.png
Sucessfully converted D:\raw\img6 -> converted\img6.png
Sucessfully converted D:\raw\img7 -> converted\img7.png
Sucessfully converted D:\raw\img8 -> converted\img8.png
Sucessfully converted D:\raw\img9 -> converted\img9.png
$ 
```