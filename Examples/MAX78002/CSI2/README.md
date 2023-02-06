## Description

This example captures an image using the camera (e.g. OV5640) via MIPI CSI-2 Protocol and streams either to the on-board TFT display or to your PC through the COM port. Press PB0 to capture an image once the device is ready.

Use the [utils/console.py](utils/console.py) script to grab the camera data and create a png image of the captured image. More information can be found in [utils/README.md](utils/README.md).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

### Project-Specific Build Notes

TODO

## Expected Output

Ex:
```shell
$ python utils/console.py COM6

Started ImgCapture console and opened COM6
MCU: *SYNC*
MCU: Established communications with host!
MCU: Registered 3 total commands:
MCU: -----
MCU: 'help' : Print this help string
MCU: 'reset' : Issue a soft reset to the host MCU.
MCU: 'capture' : Perform a standard blocking capture of a single image
MCU: -----
MCU:
MCU:
MCU: **** MIPI CSI-2 Example ****
MCU: This example streams the image data through the COM port
MCU: and a script running on the host pc converts the data into
MCU: a .png image. Note: You can not run the script and have
MCU: a serial terminal open running on the same COM port at the
MCU: the same time.
MCU:
MCU: Go into the pc_utility folder and run the script:
MCU: python grab_image.py [COM#] [baudrate]
MCU:
MCU: Press PB1 (SW4) to trigger a frame capture.
MCU: Camera ID = 5640
$
```

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
