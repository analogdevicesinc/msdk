## Description

This example captures an image using the camera (e.g. OV5640) via MIPI CSI-2 Protocol and streams either to the on-board TFT display or to your PC through the COM port. Press PB0 to capture an image once the device is ready.

Use the pc_utility/grab_image.py script to grab the camera data and create a png image of the captured image. More information can be found in pc_utility/README.md.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages when streaming to the TFT Display:

```
**** MIPI CSI-2 Example ****
Camera ID detected: 5640

```

Warning: The pc_utility/grab_image.py script connects to the same COM port as USB/UART (Console UART output).
