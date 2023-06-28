## Description

This example captures images using the camera (OV7692) and streams either to the on-board TFT display or to your PC through the COM port.

Use the pc_utility/grab_image.py script to grab the camera data and create a png image of the captured image. More information can be found in [pc_utility/README.md](pc_utility/README.md).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This example supports the following cameras:

* OV7962 (Default)
* HM0360 (MONO), for color use the CameraIF_Debayer example
* OV5640
* HM01B0

To change the camera drivers set the `CAMERA` build configuration variable in [project.mk](project.mk)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP20 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 921600, 8-N-1.
-   Ensure JP41 (CSI CAM I2C EN) is _disconnected_.
-   Ensure JP35 (I2C1 SDA) and JP36 (I2C1 SCL) are _disconnected_.
-   Close Jumper JP42 (TFT_DC).
-   Close Jumper JP43 (TFT_CS) to P0_3.

Additionally, ensure JP38 (DVP CAM PWR) is connected properly.
-   OV7692:  Connect JP38 to "ON"
-   HM0360:  Connect JP38 to "OFF"

Consult your camera module's power-down logic and the MAX78002EVKIT schematic for other camera modules.  


## Expected Output

The Console UART of the device will output these messages when streaming to the TFT Display:

```
CameraIF Example
Camera I2C slave address: 3c
Camera ID detected: 7692
Capture image
```

Warning: The pc_utility/grab_image.py script connects to the same COM port as USB/UART (Console UART output).
