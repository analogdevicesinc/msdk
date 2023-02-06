## Description

The example demonstartes the use of USB Mass Storage driver class. After doing the required connections given below, run the program and a new portable device appears in the device manager. It will appear as a new drive which can be read and written to.
The device RAM is emulated as 1GB RAM for USB throughput testing purpose.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

### Project-Specific Build Notes

(None)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a USB cable between the PC and the CN1 (USB) connector.
-   Default EV kit hardware configuration.


## Expected Output

The Console UART of the device will output these messages:

```
***** MAX78002 USB Mass Storage Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset Done: High speed
Enumeration complete.
```
