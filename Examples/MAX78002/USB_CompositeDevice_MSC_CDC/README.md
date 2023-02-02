## Description

The example demonstartes the use of USB composite device with Mass Storage driver class and CDC-ACM driver class. After doing the required connections given below, run the program and two new devices appear in the device manager, a Portable device and COM Port. The portable device can be read and written to. Open a terminal application on the PC at 9600 baud rate, and echo the characters on the terminal.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

- For a "quick-start" or for first-time users see ["Getting Started"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#getting-started)
- See ["Development Guide"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#development-guide) for a detailed reference.

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
***** MAX78002 USB Composite Device (CDCACM and Mass Storage) Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset Done: High speed
Enumeration complete...
```
