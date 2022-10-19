## Description

The example demonstartes the use of USB HID driver class. After doing the required connections given below, run the program and a new HID keyboard device appears in the device manager. Open any text editor in the PC, pressing the switch SW2 on the EV Kit will print out 'Maxim Integrated' in the editor one letter at a time.

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
***** MAX78002 USB HID Keyboard Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset
Enumeration complete. Press SW2 to send character.
```
