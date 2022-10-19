## Description

The example demonstartes the use of USB CDC-ACM driver class. After doing the required connections given below, run the program and a new serial terminal (COM port) appears in the device manager. Open a terminal application on the PC at 9600 baud rate, and echo the characters on the terminal.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector. Make sure JP11 (2-3) is connected to UART.
-   Default EV kit hardware configuration.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32650 USB CDC-ACM Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset Done: High speed
Enumeration complete...
```
