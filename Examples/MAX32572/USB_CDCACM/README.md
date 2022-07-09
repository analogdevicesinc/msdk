## Description

The example demonstartes the use of USB CDC-ACM driver class. After doing the required connections given below, run the program and a new serial terminal (COM port) appears in the device manager. Open a terminal application on the PC at 9600 baud rate, and echo the characters on the terminal.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32572 USB CDC-ACM Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset
Enumeration complete. Waiting for characters...
```
