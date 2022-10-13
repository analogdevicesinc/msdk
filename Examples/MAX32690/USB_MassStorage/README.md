## Description

The example demonstartes the use of USB Mass Storage driver class. After doing the required connections given below, run the program and a new portable device appears in the device manager. It will appear as a new drive which can be read and written to.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32690 USB Mass Storage Example *****
Waiting for VBUS...
VBUS Connect
Suspended
Bus Reset
Bus Reset Done: High speed
Enumeration complete.
```
