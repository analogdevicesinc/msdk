## Description

This version of USB_MassStorage configures USB peripheral as Mass Storage device.
The device RAM is emulated as 1GB RAM for USB throughput testing purpose.

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
Hello World!
count : 0
count : 1
count : 2
count : 3
```

You will also observe LED1 blinking at a rate of 2Hz.
